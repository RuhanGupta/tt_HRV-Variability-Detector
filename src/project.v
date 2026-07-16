/*
 * tt_um_ruhangupta_hrv_variability_detector
 *
 * Copyright (c) 2026 Ruhan Gupta
 * SPDX-License-Identifier: Apache-2.0
 *
 * SAMPLE-STAGE circuit only. This is a direct Verilog port of a hand-built
 * Wokwi gate diagram used to bring up the Tiny Tapeout toolchain and prove
 * out one building block: the magnitude and direction of the change in
 * pulse-to-pulse interval from one detected beat to the next. It does NOT
 * implement the full fatigue-monitor ASIC (no I2C, no MAX30102, no
 * calibration/baseline, no persistence counter, no alert logic). Those are
 * separate modules built and tested later.
 *
 * What it does
 * ------------
 *  - A free-running 3-bit counter counts main-clock ticks since the last
 *    detected beat (one beat = one pulse on ui_in[0]).
 *  - On each new beat, the counter's value from just before it resets (the
 *    interval that just finished) is latched into a second 3-bit register:
 *    the "previous interval".
 *  - Every cycle, the design continuously computes
 *        diff = (running count since last beat) - (previous interval)
 *    and reports |diff| as a 7-line thermometer code (uo_out[6:0], one bit
 *    per threshold 1..7) plus a direction bit (uo_out[7]): 1 means the
 *    elapsed time is currently running short of the previous interval
 *    (pace speeding up so far this interval); 0 means it has caught up to
 *    or passed it.
 *  - The previous-interval register is also exposed directly for debug on
 *    the bidirectional pins.
 *
 * Two deliberate departures from the literal Wokwi gate diagram, both
 * required to make this synthesizable as a normal single-clock-domain ASIC:
 *
 *  1. The Wokwi version clocked its "previous interval" flip-flops (r0-r2)
 *     directly off the beat button, i.e. off a second, independent clock.
 *     A real chip has one clock net. Here, ui_in[0] is treated as ordinary
 *     data: it is synchronized with a 2-flop synchronizer, and a rising
 *     edge on the synchronized signal triggers the latch, all on `clk`.
 *  2. The Wokwi flip-flops had no reset (dff_cell has none). Real silicon
 *     powers up in an unknown state, so `rst_n` forces known start values.
 *
 * Everything else (the counter's clear-while-beat-held behaviour, the
 * subtract-then-absolute-value, and the threshold decode into a
 * thermometer code) is functionally identical to the Wokwi diagram, just
 * written with arithmetic/comparison operators instead of individual
 * gate instances -- synthesis reduces both to the same kind of hardware.
 *
 * Known limitation carried over unchanged from the Wokwi version: the
 * counter and register are only 3 bits wide (0-7 clock ticks), so an
 * interval longer than 7 clock ticks silently wraps. This is fine for a
 * toolchain/logic bring-up sample and is not something to fix here.
 *
 * Pinout
 * ------
 *  ui_in[0]   beat pulse in (one detected heartbeat = one pulse, active high)
 *  ui_in[7:1] unused
 *
 *  uo_out[0]  |interval difference| >= 1
 *  uo_out[1]  |interval difference| >= 2
 *  uo_out[2]  |interval difference| >= 3
 *  uo_out[3]  |interval difference| >= 4
 *  uo_out[4]  |interval difference| >= 5
 *  uo_out[5]  |interval difference| >= 6
 *  uo_out[6]  |interval difference| >= 7 (max, saturates)
 *  uo_out[7]  direction: 1 = elapsed time still short of previous interval
 *
 *  uio[2:0]   previous interval value (debug, output)
 *  uio[7:3]   unused (driven low)
 */

`default_nettype none

module tt_um_ruhangupta_hrv_variability_detector (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path (unused)
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so it can be ignored
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ------------------------------------------------------------------
  // Synchronize the beat input and detect its rising edge. ui_in[0] is an
  // external, asynchronous-to-clk signal (in the final system it will come
  // from the peak detector; here it stands in for a detected beat), so it
  // is brought through a 2-flop synchronizer before being used for
  // anything, per standard practice for any off-chip input.
  // ------------------------------------------------------------------
  reg [1:0] beat_sync;
  reg       beat_prev;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      beat_sync <= 2'b00;
      beat_prev <= 1'b0;
    end else begin
      beat_sync <= {beat_sync[0], ui_in[0]};
      beat_prev <= beat_sync[1];
    end
  end

  wire beat_level = beat_sync[1];             // synchronized level
  wire beat_edge  = beat_level & ~beat_prev;   // one-cycle rising-edge pulse

  // ------------------------------------------------------------------
  // Free-running interval counter and the captured previous interval.
  // Mirrors the Wokwi diagram's toggle-flop counter (c0-c2) with a
  // level-qualified synchronous clear, and its beat-clocked capture
  // register (r0-r2), now clocked by the same `clk` and gated by beat_edge.
  // ------------------------------------------------------------------
  reg [2:0] cur_interval;
  reg [2:0] prev_interval;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cur_interval  <= 3'd0;
      prev_interval <= 3'd0;
    end else begin
      if (beat_edge)
        prev_interval <= cur_interval;  // snapshot the interval that just finished

      cur_interval <= beat_level ? 3'd0 : (cur_interval + 3'd1);
    end
  end

  // ------------------------------------------------------------------
  // diff = cur_interval - prev_interval, computed continuously.
  // sign     = 1 when cur_interval < prev_interval (elapsed time is still
  //            short of the previous interval).
  // abs_diff = |diff|, 0..7.
  // Mirrors the Wokwi diagram's 3-bit subtractor + sign + abs-value gates.
  // ------------------------------------------------------------------
  wire signed [3:0] diff     = $signed({1'b0, cur_interval}) - $signed({1'b0, prev_interval});
  wire              sign     = diff[3];
  wire       [2:0]  abs_diff = sign ? (~diff[2:0] + 3'd1) : diff[2:0];

  // ------------------------------------------------------------------
  // Thermometer-code decode of abs_diff, matching the Wokwi threshold
  // gates that drove the 8-LED bargraph.
  // ------------------------------------------------------------------
  wire ge1 = abs_diff >= 3'd1;
  wire ge2 = abs_diff >= 3'd2;
  wire ge3 = abs_diff >= 3'd3;
  wire ge4 = abs_diff >= 3'd4;
  wire ge5 = abs_diff >= 3'd5;
  wire ge6 = abs_diff >= 3'd6;
  wire ge7 = abs_diff >= 3'd7;

  // ------------------------------------------------------------------
  // Outputs
  // ------------------------------------------------------------------
  assign uo_out = {sign, ge7, ge6, ge5, ge4, ge3, ge2, ge1};

  assign uio_out = {5'b00000, prev_interval};
  assign uio_oe  = {5'b00000, 3'b111};

  // Silence unused-signal lint warnings
  wire _unused = &{ena, uio_in, ui_in[7:1], 1'b0};

endmodule
