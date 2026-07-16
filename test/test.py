# SPDX-FileCopyrightText: © 2026 Ruhan Gupta
# SPDX-License-Identifier: Apache-2.0
#
# Tests for the HRV variability sample-stage circuit (toolchain bring-up).
# This is NOT the full fatigue-monitor ASIC -- just the interval-difference
# building block ported from the Wokwi diagram.
#
# Timing note: driving ui_in[0] from the testbench and then synchronizing
# it on-chip (2-flop synchronizer + edge detect) takes a fixed, measured
# number of clock edges before cur_interval/prev_interval react to it, in
# both directions (raising it and dropping it back down). The free-running
# counter keeps ticking during the "raise" window, so it gains
# BEAT_EXTRA_TICKS extra counts before the capture actually lands. This is
# a constant, fixed delay -- it does not bias the *difference* between
# successive intervals, since it is added equally to every interval. The
# exact edge counts below (BEAT_PROCESS_EDGES, RELEASE_DECAY_EDGES) were
# measured directly from waveform traces of this design, not guessed.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

CLOCK_PERIOD_NS = 10  # 100 MHz for a fast sim; the design is purely
                       # cycle-counted so the real clock rate doesn't matter

BEAT_PROCESS_EDGES = 4   # edges from raising ui_in[0] until prev_interval
                          # updates and cur_interval clears to 0
BEAT_EXTRA_TICKS = 3     # cur_interval's free-running increments during
                          # that window
RELEASE_DECAY_EDGES = 3  # edges from dropping ui_in[0] until cur_interval
                          # resumes counting from a clean 0


# ------------------------------------------------------------------ helpers

def uo(dut):
    return dut.uo_out.value.to_unsigned()


def ge(dut, n):
    """thermometer bit for threshold n (1..7)"""
    return (uo(dut) >> (n - 1)) & 1


def sign(dut):
    return (uo(dut) >> 7) & 1


def prev_interval(dut):
    return dut.uio_out.value.to_unsigned() & 0x7


async def start(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, unit="ns").start())
    dut.ena.value = 1
    dut.uio_in.value = 0
    dut.ui_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)


async def press_beat(dut):
    """Raise ui_in[0] and wait exactly long enough for the beat to be
    processed: prev_interval updated, cur_interval cleared to 0. Leaves
    ui_in held high."""
    dut.ui_in.value = 1
    await ClockCycles(dut.clk, BEAT_PROCESS_EDGES)


async def release_beat(dut, extra_ticks=0):
    """Drop ui_in[0] back to 0 and wait for the release to fully
    propagate. After this call, cur_interval == extra_ticks exactly."""
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, RELEASE_DECAY_EDGES + extra_ticks)


# -------------------------------------------------------------------- tests

@cocotb.test()
async def test_reset_state(dut):
    await start(dut)
    assert uo(dut) == 0, "everything low at reset: no elapsed time yet, prev=0"
    assert prev_interval(dut) == 0, "previous interval starts at 0"


@cocotb.test()
async def test_counter_free_runs_against_zero_baseline(dut):
    # Before any beat, prev_interval == 0, so diff == cur_interval directly.
    await start(dut)

    await ClockCycles(dut.clk, 3)  # cur_interval == 3
    assert ge(dut, 3) == 1 and ge(dut, 4) == 0, "after 3 ticks with prev=0, diff==3"
    assert sign(dut) == 0, "cur (3) >= prev (0): not running short"


@cocotb.test()
async def test_beat_captures_previous_interval(dut):
    await start(dut)

    await ClockCycles(dut.clk, 3)  # cur_interval == 3 at the moment of the beat
    await press_beat(dut)          # prev <= 3 + BEAT_EXTRA_TICKS, cur -> 0

    assert prev_interval(dut) == 3 + BEAT_EXTRA_TICKS, \
        "the interval that just finished is latched as previous"
    # cur_interval is freshly reset to 0 (still held), prev_interval == 6:
    assert sign(dut) == 1, "cur_interval == 0 < prev_interval == 6: running short"
    assert ge(dut, 6) == 1 and ge(dut, 7) == 0, "|0-6| == 6"

    await release_beat(dut)  # let go of the button before the next test phase


@cocotb.test()
async def test_direction_bit_sped_up(dut):
    # First interval capture: 6 ticks. Then a shorter second interval: only
    # 2 ticks have elapsed when we check -- still short of the previous
    # interval (6), so sign == 1.
    await start(dut)

    await ClockCycles(dut.clk, 3)
    await press_beat(dut)
    assert prev_interval(dut) == 6, "first interval captured as 6"

    await release_beat(dut, extra_ticks=2)  # cur_interval == 2
    assert sign(dut) == 1, "elapsed (2) < previous (6): running short / sped up"
    assert ge(dut, 4) == 1 and ge(dut, 5) == 0, "|2-6| == 4"


@cocotb.test()
async def test_diff_crosses_zero_at_matching_elapsed(dut):
    # Establish prev_interval = 6, then let the new interval free-run past
    # it. The direction bit should read 1 while short, 0 at the exact
    # matching point (diff == 0), and 0 once it has passed.
    await start(dut)

    await ClockCycles(dut.clk, 3)
    await press_beat(dut)
    assert prev_interval(dut) == 6

    await release_beat(dut, extra_ticks=5)  # cur_interval == 5
    assert sign(dut) == 1 and ge(dut, 1) == 1, "one tick short of matching: |5-6|==1, sign=1"

    await ClockCycles(dut.clk, 1)  # cur_interval == 6 (pure free-run tick)
    assert uo(dut) == 0, "elapsed matches the previous interval exactly: diff == 0"

    await ClockCycles(dut.clk, 1)  # cur_interval == 7
    assert sign(dut) == 0 and ge(dut, 1) == 1, "one tick past matching: |7-6|==1, sign=0"


@cocotb.test()
async def test_three_bit_wraparound_is_expected(dut):
    # Known limitation carried over from the Wokwi version: the counter is
    # only 3 bits, so 8 ticks wraps back to 0 instead of saturating at 7.
    await start(dut)

    await ClockCycles(dut.clk, 8)
    assert uo(dut) == 0, "8 ticks wraps the 3-bit counter back to 0 (known limitation)"
