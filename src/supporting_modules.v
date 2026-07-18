/*
Step 5: 

interval_tracker
interval_validator
variability_calculator
calibration_controller
alert_controller
*/

module interval_tracker (
    input sample_valid,
    input peak_valid, 
    input wire clk,
    input wire rst_n,
    output reg [9:0] interval_out,
    output reg interval_valid
);
    reg have_previous_peak;
    reg [9:0] interval_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin 
            have_previous_peak <= 0;
            interval_counter <= 0;
            interval_out <= 0;
            interval_valid <= 0;
        end
        else begin
            interval_valid <= 0; 
            if (sample_valid) begin 
                interval_counter <= interval_counter + 1;
                if (peak_valid) begin 
                    if (have_previous_peak) begin 
                        interval_out <= interval_counter + 1;
                        interval_valid <= 1;
                    end else begin 
                        have_previous_peak <= 1;
                    end
                    interval_counter <= 0;
                end
                else begin 
                    if (interval_counter >= 10'd1022) begin 
                        interval_counter <= 10'd1022;
                    end
                end
            end
        end
    end
endmodule



module interval_validator (
    input [9:0] interval_in, 
    input interval_valid,
    input wire clk,
    input wire rst_n,
    output reg [9:0] accepted_interval, 
    output reg accepted_interval_valid
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accepted_interval <= 0;
            accepted_interval_valid <= 0;
        end
        else begin
            accepted_interval_valid <= 0;
            if (interval_valid) begin 
                if (interval_in >= 30 && interval_in <= 200) begin
                    accepted_interval <= interval_in;
                    accepted_interval_valid <= 1;
                end
            end
        end
    end

    /* Potential additional feature, minimal logic
    Checks if there is a sudden change in the interval, if there is, then it is invalid 
    just this logic: abs(interval_in - previous_interval) <= MAX_INTERVAL_CHANGE as part of the &&
    obviously, need to hold previous_interval in a register, and update it when accepted_interval_valid is high
    */
endmodule

module variability_calculator (
    input [9:0] accepted_interval,
    input accepted_interval_valid,
    input wire clk,
    input wire rst_n,
    output reg [9:0] current_variability, 
    output reg variability_valid
);
    reg [9:0] previous_interval;
    reg have_previous_interval;
    reg [12:0] difference_sum;
    reg [3:0] difference_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            previous_interval <= 0;
            have_previous_interval <= 0;
            difference_sum <= 0;
            difference_count <= 0;
            current_variability <= 0;
            variability_valid <= 0;
        end
        else begin
            variability_valid <= 0;
            if (accepted_interval_valid) begin
                if (!have_previous_interval) begin
                    previous_interval <= accepted_interval;
                    have_previous_interval <= 1;
                end
                else begin
                    reg [9:0] difference;
                    if (accepted_interval > previous_interval) begin
                        difference = accepted_interval - previous_interval;
                    end else begin
                        difference = previous_interval - accepted_interval;
                    end
                    difference_sum <= difference_sum + difference;
                    difference_count <= difference_count + 1;

                    previous_interval <= accepted_interval;
                    if (difference_count == 7) begin
                        current_variability <= (difference_sum + difference) >> 3; // divide by 8
                        variability_valid <= 1;
                        difference_sum <= 0;
                        difference_count <= 0;
                        end
                    end
                end
            end
        end
endmodule

module calibration_controller (
    input [9:0] current_variability,
    input variability_valid,
    input wire clk,
    input wire rst_n,
    output reg [9:0] baseline_variability,
    output reg baseline_valid // once this is set high, then it stays high until reset
);
    reg [13:0] baseline_sum;
    reg [4:0] baseline_count;   // widened from [3:0] to [4:0] so it can actually reach 16

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baseline_sum <= 0;
            baseline_count <= 0;
            baseline_variability <= 0;
            baseline_valid <= 0;
        end
        else begin
            if (!baseline_valid) begin
                if (variability_valid) begin
                    baseline_sum <= baseline_sum + current_variability;
                    baseline_count <= baseline_count + 1;
                    if (baseline_count == 15) begin
                        baseline_variability <= (baseline_sum + current_variability) >> 4;
                        baseline_valid <= 1;
                    end
                end
            end
        end
    end
endmodule

module alert_controller (
    input [9:0] current_variability,
    input variability_valid,
    input [9:0] baseline_variability,
    input baseline_valid, 
    input wire clk,
    input wire rst_n,
    output reg alert_out
);
    parameter PERSISTENCE_LIMIT = 4; // number of consecutive low variability readings before alert

    reg [2:0] low_count;

    wire [9:0] threshold = baseline_variability - (baseline_variability >> 2); // 75% of baseline

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            low_count <= 0;
            alert_out <= 0;
        end
        else begin
            if (baseline_valid && variability_valid) begin
                if (current_variability >= threshold) begin
                    low_count <= 0;
                    alert_out <= 0;
                end
                else begin
                    if (low_count < PERSISTENCE_LIMIT) begin
                        low_count <= low_count + 1;
                        if (low_count + 1 >= PERSISTENCE_LIMIT) begin
                            alert_out <= 1;
                        end
                        /* 
                        There are 3 options for managing clearing the alert_out 
                        1. automatic, very responsive to current_variability, as soon as it gets to normal then turn it off
                        2. automatic, low responsive to current_variability, when it activates, keep it on for 8 sec (800 cycle) after it gets to normal
                        3. manual, it is on until reset is pressed
                        */
                    end
                end
            end
        end
    end
endmodule

module dc_filter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_valid,
    input  wire [17:0] raw_sample,
    output reg  [18:0] ac_sample,
    output reg         filter_valid
);

    reg [23:0] dc_estimate;
    reg        first_sample_seen;


    reg signed [24:0] raw_extended_s;
    reg signed [24:0] dc_estimate_s;
    reg signed [24:0] error_val;
    reg signed [24:0] delta;
    reg signed [25:0] dc_estimate_next_wide;

    reg signed [18:0] raw_sample_s;
    reg signed [18:0] dc_integer_s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dc_estimate       <= 24'd0;
            ac_sample         <= 19'd0;
            filter_valid      <= 1'b0;
            first_sample_seen <= 1'b0;
        end else begin
            if (sample_valid) begin
                if (!first_sample_seen) begin
                    dc_estimate       <= {raw_sample, 6'b000000};
                    first_sample_seen <= 1'b1;
                    filter_valid      <= 1'b0;
                end else begin
                    raw_extended_s = {raw_sample, 6'b000000};
                    dc_estimate_s  = dc_estimate;
                    error_val      = raw_extended_s - dc_estimate_s;
                    delta          = error_val >>> 7;

                    dc_estimate_next_wide = dc_estimate_s + delta;
                    dc_estimate  <= dc_estimate_next_wide[23:0];

                    raw_sample_s  = raw_sample;
                    dc_integer_s  = dc_estimate[23:6];
                    ac_sample    <= raw_sample_s - dc_integer_s;
                    filter_valid <= 1'b1;
                end
            end else begin
                filter_valid <= 1'b0;
            end
        end
    end

endmodule

module peak_detector (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               filter_valid,
    input  wire signed [18:0] ac_sample,
    output reg                peak_valid
);
    reg signed [18:0] envelope;
    reg signed [18:0] threshold;
    reg signed [18:0] sample;
    reg signed [18:0] candidate_peak;
    reg               rising;
    reg [5:0]         refractory_count;
    reg [7:0]         warmup_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            envelope         <= 19'sd0;
            threshold        <= 19'sd0;
            candidate_peak   <= 19'sd0;
            rising           <= 1'b0;
            refractory_count <= 6'd0;
            warmup_count     <= 8'd0;
            peak_valid       <= 1'b0;
        end else begin
            peak_valid <= 1'b0; // default: overridden below only when a peak is declared

            if (filter_valid) begin
                sample = -ac_sample; // polarity: beats are dips in ac_sample

                if (sample > envelope) begin
                    envelope <= sample;
                end else begin
                    envelope <= envelope - (envelope >>> 6);
                end

                threshold <= envelope - (envelope >>> 2);

                if (warmup_count < 200) begin
                    warmup_count <= warmup_count + 1;
                end else if (refractory_count > 0) begin
                    refractory_count <= refractory_count - 1;
                end else if (!rising && sample > threshold) begin
                    rising         <= 1'b1;
                    candidate_peak <= sample;
                end else if (rising && sample > candidate_peak) begin
                    candidate_peak <= sample;
                end else if (rising && sample <= candidate_peak) begin
                    rising           <= 1'b0;
                    refractory_count <= 6'd35;
                    peak_valid       <= 1'b1;
                end
            end
        end
    end
endmodule

module tick_generator (
    input clk, 
    input rst_n,
    output reg i2c_tick
    // could have ena input 
);
    parameter integer F_clk = 10_000_000; // 10 MHz, change once you have the real clk rate from info.yaml
    localparam integer DIVIDER_VALUE = F_clk / 400_000; // 400 kHz tick (4 subphases x 100 kHz I2C)
    localparam integer COUNTER_WIDTH = $clog2(DIVIDER_VALUE);

    reg [COUNTER_WIDTH-1:0] counter;

    always @(posedge clk or negedge rst_n) begin
        i2c_tick <= 0;
        if (!rst_n) begin
            counter <= 0;
        end else begin
            if (counter == DIVIDER_VALUE - 1) begin
                counter <= 0;
                i2c_tick <= 1;
            end else begin
                counter <= counter + 1;
            end
        end
    end
endmodule



module i2c_byte_engine (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       i2c_tick,

    input  wire       cmd_valid,
    input  wire [1:0] cmd_type,
    input  wire [7:0] tx_byte,
    input  wire       read_ack_value,

    input  wire       sda_in,

    output wire       cmd_ready,
    output reg        cmd_done,
    output reg [7:0]  rx_byte,
    output reg        ack_received,
    output reg        i2c_error,

    output reg        sda_drive_low,
    output reg        scl_drive_low
);

    // ---- command type encoding ----
    localparam CMD_START      = 2'd0;
    localparam CMD_STOP       = 2'd1;
    localparam CMD_WRITE_BYTE = 2'd2;
    localparam CMD_READ_BYTE  = 2'd3;

    // ---- FSM states ----
    localparam S_IDLE               = 4'd0;
    localparam S_START_RELEASE_SDA  = 4'd1;
    localparam S_START_RELEASE_SCL  = 4'd2;
    localparam S_START_EDGE         = 4'd3;
    localparam S_START_SCL_LOW      = 4'd4;
    localparam S_BIT_LOW            = 4'd5;
    localparam S_BIT_SETUP          = 4'd6;
    localparam S_BIT_HIGH           = 4'd7;
    localparam S_BIT_SAMPLE         = 4'd8;
    localparam S_STOP_FORCE_SDA_LOW = 4'd9;
    localparam S_STOP_RAISE_SCL     = 4'd10;
    localparam S_STOP_EDGE          = 4'd11;
    localparam S_STOP_RELEASE_SCL   = 4'd12;

    reg [3:0] state;

    reg [7:0] tx_byte_latched;
    reg       read_ack_latched;
    reg       is_write;
    reg [3:0] bit_counter;   // 0-7 = data bits, 8 = ack/nack phase
    reg [7:0] rx_shift;

    wire currently_transmitting = is_write ? (bit_counter < 8) : (bit_counter == 8);
    wire tx_bit_value           = (bit_counter < 8) ? tx_byte_latched[7 - bit_counter] : read_ack_latched;

    assign cmd_ready = (state == S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            cmd_done         <= 1'b0;
            rx_byte          <= 8'd0;
            ack_received     <= 1'b0;
            i2c_error        <= 1'b0;
            sda_drive_low    <= 1'b0;
            scl_drive_low    <= 1'b0;
            tx_byte_latched  <= 8'd0;
            read_ack_latched <= 1'b0;
            is_write         <= 1'b0;
            bit_counter      <= 4'd0;
            rx_shift         <= 8'd0;
        end
        else begin
            cmd_done  <= 1'b0;   // default: pulse only when explicitly completing a command

            if (i2c_tick) begin
                case (state)

                    S_IDLE: begin
                        if (cmd_valid) begin
                            case (cmd_type)
                                CMD_START: begin
                                    state <= S_START_RELEASE_SDA;
                                end
                                CMD_STOP: begin
                                    state <= S_STOP_FORCE_SDA_LOW;
                                end
                                CMD_WRITE_BYTE: begin
                                    tx_byte_latched <= tx_byte;
                                    is_write        <= 1'b1;
                                    bit_counter     <= 4'd0;
                                    state           <= S_BIT_LOW;
                                end
                                CMD_READ_BYTE: begin
                                    read_ack_latched <= read_ack_value;
                                    is_write         <= 1'b0;
                                    bit_counter      <= 4'd0;
                                    state            <= S_BIT_LOW;
                                end
                                default: begin
                                    state <= S_IDLE;
                                end
                            endcase
                        end
                    end

                    // ---- START (also serves as REPEATED START) ----
                    S_START_RELEASE_SDA: begin
                        sda_drive_low <= 1'b0;   // release SDA while SCL stays low
                        state <= S_START_RELEASE_SCL;
                    end

                    S_START_RELEASE_SCL: begin
                        scl_drive_low <= 1'b0;   // release SCL too: bus should now float idle-high
                        state <= S_START_EDGE;
                    end

                    S_START_EDGE: begin
                        if (!sda_in) begin
                            // SDA never actually floated high -- something else is holding the bus
                            i2c_error <= 1'b1;
                        end
                        sda_drive_low <= 1'b1;   // pull SDA low while SCL is high: the START edge
                        state <= S_START_SCL_LOW;
                    end

                    S_START_SCL_LOW: begin
                        scl_drive_low <= 1'b1;   // bring SCL low, ready to clock out the first bit
                        state    <= S_IDLE;
                        cmd_done <= 1'b1;
                    end

                    // ---- one bit cycle: shared by data bits AND the ack/nack bit ----
                    S_BIT_LOW: begin
                        scl_drive_low <= 1'b1;
                        if (currently_transmitting)
                            sda_drive_low <= tx_bit_value ? 1'b0 : 1'b1;
                        else
                            sda_drive_low <= 1'b0;  // release; the other side drives this bit
                        state <= S_BIT_SETUP;
                    end

                    S_BIT_SETUP: begin
                        state <= S_BIT_HIGH;
                    end

                    S_BIT_HIGH: begin
                        scl_drive_low <= 1'b0;
                        state <= S_BIT_SAMPLE;
                    end

                    S_BIT_SAMPLE: begin
                        if (!currently_transmitting) begin
                            if (bit_counter < 8)
                                rx_shift <= {rx_shift[6:0], sda_in};
                            else
                                ack_received <= ~sda_in;  // SDA low = ACK = success
                        end

                        if (bit_counter == 8) begin
                            if (!is_write)
                                rx_byte <= rx_shift;
                            scl_drive_low <= 1'b1;   // always park SCL low between commands
                            state    <= S_IDLE;
                            cmd_done <= 1'b1;
                        end else begin
                            bit_counter <= bit_counter + 1'b1;
                            state       <= S_BIT_LOW;
                        end
                    end

                    // ---- STOP ----
                    S_STOP_FORCE_SDA_LOW: begin
                        sda_drive_low <= 1'b1;
                        state <= S_STOP_RAISE_SCL;
                    end

                    S_STOP_RAISE_SCL: begin
                        scl_drive_low <= 1'b0;
                        state <= S_STOP_EDGE;
                    end

                    S_STOP_EDGE: begin
                        sda_drive_low <= 1'b0;   // release SDA while SCL is high: the STOP edge
                        state <= S_STOP_RELEASE_SCL;
                    end

                    S_STOP_RELEASE_SCL: begin
                        scl_drive_low <= 1'b0;
                        state    <= S_IDLE;
                        cmd_done <= 1'b1;
                    end

                    default: begin
                        state <= S_IDLE;
                    end

                endcase
            end
        end
    end

endmodule

module max30102_controller (
    input  wire        clk,
    input  wire        rst_n,

    // command interface to i2c_byte_engine
    output reg         cmd_valid,
    output reg  [1:0]  cmd_type,
    output reg  [7:0]  tx_byte,
    output reg         read_ack_value,
    input  wire        cmd_ready,
    input  wire        cmd_done,
    input  wire [7:0]  rx_byte,
    input  wire        ack_received,
    input  wire        i2c_error,

    // outputs to the rest of the chip
    output reg  [17:0] raw_sample,
    output reg         sample_valid,
    output reg         sensor_fault,
    output reg         init_done
);

    // ---- i2c_byte_engine command encoding (must match i2c_byte_engine.v) ----
    localparam CMD_START      = 2'd0;
    localparam CMD_STOP       = 2'd1;
    localparam CMD_WRITE_BYTE = 2'd2;
    localparam CMD_READ_BYTE  = 2'd3;

    // ---- MAX30102 constants, confirmed against the datasheet ----
    localparam [7:0] ADDR_WRITE = 8'hAE;
    localparam [7:0] ADDR_READ  = 8'hAF;

    localparam [7:0] REG_FIFO_WR_PTR   = 8'h04;
    localparam [7:0] REG_OVF_COUNTER   = 8'h05;
    localparam [7:0] REG_FIFO_RD_PTR   = 8'h06;
    localparam [7:0] REG_FIFO_DATA     = 8'h07;
    localparam [7:0] REG_FIFO_CONFIG   = 8'h08;
    localparam [7:0] REG_MODE_CONFIG   = 8'h09;
    localparam [7:0] REG_SPO2_CONFIG   = 8'h0A;
    localparam [7:0] REG_LED2_PA       = 8'h0D;  // IR current: slot1=LED2=IR, so LED2_PA, not LED1_PA
    localparam [7:0] REG_SLOT_CONFIG   = 8'h11;
    localparam [7:0] REG_PART_ID       = 8'hFF;

    localparam [7:0] PART_ID_EXPECTED  = 8'h15;
    localparam [7:0] MODE_RESET_CMD    = 8'h40;  // RESET bit set
    localparam [7:0] MODE_MULTI_LED    = 8'h07;  // SHDN=0, RESET=0, MODE=111

    // provisional, pending real ESP32 characterization data:
    localparam [7:0] FIFO_CONFIG_VALUE  = 8'h10;  // no averaging, rollover enabled
    localparam [7:0] SPO2_CONFIG_VALUE  = 8'h27;  // ADC_RGE=01(placeholder), SR=100sps, PW=411us/18-bit
    localparam [7:0] LED2_CURRENT_VALUE = 8'h24;  // placeholder IR current
    localparam [7:0] SLOT_CONFIG_VALUE  = 8'h02;  // slot1 = IR (LED2)

    localparam POWER_WAIT_CYCLES = 16;            // placeholder startup delay

    // ---- top-level FSM states ----
    localparam S_POWER_WAIT           = 5'd0;
    localparam S_CHECK_PART_ID        = 5'd1;
    localparam S_POLL_RESET_SETUP     = 5'd2;
    localparam S_POLL_RESET_CHECK     = 5'd3;
    localparam S_INIT_WRITE_SETUP     = 5'd4;
    localparam S_INIT_WRITE_NEXT      = 5'd5;
    localparam S_RUNTIME_POLL_SETUP   = 5'd6;
    localparam S_RUNTIME_POLL_CHECK   = 5'd7;
    localparam S_RUNTIME_SAMPLE_READY = 5'd8;
    localparam S_FAULT_HALT           = 5'd9;

    // shared WRITE_REG sub-sequence: START, addr(W), reg, value, STOP
    localparam S_WRITE_REG_START    = 5'd10;
    localparam S_WRITE_REG_ADDR     = 5'd11;
    localparam S_WRITE_REG_REGADDR  = 5'd12;
    localparam S_WRITE_REG_VALUE    = 5'd13;
    localparam S_WRITE_REG_STOP     = 5'd14;

    // shared READ_REG sub-sequence: START, addr(W), reg, Sr, addr(R), byte(s), STOP
    localparam S_READ_REG_START     = 5'd15;
    localparam S_READ_REG_ADDRW     = 5'd16;
    localparam S_READ_REG_REGADDR   = 5'd17;
    localparam S_READ_REG_RSTART    = 5'd18;
    localparam S_READ_REG_ADDRR     = 5'd19;
    localparam S_READ_REG_BYTE      = 5'd20;
    localparam S_READ_REG_STOP      = 5'd21;

    reg [4:0] state;
    reg [4:0] return_state;

    // parameters for the shared sub-sequences
    reg [7:0] seq_reg_addr;
    reg [7:0] seq_write_value;
    reg [1:0] seq_read_count;   // 1 or 3
    reg [1:0] seq_byte_index;
    reg [7:0] seq_rx0, seq_rx1, seq_rx2;

    reg [7:0]  power_wait_counter;
    reg [2:0]  init_write_index;
    reg [4:0]  rd_ptr;
    reg [4:0]  samples_remaining;

    // ============================================================
    // Command outputs are PURELY COMBINATIONAL functions of `state`.
    // This is deliberate: if these were assigned inside the sequential
    // block on a per-state basis, the last cycle of a finishing state
    // would still be re-driving that state's OLD command value (Verilog
    // evaluates the current state's own logic before `state` itself
    // updates), creating a one-cycle window where `state` has already
    // moved on but the command outputs haven't caught up. A fast-enough
    // i2c_tick could land exactly in that window and latch stale data.
    // Deriving these combinationally from `state` makes that impossible:
    // they change in perfect lockstep with `state`, same cycle, always.
    // ============================================================
    always @(*) begin
        cmd_valid      = 1'b0;
        cmd_type       = CMD_START;
        tx_byte        = 8'd0;
        read_ack_value = 1'b0;
        case (state)
            S_WRITE_REG_START:   begin cmd_valid = 1'b1; cmd_type = CMD_START; end
            S_WRITE_REG_ADDR:    begin cmd_valid = 1'b1; cmd_type = CMD_WRITE_BYTE; tx_byte = ADDR_WRITE; end
            S_WRITE_REG_REGADDR: begin cmd_valid = 1'b1; cmd_type = CMD_WRITE_BYTE; tx_byte = seq_reg_addr; end
            S_WRITE_REG_VALUE:   begin cmd_valid = 1'b1; cmd_type = CMD_WRITE_BYTE; tx_byte = seq_write_value; end
            S_WRITE_REG_STOP:    begin cmd_valid = 1'b1; cmd_type = CMD_STOP; end

            S_READ_REG_START:    begin cmd_valid = 1'b1; cmd_type = CMD_START; end
            S_READ_REG_ADDRW:    begin cmd_valid = 1'b1; cmd_type = CMD_WRITE_BYTE; tx_byte = ADDR_WRITE; end
            S_READ_REG_REGADDR:  begin cmd_valid = 1'b1; cmd_type = CMD_WRITE_BYTE; tx_byte = seq_reg_addr; end
            S_READ_REG_RSTART:   begin cmd_valid = 1'b1; cmd_type = CMD_START; end
            S_READ_REG_ADDRR:    begin cmd_valid = 1'b1; cmd_type = CMD_WRITE_BYTE; tx_byte = ADDR_READ; end
            S_READ_REG_BYTE:     begin
                cmd_valid      = 1'b1;
                cmd_type       = CMD_READ_BYTE;
                read_ack_value = (seq_byte_index == seq_read_count - 1'b1) ? 1'b1 : 1'b0;
            end
            S_READ_REG_STOP:     begin cmd_valid = 1'b1; cmd_type = CMD_STOP; end

            default: ; // cmd_valid stays 0: idle, or a bookkeeping-only state
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_POWER_WAIT;
            return_state       <= S_POWER_WAIT;
            raw_sample         <= 18'd0;
            sample_valid       <= 1'b0;
            sensor_fault       <= 1'b0;
            init_done          <= 1'b0;
            seq_reg_addr       <= 8'd0;
            seq_write_value    <= 8'd0;
            seq_read_count     <= 2'd0;
            seq_byte_index     <= 2'd0;
            seq_rx0            <= 8'd0;
            seq_rx1            <= 8'd0;
            seq_rx2            <= 8'd0;
            power_wait_counter <= 8'd0;
            init_write_index   <= 3'd0;
            rd_ptr             <= 5'd0;
            samples_remaining  <= 5'd0;
        end
        else begin
            sample_valid <= 1'b0;   // default: pulse only on a completed sample

            case (state)

                // ============== POWER-UP ==============
                S_POWER_WAIT: begin
                    if (power_wait_counter == POWER_WAIT_CYCLES - 1) begin
                        seq_reg_addr   <= REG_PART_ID;
                        seq_read_count <= 2'd1;
                        return_state   <= S_CHECK_PART_ID;
                        state          <= S_READ_REG_START;
                    end else begin
                        power_wait_counter <= power_wait_counter + 1'b1;
                    end
                end

                S_CHECK_PART_ID: begin
                    if (seq_rx0 == PART_ID_EXPECTED) begin
                        seq_reg_addr    <= REG_MODE_CONFIG;
                        seq_write_value <= MODE_RESET_CMD;
                        return_state    <= S_POLL_RESET_SETUP;
                        state           <= S_WRITE_REG_START;
                    end else begin
                        sensor_fault <= 1'b1;
                        state        <= S_FAULT_HALT;
                    end
                end

                // ============== RESET POLLING ==============
                S_POLL_RESET_SETUP: begin
                    seq_reg_addr   <= REG_MODE_CONFIG;
                    seq_read_count <= 2'd1;
                    return_state   <= S_POLL_RESET_CHECK;
                    state          <= S_READ_REG_START;
                end

                S_POLL_RESET_CHECK: begin
                    if (seq_rx0[6]) begin
                        state <= S_POLL_RESET_SETUP;   // still resetting, poll again
                    end else begin
                        init_write_index <= 3'd0;
                        state            <= S_INIT_WRITE_SETUP;
                    end
                end

                // ============== REMAINING INIT WRITES (table-driven) ==============
                S_INIT_WRITE_SETUP: begin
                    return_state <= S_INIT_WRITE_NEXT;
                    state        <= S_WRITE_REG_START;
                    case (init_write_index)
                        3'd0: begin seq_reg_addr <= REG_FIFO_WR_PTR; seq_write_value <= 8'h00; end
                        3'd1: begin seq_reg_addr <= REG_OVF_COUNTER; seq_write_value <= 8'h00; end
                        3'd2: begin seq_reg_addr <= REG_FIFO_RD_PTR; seq_write_value <= 8'h00; end
                        3'd3: begin seq_reg_addr <= REG_FIFO_CONFIG; seq_write_value <= FIFO_CONFIG_VALUE; end
                        3'd4: begin seq_reg_addr <= REG_SPO2_CONFIG; seq_write_value <= SPO2_CONFIG_VALUE; end
                        3'd5: begin seq_reg_addr <= REG_LED2_PA;     seq_write_value <= LED2_CURRENT_VALUE; end
                        3'd6: begin seq_reg_addr <= REG_SLOT_CONFIG; seq_write_value <= SLOT_CONFIG_VALUE; end
                        3'd7: begin seq_reg_addr <= REG_MODE_CONFIG; seq_write_value <= MODE_MULTI_LED; end
                        default: begin seq_reg_addr <= REG_MODE_CONFIG; seq_write_value <= MODE_MULTI_LED; end
                    endcase
                end

                S_INIT_WRITE_NEXT: begin
                    if (init_write_index == 3'd7) begin
                        init_done <= 1'b1;
                        rd_ptr    <= 5'd0;
                        state     <= S_RUNTIME_POLL_SETUP;
                    end else begin
                        init_write_index <= init_write_index + 1'b1;
                        state            <= S_INIT_WRITE_SETUP;
                    end
                end

                // ============== RUNTIME: poll FIFO_WR_PTR ==============
                S_RUNTIME_POLL_SETUP: begin
                    seq_reg_addr   <= REG_FIFO_WR_PTR;
                    seq_read_count <= 2'd1;
                    return_state   <= S_RUNTIME_POLL_CHECK;
                    state          <= S_READ_REG_START;
                end

                S_RUNTIME_POLL_CHECK: begin
                    if (((seq_rx0[4:0] - rd_ptr) & 5'h1F) == 5'd0) begin
                        state <= S_RUNTIME_POLL_SETUP;   // nothing new yet, keep polling
                    end else begin
                        samples_remaining <= (seq_rx0[4:0] - rd_ptr) & 5'h1F;
                        seq_reg_addr      <= REG_FIFO_DATA;
                        seq_read_count    <= 2'd3;
                        return_state      <= S_RUNTIME_SAMPLE_READY;
                        state             <= S_READ_REG_START;
                    end
                end

                S_RUNTIME_SAMPLE_READY: begin
                    raw_sample   <= {seq_rx0[1:0], seq_rx1, seq_rx2};
                    sample_valid <= 1'b1;
                    rd_ptr       <= rd_ptr + 1'b1;
                    if (samples_remaining == 5'd1) begin
                        state <= S_RUNTIME_POLL_SETUP;    // that was the last backlogged sample
                    end else begin
                        samples_remaining <= samples_remaining - 1'b1;
                        seq_reg_addr      <= REG_FIFO_DATA;
                        seq_read_count    <= 2'd3;
                        return_state      <= S_RUNTIME_SAMPLE_READY;
                        state             <= S_READ_REG_START;  // more backlog, read another directly
                    end
                end

                // ============== FAULT ==============
                S_FAULT_HALT: begin
                    // sit here forever; only an external reset recovers
                end

                // ============== SHARED: WRITE_REG(seq_reg_addr, seq_write_value) ==============
                S_WRITE_REG_START: begin
                    if (cmd_done) state <= S_WRITE_REG_ADDR;
                end
                S_WRITE_REG_ADDR: begin
                    if (cmd_done) begin
                        if (!ack_received) begin sensor_fault <= 1'b1; state <= S_FAULT_HALT; end
                        else state <= S_WRITE_REG_REGADDR;
                    end
                end
                S_WRITE_REG_REGADDR: begin
                    if (cmd_done) begin
                        if (!ack_received) begin sensor_fault <= 1'b1; state <= S_FAULT_HALT; end
                        else state <= S_WRITE_REG_VALUE;
                    end
                end
                S_WRITE_REG_VALUE: begin
                    if (cmd_done) begin
                        if (!ack_received) begin sensor_fault <= 1'b1; state <= S_FAULT_HALT; end
                        else state <= S_WRITE_REG_STOP;
                    end
                end
                S_WRITE_REG_STOP: begin
                    if (cmd_done) begin
                        if (!ack_received) begin sensor_fault <= 1'b1; state <= S_FAULT_HALT; end
                        else state <= return_state;
                    end
                end

                // ============== SHARED: READ_REG(seq_reg_addr, seq_read_count) -> seq_rx0/1/2 ==============
                S_READ_REG_START: begin
                    if (cmd_done) state <= S_READ_REG_ADDRW;
                end
                S_READ_REG_ADDRW: begin
                    if (cmd_done) begin
                        if (!ack_received) begin sensor_fault <= 1'b1; state <= S_FAULT_HALT; end
                        else state <= S_READ_REG_REGADDR;
                    end
                end
                S_READ_REG_REGADDR: begin
                    if (cmd_done) begin
                        if (!ack_received) begin sensor_fault <= 1'b1; state <= S_FAULT_HALT; end
                        else state <= S_READ_REG_RSTART;
                    end
                end
                S_READ_REG_RSTART: begin
                    if (cmd_done) state <= S_READ_REG_ADDRR;
                end
                S_READ_REG_ADDRR: begin
                    if (cmd_done) begin
                        if (!ack_received) begin sensor_fault <= 1'b1; state <= S_FAULT_HALT; end
                        else begin
                            seq_byte_index <= 2'd0;
                            state          <= S_READ_REG_BYTE;
                        end
                    end
                end
                S_READ_REG_BYTE: begin
                    if (cmd_done) begin
                        case (seq_byte_index)
                            2'd0: seq_rx0 <= rx_byte;
                            2'd1: seq_rx1 <= rx_byte;
                            2'd2: seq_rx2 <= rx_byte;
                            default: ;
                        endcase
                        if (seq_byte_index == seq_read_count - 1'b1) begin
                            state <= S_READ_REG_STOP;
                        end else begin
                            seq_byte_index <= seq_byte_index + 1'b1;
                            // stays in S_READ_REG_BYTE; the combinational
                            // block above will issue the next READ_BYTE
                            // using the updated seq_byte_index next cycle
                        end
                    end
                end
                S_READ_REG_STOP: begin
                    if (cmd_done) state <= return_state;
                end

                default: state <= S_POWER_WAIT;

            endcase
        end
    end

endmodule