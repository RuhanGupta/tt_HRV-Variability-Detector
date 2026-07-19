module interval_tracker (
    input sample_valid,
    input peak_valid, 
    input wire clk,
    input wire rst_n,
    output reg [7:0] interval_out,
    output reg interval_valid
);
    reg have_previous_peak;
    reg [7:0] interval_counter;

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
                    if (interval_counter >= 8'd254) begin 
                        interval_counter <= 8'd254;
                    end
                end
            end
        end
    end
endmodule



module interval_validator (
    input [7:0] interval_in, 
    input interval_valid,
    input wire clk,
    input wire rst_n,
    output reg [7:0] accepted_interval, 
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
endmodule

module variability_calculator (
    input [7:0] accepted_interval,
    input accepted_interval_valid,
    input wire clk,
    input wire rst_n,
    output reg [7:0] current_variability, 
    output reg variability_valid
);
    reg [7:0] previous_interval;
    reg have_previous_interval;
    reg [10:0] difference_sum;
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
                else begin : diff_calc
                    reg [7:0] difference;
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
    input [7:0] current_variability,
    input variability_valid,
    input wire clk,
    input wire rst_n,
    output reg [7:0] baseline_variability,
    output reg baseline_valid
);
    reg [11:0] baseline_sum;
    reg [4:0] baseline_count; 

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
    input [7:0] current_variability,
    input variability_valid,
    input [7:0] baseline_variability,
    input baseline_valid, 
    input wire clk,
    input wire rst_n,
    output reg alert_out
);
    parameter PERSISTENCE_LIMIT = 4; 

    reg [2:0] low_count;

    wire [7:0] threshold = baseline_variability - (baseline_variability >> 2); // 75% of baseline

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


/*
Peak detector constants match the Session 3 model defaults:
  ENVELOPE_DECAY_SHIFT = 6
  THRESHOLD_SHIFT = 2 75% of envelope
  REFRACTORY_SAMPLES = 35 350 ms at 100 sps
  WARMUP_SAMPLES = 200
  POLARITY_INVERT = 1 dip = beat
*/

module dc_filter #(
    parameter integer FILTER_SHIFT = 7,
    parameter integer DC_FRAC_BITS = 6
)(
    input wire clk,
    input wire rst_n,
    input wire sample_valid,
    input wire [17:0] raw_sample,
    output reg signed [18:0] ac_sample,
    output reg filter_valid
);
    localparam integer DC_WIDTH = 18 + DC_FRAC_BITS;

    reg [DC_WIDTH-1:0] dc_estimate;
    reg dc_initialized;
    wire signed [DC_WIDTH:0] raw_scaled = {1'b0, raw_sample, {DC_FRAC_BITS{1'b0}}};
    wire signed [DC_WIDTH:0] dc_estimate_ext = {1'b0, dc_estimate};
    wire signed [DC_WIDTH:0] error = raw_scaled - dc_estimate_ext;
    wire signed [DC_WIDTH:0] dc_next = dc_estimate_ext + (error >>> FILTER_SHIFT);

    wire signed [18:0] raw_signed = $signed({1'b0, raw_sample});
    wire signed [18:0] dc_next_integer = $signed({1'b0, dc_next[DC_WIDTH-1:DC_FRAC_BITS]});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dc_estimate <= {DC_WIDTH{1'b0}};
            dc_initialized <= 1'b0;
            ac_sample <= 19'sd0;
            filter_valid <= 1'b0;
        end
        else begin
            filter_valid <= 1'b0;
            if (sample_valid) begin
                if (!dc_initialized) begin
                    dc_estimate <= {raw_sample, {DC_FRAC_BITS{1'b0}}};
                    dc_initialized <= 1'b1;
                    ac_sample <= 19'sd0;
                    filter_valid <= 1'b0; 
                end
                else begin
                    dc_estimate <= dc_next[DC_WIDTH-1:0];
                    ac_sample <= raw_signed - dc_next_integer;
                    filter_valid <= 1'b1;
                end
            end
        end
    end
endmodule


module peak_detector #(
    parameter integer ENVELOPE_DECAY_SHIFT = 6,
    parameter integer THRESHOLD_SHIFT = 2,
    parameter integer REFRACTORY_SAMPLES = 35,
    parameter integer WARMUP_SAMPLES = 200,
    parameter integer POLARITY_INVERT = 1
)(
    input wire clk,
    input wire rst_n,
    input wire filter_valid,
    input wire signed [18:0] ac_sample,
    output reg peak_valid
);
    reg [12:0] envelope;
    reg [12:0] candidate_val;
    reg rising_seen;
    reg [5:0] refractory_count; 
    reg [7:0] warmup_count; 

    wire signed [19:0] polarity_val = POLARITY_INVERT ? -{ac_sample[18], ac_sample}
                                                        : {ac_sample[18], ac_sample};
    // Full-width magnitude, then saturate into the 13-bit envelope datapath.
    // Real traces keep the envelope <= ~4677 (13-bit cap 8191); only the pre-settle
    // DC transient produces a large mag, which this clamp bounds safely.
    wire [18:0] mag_full = polarity_val[19] ? 19'd0 : polarity_val[18:0];
    wire [12:0] mag = (|mag_full[18:13]) ? 13'h1FFF : mag_full[12:0];

    wire [12:0] envelope_next = (mag > envelope) ? mag
                                                   : (envelope - (envelope >> ENVELOPE_DECAY_SHIFT));
    wire [12:0] threshold_next = envelope_next - (envelope_next >> THRESHOLD_SHIFT);

    wire in_warmup = (warmup_count < WARMUP_SAMPLES[7:0]);
    wire in_refractory = (refractory_count != 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            envelope <= 13'd0;
            candidate_val <= 13'd0;
            rising_seen <= 1'b0;
            refractory_count <= 6'd0;
            warmup_count <= 8'd0;
            peak_valid <= 1'b0;
        end
        else begin
            peak_valid <= 1'b0;

            if (filter_valid) begin
                envelope <= envelope_next;

                if (in_warmup)
                    warmup_count <= warmup_count + 8'd1;

                if (in_refractory) begin
                    refractory_count <= refractory_count - 6'd1;
                end
                else if (!in_warmup) begin
                    if (!rising_seen) begin
                        if (mag > threshold_next) begin
                            rising_seen <= 1'b1;
                            candidate_val <= mag;
                        end
                    end
                    else begin
                        if (mag >= candidate_val) begin
                            candidate_val <= mag;
                        end
                        else begin
                            peak_valid <= 1'b1;
                            rising_seen <= 1'b0;
                            refractory_count <= REFRACTORY_SAMPLES[5:0];
                        end
                    end
                end
            end
        end
    end
endmodule


module tick_generator #(
    parameter integer CLK_FREQ_HZ = 10_000_000, // need to confirm 
    parameter integer TICK_FREQ_HZ = 400_000
)(
    input wire clk,
    input wire rst_n,
    input wire ena,
    output reg i2c_tick
);
    localparam integer DIVIDER = CLK_FREQ_HZ / TICK_FREQ_HZ;
    localparam integer CW = (DIVIDER <= 1) ? 1 : $clog2(DIVIDER);

    reg [CW-1:0] count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= {CW{1'b0}};
            i2c_tick <= 1'b0;
        end
        else if (!ena) begin
            count <= {CW{1'b0}};
            i2c_tick <= 1'b0;
        end
        else if (count == DIVIDER[CW-1:0] - 1'b1) begin
            count <= {CW{1'b0}};
            i2c_tick <= 1'b1;
        end
        else begin
            count <= count + 1'b1;
            i2c_tick <= 1'b0;
        end
    end
endmodule

module i2c_byte_engine #(
    parameter integer SCL_STRETCH_TIMEOUT_TICKS = 64
)(
    input wire clk,
    input wire rst_n,
    input wire i2c_tick,

    input wire cmd_valid,
    input wire [1:0] cmd_type,
    input wire [7:0] tx_byte,
    input wire read_ack_value,
    output wire cmd_ready,
    output reg cmd_done,
    output reg [7:0] rx_byte,
    output reg ack_received,
    output reg i2c_error,

    input wire sda_in,
    input wire scl_in,
    output reg sda_drive_low,
    output reg scl_drive_low
);

    localparam [1:0] CMD_START = 2'b00;
    localparam [1:0] CMD_STOP = 2'b01;
    localparam [1:0] CMD_WRITE_BYTE = 2'b10;
    localparam [1:0] CMD_READ_BYTE = 2'b11;

    localparam [3:0]
        S_IDLE = 4'd0,
        S_START_HIGH = 4'd1, // both lines released, verify idle-high
        S_START_SDA = 4'd2, // SDA pulled low while SCL still high
        S_START_SCL = 4'd3, // SCL pulled low, ready for first bit
        S_BIT_SETUP = 4'd4, // SCL low, drive/release SDA for this bit
        S_BIT_RISE = 4'd5, // release SCL
        S_BIT_HIGH = 4'd6, // SCL high, sample on reads
        S_BIT_FALL = 4'd7, // drive SCL low again
        S_ACK_SETUP = 4'd8,
        S_ACK_RISE = 4'd9,
        S_ACK_HIGH = 4'd10,
        S_ACK_FALL = 4'd11,
        S_STOP_SETUP = 4'd12, // SCL low, SDA low
        S_STOP_RISE = 4'd13, // release SCL
        S_STOP_SDA = 4'd14, // release SDA while SCL high (the actual STOP edge)
        S_ERROR = 4'd15;

    reg [3:0] state;
    reg [2:0] bit_index; 
    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    reg [1:0] active_cmd;
    reg active_read_ack;
    reg [6:0] stretch_count;
    reg [2:0] bus_free_count; 

    localparam [2:0] BUS_FREE_TICKS = 3'd4;

    wire bus_idle_bad = (state == S_START_HIGH) && (~sda_in | ~scl_in);


    assign cmd_ready = (state == S_IDLE) && (bus_free_count >= BUS_FREE_TICKS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            bit_index <= 3'd0;
            tx_shift <= 8'd0;
            rx_shift <= 8'd0;
            active_cmd <= CMD_START;
            active_read_ack <= 1'b0;
            stretch_count <= 7'd0;
            bus_free_count <= 3'd0;
            cmd_done <= 1'b0;
            rx_byte <= 8'd0;
            ack_received <= 1'b0;
            i2c_error <= 1'b0;
            sda_drive_low <= 1'b0;
            scl_drive_low <= 1'b0;
        end
        else begin
            cmd_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    i2c_error <= 1'b0;
                    if (i2c_tick && bus_free_count < BUS_FREE_TICKS)
                        bus_free_count <= bus_free_count + 3'd1;
                    if (cmd_valid && bus_free_count >= BUS_FREE_TICKS) begin
                        active_cmd <= cmd_type;
                        tx_shift <= tx_byte;
                        active_read_ack <= read_ack_value;
                        bit_index <= 3'd7;
                        bus_free_count <= 3'd0;
                        case (cmd_type)
                            CMD_START: begin
                                sda_drive_low <= 1'b0; 
                                scl_drive_low <= 1'b0;
                                stretch_count <= 7'd0;
                                state <= S_START_HIGH;
                            end
                            CMD_STOP: begin
                                sda_drive_low <= 1'b1; 
                                scl_drive_low <= 1'b1; 
                                state <= S_STOP_SETUP;
                            end
                            CMD_WRITE_BYTE: begin
                                scl_drive_low <= 1'b1; 
                                sda_drive_low <= ~tx_byte[7];
                                state <= S_BIT_SETUP;
                            end
                            CMD_READ_BYTE: begin
                                scl_drive_low <= 1'b1;
                                sda_drive_low <= 1'b0; 
                                state <= S_BIT_SETUP;
                            end
                            default: state <= S_IDLE;
                        endcase
                    end
                end

                S_START_HIGH: begin
                    if (i2c_tick) begin
                        if (bus_idle_bad) begin
                            i2c_error <= 1'b1;
                            sda_drive_low <= 1'b0;
                            scl_drive_low <= 1'b0;
                            state <= S_IDLE; cmd_done <= 1'b1;
                        end
                        else begin
                            sda_drive_low <= 1'b1; 
                            state <= S_START_SDA;
                        end
                    end
                end
                S_START_SDA: begin
                    if (i2c_tick) begin
                        scl_drive_low <= 1'b1; 
                        state <= S_START_SCL;
                    end
                end
                S_START_SCL: begin
                    if (i2c_tick) begin
                        state <= S_IDLE; cmd_done <= 1'b1;
                    end
                end
                S_BIT_SETUP: begin
                    if (i2c_tick) begin
                        stretch_count <= 7'd0;
                        scl_drive_low <= 1'b0;
                        state <= S_BIT_RISE;
                    end
                end
                S_BIT_RISE: begin
                    if (i2c_tick) begin
                        if (scl_in) begin
                            state <= S_BIT_HIGH;
                        end
                        else if (stretch_count == SCL_STRETCH_TIMEOUT_TICKS[6:0]) begin
                            i2c_error <= 1'b1;
                            sda_drive_low <= 1'b0;
                            scl_drive_low <= 1'b0;
                            state <= S_IDLE; cmd_done <= 1'b1;
                        end
                        else begin
                            stretch_count <= stretch_count + 7'd1;
                        end
                    end
                end
                S_BIT_HIGH: begin
                    if (i2c_tick) begin
                        if (active_cmd == CMD_READ_BYTE)
                            rx_shift <= {rx_shift[6:0], sda_in};
                        scl_drive_low <= 1'b1; 
                        state <= S_BIT_FALL;
                    end
                end
                S_BIT_FALL: begin
                    if (i2c_tick) begin
                        if (bit_index == 3'd0) begin
                            if (active_cmd == CMD_WRITE_BYTE)
                                sda_drive_low <= 1'b0;
                            else
                                sda_drive_low <= active_read_ack;
                            state <= S_ACK_SETUP;
                        end
                        else begin
                            bit_index <= bit_index - 3'd1;
                            if (active_cmd == CMD_WRITE_BYTE)
                                sda_drive_low <= ~tx_shift[bit_index - 3'd1];
                            else
                                sda_drive_low <= 1'b0; 
                            state <= S_BIT_SETUP;
                        end
                    end
                end
                S_ACK_SETUP: begin
                    if (i2c_tick) begin
                        stretch_count <= 7'd0;
                        scl_drive_low <= 1'b0;
                        state <= S_ACK_RISE;
                    end
                end
                S_ACK_RISE: begin
                    if (i2c_tick) begin
                        if (scl_in) begin
                            state <= S_ACK_HIGH;
                        end
                        else if (stretch_count == SCL_STRETCH_TIMEOUT_TICKS[6:0]) begin
                            i2c_error <= 1'b1;
                            sda_drive_low <= 1'b0;
                            scl_drive_low <= 1'b0;
                            state <= S_IDLE; cmd_done <= 1'b1;
                        end
                        else begin
                            stretch_count <= stretch_count + 7'd1;
                        end
                    end
                end
                S_ACK_HIGH: begin
                    if (i2c_tick) begin
                        if (active_cmd == CMD_WRITE_BYTE)
                            ack_received <= ~sda_in; 
                        scl_drive_low <= 1'b1;
                        state <= S_ACK_FALL;
                    end
                end
                S_ACK_FALL: begin
                    if (i2c_tick) begin
                        sda_drive_low <= 1'b0; 
                        rx_byte <= rx_shift;
                        if (active_cmd == CMD_WRITE_BYTE && !ack_received) begin
                            i2c_error <= 1'b1;
                        end
                        state <= S_IDLE; cmd_done <= 1'b1;
                    end
                end
                S_STOP_SETUP: begin
                    if (i2c_tick) begin
                        scl_drive_low <= 1'b0; 
                        state <= S_STOP_RISE;
                    end
                end
                S_STOP_RISE: begin
                    if (i2c_tick) begin
                        state <= S_STOP_SDA;
                    end
                end
                S_STOP_SDA: begin
                    if (i2c_tick) begin
                        sda_drive_low <= 1'b0; 
                        state <= S_IDLE; cmd_done <= 1'b1;
                    end
                end

                default: begin
                    sda_drive_low <= 1'b0;
                    scl_drive_low <= 1'b0;
                    state <= S_IDLE; cmd_done <= 1'b1;
                end
            endcase
        end
    end
endmodule

module max30102_controller #(
    parameter [7:0] LED1_PA_DEFAULT = 8'h24,
    parameter [7:0] SPO2_CONFIG_VAL = 8'h27, 
    parameter integer FAULT_RETRY_LIMIT = 8,
    parameter integer POLL_IDLE_TICKS = 200 
)(
    input wire clk,
    input wire rst_n,
    input wire i2c_tick,

    
    output reg [17:0] raw_sample,
    output reg sample_valid,
    output reg sensor_fault,
    output reg calibrating_init, 

    
    input wire sda_in,
    input wire scl_in,
    output wire sda_drive_low,
    output wire scl_drive_low
);

    
    
    
    localparam [7:0] SLAVE_ADDR_W = 8'hAE;
    localparam [7:0] SLAVE_ADDR_R = 8'hAF;

    localparam [7:0] REG_FIFO_WR_PTR = 8'h04;
    localparam [7:0] REG_OVF_COUNTER = 8'h05;
    localparam [7:0] REG_FIFO_RD_PTR = 8'h06;
    localparam [7:0] REG_FIFO_DATA = 8'h07;
    localparam [7:0] REG_FIFO_CONFIG = 8'h08;
    localparam [7:0] REG_MODE_CONFIG = 8'h09;
    localparam [7:0] REG_SPO2_CONFIG = 8'h0A;
    localparam [7:0] REG_LED1_PA = 8'h0C;
    localparam [7:0] REG_SLOT21 = 8'h11;
    localparam [7:0] REG_SLOT43 = 8'h12;
    localparam [7:0] REG_PART_ID = 8'hFF;

    localparam [7:0] MODE_RESET = 8'h40; 
    localparam [7:0] MODE_MULTI_LED = 8'h07; 
    localparam [7:0] FIFO_CONFIG_VAL = 8'h10; 
    localparam [7:0] SLOT_CONFIG_VAL = 8'h02; 
    localparam [7:0] SLOT43_OFF_VAL = 8'h00;
    localparam [7:0] PART_ID_EXPECT = 8'h15;

    
    
    
    localparam [1:0]
        TXN_WRITE_REG = 2'd0,
        TXN_READ_REG = 2'd1,
        TXN_READ_FIFO3 = 2'd2;

    reg [1:0] txn_op;
    reg [7:0] txn_reg_addr;
    reg [7:0] txn_write_data;
    reg txn_start;
    wire txn_busy;
    reg txn_done;
    reg txn_error;
    reg [7:0] txn_rx0, txn_rx1, txn_rx2;

    localparam [3:0]
        T_IDLE = 4'd0,
        T_START1 = 4'd1,
        T_ADDR_W = 4'd2,
        T_REGADDR = 4'd3,
        T_DATA = 4'd4,
        T_RESTART = 4'd5,
        T_ADDR_R = 4'd6,
        T_READ1 = 4'd7,
        T_READ2 = 4'd8,
        T_READ3 = 4'd9,
        T_STOP = 4'd10,
        T_WAIT_CMD = 4'd11,
        T_DONE = 4'd12;

    reg [3:0] t_state;
    reg [3:0] t_return_state; 

    
    reg eng_cmd_valid;
    reg [1:0] eng_cmd_type;
    reg [7:0] eng_tx_byte;
    reg eng_read_ack_value;
    wire eng_cmd_ready;
    wire eng_cmd_done;
    wire [7:0] eng_rx_byte;
    wire eng_ack_received;
    wire eng_i2c_error;

    localparam [1:0] CMD_START = 2'b00;
    localparam [1:0] CMD_STOP = 2'b01;
    localparam [1:0] CMD_WRITE_BYTE = 2'b10;
    localparam [1:0] CMD_READ_BYTE = 2'b11;

    i2c_byte_engine u_i2c_engine (
        .clk (clk),
        .rst_n (rst_n),
        .i2c_tick (i2c_tick),
        .cmd_valid (eng_cmd_valid),
        .cmd_type (eng_cmd_type),
        .tx_byte (eng_tx_byte),
        .read_ack_value (eng_read_ack_value),
        .cmd_ready (eng_cmd_ready),
        .cmd_done (eng_cmd_done),
        .rx_byte (eng_rx_byte),
        .ack_received (eng_ack_received),
        .i2c_error (eng_i2c_error),
        .sda_in (sda_in),
        .scl_in (scl_in),
        .sda_drive_low (sda_drive_low),
        .scl_drive_low (scl_drive_low)
    );

    assign txn_busy = (t_state != T_IDLE);

    
    
    always @(*) begin
        eng_cmd_valid = 1'b0;
        eng_cmd_type = CMD_START;
        eng_tx_byte = 8'd0;
        eng_read_ack_value = 1'b0;
        case (t_state)
            T_START1, T_RESTART: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_START;
            end
            T_ADDR_W: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_WRITE_BYTE;
                eng_tx_byte = SLAVE_ADDR_W;
            end
            T_ADDR_R: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_WRITE_BYTE;
                eng_tx_byte = SLAVE_ADDR_R;
            end
            T_REGADDR: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_WRITE_BYTE;
                eng_tx_byte = txn_reg_addr;
            end
            T_DATA: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_WRITE_BYTE;
                eng_tx_byte = txn_write_data;
            end
            T_READ1: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_READ_BYTE;
                eng_read_ack_value = (txn_op == TXN_READ_FIFO3) ? 1'b0 : 1'b1; 
            end
            T_READ2: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_READ_BYTE;
                eng_read_ack_value = 1'b0; 
            end
            T_READ3: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_READ_BYTE;
                eng_read_ack_value = 1'b1; 
            end
            T_STOP: begin
                eng_cmd_valid = 1'b1;
                eng_cmd_type = CMD_STOP;
            end
            default: ; 
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_state <= T_IDLE;
            t_return_state <= T_IDLE;
            txn_done <= 1'b0;
            txn_error <= 1'b0;
            txn_rx0 <= 8'd0;
            txn_rx1 <= 8'd0;
            txn_rx2 <= 8'd0;
        end
        else begin
            txn_done <= 1'b0;

            case (t_state)
                T_IDLE: begin
                    if (txn_start) begin
                        txn_error <= 1'b0;
                        t_state <= T_START1;
                    end
                end

                
                T_START1: if (eng_cmd_ready) begin t_return_state <= T_ADDR_W; t_state <= T_WAIT_CMD; end
                T_ADDR_W: if (eng_cmd_ready) begin
                    t_return_state <= T_REGADDR;
                    t_state <= T_WAIT_CMD;
                end
                T_REGADDR: if (eng_cmd_ready) begin
                    case (txn_op)
                        TXN_WRITE_REG: t_return_state <= T_DATA;
                        default: t_return_state <= T_RESTART; 
                    endcase
                    t_state <= T_WAIT_CMD;
                end
                T_DATA: if (eng_cmd_ready) begin t_return_state <= T_STOP; t_state <= T_WAIT_CMD; end
                T_RESTART: if (eng_cmd_ready) begin t_return_state <= T_ADDR_R; t_state <= T_WAIT_CMD; end
                T_ADDR_R: if (eng_cmd_ready) begin t_return_state <= T_READ1; t_state <= T_WAIT_CMD; end
                T_READ1: if (eng_cmd_ready) begin
                    t_return_state <= (txn_op == TXN_READ_FIFO3) ? T_READ2 : T_STOP;
                    t_state <= T_WAIT_CMD;
                end
                T_READ2: if (eng_cmd_ready) begin t_return_state <= T_READ3; t_state <= T_WAIT_CMD; end
                T_READ3: if (eng_cmd_ready) begin t_return_state <= T_STOP; t_state <= T_WAIT_CMD; end
                T_STOP: if (eng_cmd_ready) begin t_return_state <= T_DONE; t_state <= T_WAIT_CMD; end

                T_WAIT_CMD: begin
                    if (eng_cmd_done) begin
                        if (eng_i2c_error)
                            txn_error <= 1'b1;
                        
                        
                        
                        
                        
                        if (t_return_state == T_READ2)
                            txn_rx0 <= eng_rx_byte;
                        else if (t_return_state == T_READ3)
                            txn_rx1 <= eng_rx_byte;
                        else if (t_return_state == T_STOP && txn_op != TXN_WRITE_REG)
                            txn_rx2 <= eng_rx_byte; 
                        t_state <= t_return_state;
                    end
                end

                T_DONE: begin
                    txn_done <= 1'b1;
                    t_state <= T_IDLE;
                end

                default: t_state <= T_IDLE;
            endcase
        end
    end

    
    
    wire [7:0] txn_read_reg_result = txn_rx2;

    
    
    
    localparam [4:0]
        S_POWER_WAIT = 5'd0,
        S_READ_PARTID = 5'd1,
        S_CHECK_PARTID = 5'd2,
        S_ISSUE_RESET = 5'd3,
        S_POLL_RESET = 5'd4,
        S_CHECK_RESET = 5'd5,
        S_CLEAR_WR_PTR = 5'd6,
        S_CLEAR_OVF = 5'd7,
        S_CLEAR_RD_PTR = 5'd8,
        S_WRITE_FIFO_CFG = 5'd9,
        S_WRITE_SPO2_CFG = 5'd10,
        S_WRITE_LED_CUR = 5'd11,
        S_WRITE_SLOT21 = 5'd12,
        S_WRITE_SLOT43 = 5'd13,
        S_START_RUN = 5'd14,
        S_RUN_READ_WRPTR = 5'd15,
        S_RUN_READ_RDPTR = 5'd16,
        S_RUN_CHECK_AVAIL = 5'd17,
        S_RUN_READ_SAMPLE = 5'd18,
        S_RUN_SAMPLE_DONE = 5'd19,
        S_RUN_POLL_WAIT = 5'd20,
        S_FAULT_WAIT = 5'd21,
        S_WAIT_RESET_WRITE = 5'd22;

    reg [4:0] seq_state;
    reg [10:0] power_wait_count;
    reg [7:0] poll_wait_count;
    reg [4:0] fault_count;
    reg [4:0] fifo_wr_ptr;
    reg [4:0] fifo_available;

    localparam [10:0] POWER_WAIT_CYCLES = 11'd2000; 

    
    

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seq_state <= S_POWER_WAIT;
            power_wait_count <= 11'd0;
            poll_wait_count <= 8'd0;
            fault_count <= 5'd0;
            fifo_wr_ptr <= 5'd0;
            fifo_available <= 5'd0;
            raw_sample <= 18'd0;
            sample_valid <= 1'b0;
            sensor_fault <= 1'b0;
            calibrating_init <= 1'b1;
            txn_op <= TXN_WRITE_REG;
            txn_reg_addr <= 8'd0;
            txn_write_data <= 8'd0;
            txn_start <= 1'b0;
        end
        else begin
            sample_valid <= 1'b0;
            txn_start <= 1'b0;

            
            
            if (txn_done) begin
                if (txn_error) begin
                    if (fault_count == FAULT_RETRY_LIMIT[4:0])
                        fault_count <= fault_count; 
                    else
                        fault_count <= fault_count + 5'd1;
                end
                else begin
                    fault_count <= 5'd0;
                end
            end

            if (fault_count >= FAULT_RETRY_LIMIT[4:0] && seq_state != S_FAULT_WAIT) begin
                sensor_fault <= 1'b1;
                seq_state <= S_FAULT_WAIT;
            end
            else begin
                case (seq_state)
                    
                    S_POWER_WAIT: begin
                        calibrating_init <= 1'b1;
                        if (power_wait_count == POWER_WAIT_CYCLES)
                            seq_state <= S_READ_PARTID;
                        else
                            power_wait_count <= power_wait_count + 11'd1;
                    end

                    S_READ_PARTID: begin
                        if (!txn_busy) begin
                            txn_op <= TXN_READ_REG;
                            txn_reg_addr <= REG_PART_ID;
                            txn_start <= 1'b1;
                            seq_state <= S_CHECK_PARTID;
                        end
                    end
                    S_CHECK_PARTID: begin
                        if (txn_done) begin
                            if (!txn_error && txn_read_reg_result == PART_ID_EXPECT)
                                seq_state <= S_ISSUE_RESET;
                            else
                                seq_state <= S_READ_PARTID; 
                        end
                    end

                    S_ISSUE_RESET: begin
                        if (!txn_busy) begin
                            txn_op <= TXN_WRITE_REG;
                            txn_reg_addr <= REG_MODE_CONFIG;
                            txn_write_data <= MODE_RESET;
                            txn_start <= 1'b1;
                            seq_state <= S_WAIT_RESET_WRITE;
                        end
                    end
                    S_WAIT_RESET_WRITE: begin
                        if (txn_done) begin
                            if (!txn_error)
                                seq_state <= S_POLL_RESET;
                            else
                                txn_start <= 1'b1; 
                        end
                    end
                    
                    
                    
                    S_POLL_RESET: begin
                        if (!txn_busy) begin
                            txn_op <= TXN_READ_REG;
                            txn_reg_addr <= REG_MODE_CONFIG;
                            txn_start <= 1'b1;
                            seq_state <= S_CHECK_RESET;
                        end
                    end
                    S_CHECK_RESET: begin
                        if (txn_done) begin
                            if (!txn_error && !txn_read_reg_result[6])
                                seq_state <= S_CLEAR_WR_PTR; 
                            else
                                seq_state <= S_POLL_RESET; 
                        end
                    end

                    S_CLEAR_WR_PTR: begin
                        if (!txn_busy) begin
                            txn_op <= TXN_WRITE_REG;
                            txn_reg_addr <= REG_FIFO_WR_PTR;
                            txn_write_data <= 8'h00;
                            txn_start <= 1'b1;
                            seq_state <= S_CLEAR_OVF;
                        end
                    end
                    S_CLEAR_OVF: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                txn_op <= TXN_WRITE_REG;
                                txn_reg_addr <= REG_OVF_COUNTER;
                                txn_write_data <= 8'h00;
                                txn_start <= 1'b1;
                                seq_state <= S_CLEAR_RD_PTR;
                            end
                            else
                                txn_start <= 1'b1;
                        end
                    end
                    S_CLEAR_RD_PTR: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                txn_op <= TXN_WRITE_REG;
                                txn_reg_addr <= REG_FIFO_RD_PTR;
                                txn_write_data <= 8'h00;
                                txn_start <= 1'b1;
                                seq_state <= S_WRITE_FIFO_CFG;
                            end
                            else
                                txn_start <= 1'b1;
                        end
                    end
                    S_WRITE_FIFO_CFG: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                txn_op <= TXN_WRITE_REG;
                                txn_reg_addr <= REG_FIFO_CONFIG;
                                txn_write_data <= FIFO_CONFIG_VAL;
                                txn_start <= 1'b1;
                                seq_state <= S_WRITE_SPO2_CFG;
                            end
                            else
                                txn_start <= 1'b1;
                        end
                    end
                    S_WRITE_SPO2_CFG: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                txn_op <= TXN_WRITE_REG;
                                txn_reg_addr <= REG_SPO2_CONFIG;
                                txn_write_data <= SPO2_CONFIG_VAL;
                                txn_start <= 1'b1;
                                seq_state <= S_WRITE_LED_CUR;
                            end
                            else
                                txn_start <= 1'b1;
                        end
                    end
                    S_WRITE_LED_CUR: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                txn_op <= TXN_WRITE_REG;
                                txn_reg_addr <= REG_LED1_PA;
                                txn_write_data <= LED1_PA_DEFAULT;
                                txn_start <= 1'b1;
                                seq_state <= S_WRITE_SLOT21;
                            end
                            else
                                txn_start <= 1'b1;
                        end
                    end
                    S_WRITE_SLOT21: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                txn_op <= TXN_WRITE_REG;
                                txn_reg_addr <= REG_SLOT21;
                                txn_write_data <= SLOT_CONFIG_VAL;
                                txn_start <= 1'b1;
                                seq_state <= S_WRITE_SLOT43;
                            end
                            else
                                txn_start <= 1'b1;
                        end
                    end
                    S_WRITE_SLOT43: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                txn_op <= TXN_WRITE_REG;
                                txn_reg_addr <= REG_MODE_CONFIG;
                                txn_write_data <= MODE_MULTI_LED;
                                txn_start <= 1'b1;
                                seq_state <= S_START_RUN;
                            end
                            else
                                txn_start <= 1'b1;
                        end
                    end
                    S_START_RUN: begin
                        if (txn_done) begin
                            if (!txn_error)
                                seq_state <= S_RUN_READ_WRPTR;
                            else
                                txn_start <= 1'b1;
                        end
                    end

                    
                    S_RUN_READ_WRPTR: begin
                        calibrating_init <= 1'b0;
                        if (!txn_busy && !txn_start) begin
                            txn_op <= TXN_READ_REG;
                            txn_reg_addr <= REG_FIFO_WR_PTR;
                            txn_start <= 1'b1;
                            seq_state <= S_RUN_READ_RDPTR;
                        end
                    end
                    S_RUN_READ_RDPTR: begin
                        // txn_done here belongs to the WR_PTR read started in
                        // S_RUN_READ_WRPTR, so capture fifo_wr_ptr in THIS state.
                        if (txn_done) begin
                            if (!txn_error) begin
                                fifo_wr_ptr <= txn_read_reg_result[4:0];
                                txn_op <= TXN_READ_REG;
                                txn_reg_addr <= REG_FIFO_RD_PTR;
                                txn_start <= 1'b1;
                                seq_state <= S_RUN_CHECK_AVAIL;
                            end
                            else
                                seq_state <= S_RUN_READ_WRPTR; 
                        end
                    end
                    S_RUN_CHECK_AVAIL: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                fifo_available <= (fifo_wr_ptr - txn_read_reg_result[4:0]) & 5'h1F;
                                seq_state <= S_RUN_READ_SAMPLE; 
                            end
                            else
                                seq_state <= S_RUN_READ_WRPTR; 
                        end
                    end
                    S_RUN_READ_SAMPLE: begin
                        if (fifo_available == 5'd0) begin
                            
                            poll_wait_count <= 8'd0;
                            seq_state <= S_RUN_POLL_WAIT;
                        end
                        else if (!txn_busy) begin
                            txn_op <= TXN_READ_FIFO3;
                            txn_reg_addr <= REG_FIFO_DATA;
                            txn_start <= 1'b1;
                            seq_state <= S_RUN_SAMPLE_DONE;
                        end
                    end
                    S_RUN_SAMPLE_DONE: begin
                        if (txn_done) begin
                            if (!txn_error) begin
                                
                                
                                raw_sample <= {txn_rx0[1:0], txn_rx1, txn_rx2};
                                sample_valid <= 1'b1;
                            end
                            seq_state <= S_RUN_READ_WRPTR;
                        end
                    end
                    S_RUN_POLL_WAIT: begin
                        if (poll_wait_count == POLL_IDLE_TICKS[7:0])
                            seq_state <= S_RUN_READ_WRPTR;
                        else if (i2c_tick)
                            poll_wait_count <= poll_wait_count + 8'd1;
                    end

                    
                    S_FAULT_WAIT: begin
                        sensor_fault <= 1'b1;
                        calibrating_init <= 1'b1;
                        if (poll_wait_count == POLL_IDLE_TICKS[7:0]) begin
                            poll_wait_count <= 8'd0;
                            fault_count <= 5'd0;
                            power_wait_count <= 11'd0;
                            sensor_fault <= 1'b0;
                            seq_state <= S_POWER_WAIT; 
                        end
                        else if (i2c_tick)
                            poll_wait_count <= poll_wait_count + 8'd1;
                    end

                    default: seq_state <= S_POWER_WAIT;
                endcase
            end
        end
    end

endmodule



module tt_um_fatigue_monitor (
    input wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input wire ena,
    input wire clk,
    input wire rst_n
);

    
    
    
    wire i2c_tick;

    tick_generator #(
        .CLK_FREQ_HZ (10_000_000),
        .TICK_FREQ_HZ (400_000)
    ) u_tick_gen (
        .clk (clk),
        .rst_n (rst_n),
        .ena (ena),
        .i2c_tick (i2c_tick)
    );

    wire [17:0] raw_sample;
    wire sample_valid;
    wire sensor_fault;
    wire calibrating_init;
    wire sda_drive_low;
    wire scl_drive_low;

    max30102_controller u_sensor (
        .clk (clk),
        .rst_n (rst_n),
        .i2c_tick (i2c_tick),
        .raw_sample (raw_sample),
        .sample_valid (sample_valid),
        .sensor_fault (sensor_fault),
        .calibrating_init (calibrating_init),
        .sda_in (uio_in[0]),
        .scl_in (uio_in[1]),
        .sda_drive_low (sda_drive_low),
        .scl_drive_low (scl_drive_low)
    );

    
    
    
    wire alert_out;
    wire baseline_valid;
    wire variability_valid;
    wire beat_debug;
    wire [7:0] baseline_variability_unused;
    wire [7:0] current_variability_unused;

    main u_main (
        .clk (clk),
        .rst_n (rst_n),
        .sample_valid (sample_valid),
        .raw_sample (raw_sample),
        .alert_out (alert_out),
        .main_baseline_variability (baseline_variability_unused),
        .main_baseline_valid (baseline_valid),
        .main_current_variability (current_variability_unused),
        .main_variability_valid (variability_valid),
        .beat_debug (beat_debug)
    );

    
    
    
    assign uo_out[0] = alert_out;
    assign uo_out[1] = sensor_fault;
    assign uo_out[2] = baseline_valid;
    assign uo_out[3] = beat_debug;
    assign uo_out[4] = calibrating_init;
    assign uo_out[5] = sample_valid;
    assign uo_out[6] = variability_valid;
    assign uo_out[7] = 1'b0;

    
    assign uio_out[0] = 1'b0;
    assign uio_out[1] = 1'b0;
    assign uio_out[7:2] = 6'b0;
    assign uio_oe[0] = sda_drive_low;
    assign uio_oe[1] = scl_drive_low;
    assign uio_oe[7:2] = 6'b0;

    
    
    wire _unused_ok = &{1'b0, ui_in, 1'b0};

endmodule