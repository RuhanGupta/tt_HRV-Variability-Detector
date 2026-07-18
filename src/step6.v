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