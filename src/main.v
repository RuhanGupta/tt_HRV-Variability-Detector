module main (
    input wire clk,
    input wire rst_n,
    input wire sample_valid,
    input wire [17:0] raw_sample,
    output alert_out,
    output [9:0] main_baseline_variability,
    output main_baseline_valid,
    output [9:0] main_current_variability,
    output main_variability_valid
);
    wire signed [18:0] ac_sample;
    wire               filter_valid;
    wire               peak_valid;

    dc_filter dc (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .raw_sample(raw_sample),
        .ac_sample(ac_sample),
        .filter_valid(filter_valid)
    );

    peak_detector pd (
        .clk(clk),
        .rst_n(rst_n),
        .filter_valid(filter_valid),
        .ac_sample(ac_sample),
        .peak_valid(peak_valid)
    );

    wire [9:0] interval_out;
    wire interval_valid;

    wire [9:0] accepted_interval;
    wire accepted_interval_valid;

    wire [9:0] current_variability;
    wire variability_valid;

    wire [9:0] baseline_variability;
    wire baseline_valid;

    interval_tracker tracker (
        .sample_valid(sample_valid),
        .peak_valid(peak_valid),
        .clk(clk),
        .rst_n(rst_n),
        .interval_out(interval_out),
        .interval_valid(interval_valid)
    );

    interval_validator validator (
        .interval_in(interval_out),
        .interval_valid(interval_valid),
        .clk(clk),
        .rst_n(rst_n),
        .accepted_interval(accepted_interval),
        .accepted_interval_valid(accepted_interval_valid)
    );

    variability_calculator variability (
        .accepted_interval(accepted_interval),
        .accepted_interval_valid(accepted_interval_valid),
        .clk(clk),
        .rst_n(rst_n),
        .current_variability(current_variability),
        .variability_valid(variability_valid)
    );

    calibration_controller calibration (
        .current_variability(current_variability),
        .variability_valid(variability_valid),
        .clk(clk),
        .rst_n(rst_n),
        .baseline_variability(baseline_variability),
        .baseline_valid(baseline_valid)
    );

    alert_controller alert (
        .current_variability(current_variability),
        .variability_valid(variability_valid),
        .baseline_variability(baseline_variability),
        .baseline_valid(baseline_valid),
        .clk(clk),
        .rst_n(rst_n),
        .alert_out(alert_out)
    );

    assign main_current_variability = current_variability;
    assign main_variability_valid   = variability_valid;
    assign main_baseline_variability = baseline_variability;
    assign main_baseline_valid       = baseline_valid;

endmodule