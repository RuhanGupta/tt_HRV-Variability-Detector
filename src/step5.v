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

