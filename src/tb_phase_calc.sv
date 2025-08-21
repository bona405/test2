`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Simple TB for phase_calc (Q9.7 inputs for x,y, az, el)
//   - Set az/el/x/y/is_tx and see the DUT result.
//   - Prints Q1.31 "turns", degrees, and the 6-bit index.
// ---------------------------------------------------------------------------
module tb_phase_calc;

    // Match your DUT params
    localparam int CORDIC_DW   = 16;
    localparam int CORDIC_LAT  = 14;
    localparam int M_IDLY      = 1;
    localparam int M_ODLY      = 1;
    localparam int D2R_IDLY    = 1;
    localparam int D2R_ODLY    = 1;

    // Clock / reset
    logic clk, rst_n;
    initial       clk = 0;
    always  #5    clk = ~clk;       // 100 MHz

    initial begin
        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;
    end

    // DUT I/O (Q9.7 for inputs)
    logic              start;
    logic              is_tx;
    logic       [15:0] x_offset_q9_7;  // Q9.7 [mm]  (0..~256mm typical)
    logic       [15:0] y_offset_q9_7;  // Q9.7 [mm]
    logic       [15:0] az_deg_q9_7;    // Q9.7 [deg] (0..360)
    logic       [15:0] el_deg_q9_7;    // Q9.7 [deg] (0..90)

    logic       [31:0] phase_turn_q31; // Q1.31 [0,1)
    logic        [5:0] phase_idx;
    logic              phase_valid;
    logic              phase_busy;

    // DUT
    phase_calc #(
        .CORDIC_DW (CORDIC_DW),
//        .CORDIC_LAT(CORDIC_LAT),
        .M_IDLY    (M_IDLY),
        .M_ODLY    (M_ODLY),
        .D2R_IDLY  (D2R_IDLY),
        .D2R_ODLY  (D2R_ODLY)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .is_tx      (is_tx),
        .x_offset   (x_offset_q9_7), // Q9.7 mm
        .y_offset   (y_offset_q9_7), // Q9.7 mm
        .az_deg     (az_deg_q9_7),   // Q9.7 deg (unsigned)
        .el_deg     (el_deg_q9_7),   // Q9.7 deg (unsigned)
        .phase_turn (phase_turn_q31),
        .phase_idx  (phase_idx),
        .busy       (phase_busy),
        .valid      (phase_valid)
    );

    // Helpers
    localparam real Q9_7  = 128.0;
    localparam real Q1_31 = 2147483648.0; // 2^31

    function automatic logic [15:0] mm_to_q9_7 (real mm);
        mm_to_q9_7 = $rtoi(mm * Q9_7);
    endfunction

    function automatic logic [15:0] deg_to_q9_7_u (real deg);
        // clamp to 0..360 for safety
        real d = (deg < 0.0) ? 0.0 : ((deg > 360.0) ? 360.0 : deg);
        deg_to_q9_7_u = $rtoi(d * Q9_7);
    endfunction

    function automatic real q1_31_to_turns (logic [31:0] q);
        return real'(q) / Q1_31;
    endfunction

    task automatic run_one (
        real az_d, real el_d, real x_mm, real y_mm, bit tx
    );
        // declare reals first (tool-friendly)
        real turns;
        real degout;

        // drive inputs
        is_tx         <= tx;
        x_offset_q9_7 <= mm_to_q9_7(x_mm);
        y_offset_q9_7 <= mm_to_q9_7(y_mm);
        az_deg_q9_7   <= deg_to_q9_7_u(az_d);
        el_deg_q9_7   <= deg_to_q9_7_u(el_d);

        // start pulse
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // wait for result
        @(posedge clk);
        wait (phase_valid === 1'b1);

        turns  = q1_31_to_turns(phase_turn_q31);
        degout = turns * 360.0;

        $display("\n--- RESULT ---------------------------------------------");
        $display("  is_tx=%0d  az=%7.3f deg  el=%7.3f deg  x=%7.3f mm  y=%7.3f mm",
                 tx, az_d, el_d, x_mm, y_mm);
        $display("  phase_turn (Q1.31) = 0x%08h  ->  turns=%.8f  (deg=%.6f)",
                 phase_turn_q31, turns, degout);
        $display("  phase_idx (0..63)  = %0d", phase_idx);
        $display("--------------------------------------------------------\n");

        @(posedge clk); // consume valid
    endtask

    // Test sequence
    initial begin
        start         = 1'b0;
        is_tx         = 1'b0;
        x_offset_q9_7 = '0;
        y_offset_q9_7 = '0;
        az_deg_q9_7   = '0;
        el_deg_q9_7   = '0;

        wait (rst_n);
        @(posedge clk);

        // TX cases
        run_one( 30.0, 60.0, 10.0, 15.0, 1);
        run_one( 80.0, 70.0, 30.0, 40.0, 1);
        run_one( 110.0, 20.0, 30.0, 40.0, 1);
        run_one( 330.0, 25.0, 30.0, 40.0, 1);

        // RX cases
        run_one(  0.0,  0.0,  7.5,  0.0, 0);
        run_one(120.0, 30.0, 15.0, 22.5, 0);

        // Edge checks (unsigned az path)
        run_one(359.0,  0.0,  5.0,  0.0, 1); // 359° ≡ -1° expectation
        run_one(  1.0,  0.0,  5.0,  0.0, 1); // near-zero az

        $display("TB done.");
        #50;
        $finish;
    end

    // Optional VCD
    // initial begin
    //     if ($test$plusargs("dump")) begin
    //         $dumpfile("tb_phase_calc.vcd");
    //         $dumpvars(0, tb_phase_calc);
    //     end
    // end

endmodule
