// ============================================================================
// deg2phase.sv
// ----------------------------------------------------------------------------
// Input : deg_q9_7 (unsigned Q9.7 degrees, 0..360+)
// Output: phase_i  (DW-bit CORDIC code: deg * 2^(DW-1)/180)
// Latency: IDLY + 1 (DSP) + ODLY
//
// Q-formats:
//   deg_q9_7 (16b, Q9.7) × K_Q8_8 (25b) -> 43b product (Q17.15)
//   >>> (8 + 7) = 15  -> integer CORDIC code (take [DW-1:0])
//
// For DW=16: K_Q8_8 = round((2^15/180)*2^8) = 46603 = 25'sh00B60B
// 360° maps to code = 65536 -> wraps to 0 in 16 bits (correct full-turn behavior).
// ============================================================================

module deg2phase #(
    parameter int DW   = 16,  // cordic_dds.DW (typ. 16)
    parameter int IDLY = 1,   // input pipeline inside mul_dsp
    parameter int ODLY = 1    // output pipeline inside mul_dsp
)(
    input  logic               clk,
    input  logic               rst_n,
    input  logic               valid_in,    // 1-cycle pulse or level

    input  logic        [15:0] deg_in,    // unsigned Q9.7 degrees

    output logic        [DW-1:0] phase_i,   // to cordic_dds
    output logic                 valid_out
);
    // ------------------------------------------------------------------------
    // Constant K = 2^(DW-1) / 180 encoded as Q8.8 (25-bit) for DSP A-port
    // For DW != 16, recompute this constant offline and replace the literal.
    // DW=16 -> 25'sh00B60B (46603)
    // ------------------------------------------------------------------------
    localparam logic signed [24:0] DEG2CODE_K_Q8_8 = 25'sh00B60B;
    localparam int SHIFT_OUT = 8 + 7; // const frac(8) + deg frac(7) -> 15

    // Zero-extend degrees to 18b for DSP B-port (unsigned range 0..~512)
    wire logic [17:0] deg_ue = {2'b00, deg_in};

    // DSP multiply
    logic               v_mul;
    logic signed [42:0] prod, code_wide;

    mul_dsp #(
        .WA   (25),
        .WB   (18),
        .IDLY (IDLY),
        .ODLY (ODLY)
    ) u_mul_deg2code (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (valid_in),
        .a         (DEG2CODE_K_Q8_8),     // Q8.8 constant
        .b         ($signed(deg_ue)),     // unsigned → treated positive
        .valid_out (v_mul),
        .p         (prod)                 // 43-bit product
    );

    assign valid_out = v_mul;
    assign code_wide = prod >>> SHIFT_OUT; // drop 15 fractional bits
    assign phase_i = code_wide[DW-1:0];

endmodule
