// ============================================================================
// phase_calc.sv  (deg → CORDIC code; optimized arithmetic; 3 DSPs in core)
// ----------------------------------------------------------------------------
// Model
//   turns = K_turn * cos(theta) * ( x*cos(phi) - y*sin(phi) )
//   phi   = az  (unsigned degrees, Q9.7)
//   theta = el  (unsigned degrees, Q9.7)
//
// Angle path
//   Use deg2phase to map degrees (Q9.7) → CORDIC phase code (DW bits).
//   This replaces deg→rad→code with a single DSP multiply per angle.
//
// I/O Q-formats
//   x_offset, y_offset : Q9.7   [mm]  (16b unsigned; cast to signed for DSP)
//   az_deg,   el_deg   : Q9.7   [deg] (16b unsigned → deg2phase → CORDIC code)
//   cordic sin/cos     : Q1.15  (signed 16b from your cordic_dds DW=16)
//   K_turn (AIR)       : Q0.17  (turns/mm; signed 18b two's complement)
//   t_pre, t_mm        : Q10.14 (24/25b internal after scaling)
//   phase_turn         : Q1.31  (signed; we also provide wrapped-to-[0,1) view)
//   phase_idx          : 6-bit  floor(frac(turns)*64) computed directly from Q1.31
//
// DSP budget per element
//   deg2phase φ: 1 DSP
//   deg2phase θ: 1 DSP
//   core arithmetic: 3 DSP (x*cφ, y*sφ, (x*cφ - y*sφ)*cθ)
//   K_turn multiply: 1 DSP
//   (CORDICs are separate blocks)
//
// Pipeline latency
//   deg2phase      : DP_IDLY + 1 + DP_ODLY
//   cordic_dds     : CORDIC_LAT
//   core arithmetic: 3*LM + 2    (LM = M_IDLY + 1 + M_ODLY)
//   total valid    : DEG2PH_LAT + CORDIC_LAT + (3*LM) + 2
// ============================================================================

module phase_calc #(
    parameter int CORDIC_DW   = 16,   // cordic_dds.DW
    parameter int CORDIC_LAT  = 16,   // cordic_dds.PIPE_DEPTH

    // mul_dsp pipeline (per multiply): IDLY + 1 + ODLY
    parameter int M_IDLY      = 1,
    parameter int M_ODLY      = 1,

    // deg2phase pipeline (per instance): IDLY + 1 + ODLY
    parameter int DP_IDLY     = 1,
    parameter int DP_ODLY     = 1
)(
    input  logic               clk,
    input  logic               rst_n,

    input  logic               start,            // 1-cycle pulse
    input  logic               is_tx,            // 1: 29.5 GHz, 0: 19.7 GHz

    input  logic       [15:0]  x_offset,         // Q9.7 [mm] (unsigned at top)
    input  logic       [15:0]  y_offset,         // Q9.7 [mm] (unsigned at top)
    input  logic       [15:0]  az_deg,           // Q9.7 [deg] 0..360
    input  logic       [15:0]  el_deg,           // Q9.7 [deg] 0..90

    output logic       [31:0]  phase_turn,       // Q1.31 wrapped to [0,1) for visibility
    output logic        [5:0]  phase_idx,        // 0..63 (direct from signed Q1.31)
    output logic               busy,
    output logic               valid
);
    // ------------------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------------------
    // AIR speed (pre-used to compute K_turn offline):
    //   K_turn = -f / c_mmps  [turns/mm]
    //   Encoded here as Q0.17 (18-bit signed two's complement)
    localparam logic signed [17:0] K0_TURN_TX_Q0_17 = 18'sh3CD9A; // -0.0984309286 turns/mm
    localparam logic signed [17:0] K0_TURN_RX_Q0_17 = 18'sh3DE58; // -0.0657318404 turns/mm

    // Scale cuts & final shift in core
    localparam int S_A             = 8;   // Q10.22 → >>>8 → Q10.14 (24b)
    localparam int S_T             = 15;  // Q11.29 → >>>15 → Q11.14 (25b)
    localparam int SHIFT_TO_Q1_31  = 0;   // Q11.31 → >>>0 → Q1.31 (use phase_full[31:0] directly)

    // Pipeline bookkeeping
    localparam int LM              = M_IDLY + 1 + M_ODLY;               // per mul_dsp
    localparam int DEG2PH_LAT      = DP_IDLY + 1 + DP_ODLY;
    localparam int ALIGN_CORD      = DEG2PH_LAT + CORDIC_LAT;
    localparam int LAT_TOTAL       = DEG2PH_LAT + CORDIC_LAT + (3*LM) + 2;

    // ------------------------------------------------------------------------
    // 1) Degrees(Q9.7, unsigned) → CORDIC phase codes (DW bits)
    // ------------------------------------------------------------------------
    logic v_phi_code, v_th_code;
    logic [CORDIC_DW-1:0] phi_i, th_i;

    deg2phase #(.DW(CORDIC_DW), .IDLY(DP_IDLY), .ODLY(DP_ODLY)) u_deg2phase_phi (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (start),
        .deg_in    (az_deg),          // Q9.7
        .phase_i   (phi_i),           // DW bits
        .valid_out (v_phi_code)
    );

    deg2phase #(.DW(CORDIC_DW), .IDLY(DP_IDLY), .ODLY(DP_ODLY)) u_deg2phase_th (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (start),
        .deg_in    (el_deg),          // Q9.7
        .phase_i   (th_i),            // DW bits
        .valid_out (/* v_th_code */)
    );

    // ------------------------------------------------------------------------
    // 2) Align x/y/is_tx to CORDIC outputs
    // ------------------------------------------------------------------------
    logic signed [15:0] x_d, y_d;
    logic               is_tx_d, start_a;

    delay #(.W(16), .N(ALIGN_CORD)) u_dly_x (.clk(clk), .rst_n(rst_n), .din($signed(x_offset)), .dout(x_d));
    delay #(.W(16), .N(ALIGN_CORD)) u_dly_y (.clk(clk), .rst_n(rst_n), .din($signed(y_offset)), .dout(y_d));
    delay #(.W(1),  .N(ALIGN_CORD)) u_dly_t (.clk(clk), .rst_n(rst_n), .din(is_tx),            .dout(is_tx_d));
    delay #(.W(1),  .N(CORDIC_LAT)) u_dly_a (.clk(clk), .rst_n(rst_n), .din(v_phi_code),        .dout(start_a));

    // ------------------------------------------------------------------------
    // 3) CORDIC (φ, θ) → (sφ,cφ) and (sθ,cθ)  [DW:0] → [DW-1:0] Q1.15 signed
    // ------------------------------------------------------------------------
    logic signed [CORDIC_DW:0] sphi_w, cphi_w, sth_w, cth_w;
    logic signed [CORDIC_DW-1:0] sphi_q, cphi_q, cth_q;
//    logic                      err_phi, err_th;

    cordic_dds #(.DW(CORDIC_DW)) u_cordic_phi (
        .clk     (clk),
        .phase_i (phi_i),
        .sin_o   (sphi_w),
        .cos_o   (cphi_w),
        .err_o   ()
    );
    cordic_dds #(.DW(CORDIC_DW)) u_cordic_th (
        .clk     (clk),
        .phase_i (th_i),
        .sin_o   (sth_w),
        .cos_o   (cth_w),
        .err_o   ()
    );

    // Keep true Q1.15 by taking lower 16 bits (no extra >>1)
    assign sphi_q = sphi_w[CORDIC_DW-1:0];
    assign cphi_q = cphi_w[CORDIC_DW-1:0];
    assign cth_q  = cth_w [CORDIC_DW-1:0];

    // ------------------------------------------------------------------------
    // 4) Arithmetic core (optimized): cθ * ( x*cφ - y*sφ )
    //    A) x*cφ, y*sφ    16×16 → Q10.22  >>>8 → Q10.14 (24b)
    //    B) diff          24b   → Q10.14
    //    C) *cθ           24×16 → Q11.29  >>>15 → Q11.14 (25b)
    // ------------------------------------------------------------------------
    logic                      v_a_xphi, v_a_yspi;
    logic signed [31:0]        x_cphi_full, y_sphi_full;
    logic signed [23:0]        x_cphi_24,   y_sphi_24;

    mul_dsp #(.WA(16), .WB(CORDIC_DW), .IDLY(M_IDLY), .ODLY(M_ODLY)) u_mul_x_cphi (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (start_a),
        .a         (x_d),       // Q9.7
        .b         (cphi_q),    // Q1.15
        .valid_out (v_a_xphi),
        .p         (x_cphi_full) // Q10.22
    );

    mul_dsp #(.WA(16), .WB(CORDIC_DW), .IDLY(M_IDLY), .ODLY(M_ODLY)) u_mul_y_sphi (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (start_a),
        .a         (y_d),       // Q9.7
        .b         (sphi_q),    // Q1.15
        .valid_out (/*v_a_yspi*/),
        .p         (y_sphi_full) // Q10.22
    );

    // Shift to Q10.14 and register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cphi_24 <= '0;
            y_sphi_24 <= '0;
        end else begin
            x_cphi_24 <= $signed(x_cphi_full >>> S_A); // S_A = 8
            y_sphi_24 <= $signed(y_sphi_full >>> S_A);
        end
    end

    // Difference: t_pre = x*cφ - y*sφ  (Q10.14, 24b)
    logic signed [23:0] t_pre_24;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) t_pre_24 <= '0;
        else        t_pre_24 <= $signed(x_cphi_24) - $signed(y_sphi_24);
    end
    // Align valid for the registered shift (v_a_xphi == v_a_yspi)
    logic v_a_shift;
    delay #(.W(1), .N(2)) u_v_a_shift (.clk(clk), .rst_n(rst_n), .din(v_a_xphi), .dout(v_a_shift));

    // Multiply by cθ: 24×16 → Q11.29
    logic                      v_b_t, v_c;
    logic signed [39:0]        t_cth_full;
    mul_dsp #(.WA(24), .WB(CORDIC_DW), .IDLY(M_IDLY), .ODLY(M_ODLY)) u_mul_t_cth (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (v_a_shift),
        .a         (t_pre_24),  // Q10.14
        .b         (cth_q),     // Q1.15
        .valid_out (v_b_t),
        .p         (t_cth_full) // Q11.29
    );

    // Final cut to Q11.14 (25b), then delay valid by 1 to match the register
    localparam int T_WIDTH = 25;
    logic signed [T_WIDTH-1:0] t_mm_25;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) t_mm_25 <= '0;
        else        t_mm_25 <= $signed(t_cth_full >>> S_T); // S_T = 15
    end
    delay #(.W(1), .N(4)) u_dly_v_c (.clk(clk), .rst_n(rst_n), .din(v_b_t), .dout(v_c));

    // ------------------------------------------------------------------------
    // 5) Multiply by K_turn (Q0.17): 25×18 → Q11.31, >>>10 → Q1.31
    // ------------------------------------------------------------------------
    localparam int PH_WIDTH = 43;

    logic               v_d, v_e;
    logic signed [17:0] k0_turn_q0_17;
    logic signed [PH_WIDTH-1:0] phase_full;
    logic signed [31:0] turn_q1_31_s;
    logic       [31:0]  frac_q1_31, turn_wrap_q1_31;

    // Align is_tx to the point we choose K_turn (kept symmetric with older pipe)
    localparam int PHASE_ALIGN = (2*LM) + 1;
    logic is_tx_k0_d;
    delay #(.W(1), .N(PHASE_ALIGN)) u_dly_is_tx_k0 (.clk(clk), .rst_n(rst_n), .din(is_tx_d), .dout(is_tx_k0_d));

    always_comb begin
        k0_turn_q0_17 = is_tx_k0_d ? K0_TURN_TX_Q0_17 : K0_TURN_RX_Q0_17;
    end

    mul_dsp #(.WA(T_WIDTH), .WB(18), .IDLY(M_IDLY), .ODLY(M_ODLY)) u_mul_t_k0 (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (v_c),
        .a         (t_mm_25),          // Q11.14
        .b         (k0_turn_q0_17),    // Q0.17
        .valid_out (v_d),
        .p         (phase_full)        // Q11.31
    );

    // Signed Q1.31 (can be negative before wrap)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) turn_q1_31_s <= '0;
        else        turn_q1_31_s <= $signed(phase_full[31:0]); // Q1.31 directly (no shift)
    end
    delay #(.W(1), .N(1)) u_dly_v_e (.clk(clk), .rst_n(rst_n), .din(v_d), .dout(v_e));

    // ----- Direct 6-bit index from signed Q1.31 ------------------------------
    // ----- Outputs ------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_turn <= '0;
            phase_idx  <= '0;
        end else if (v_e) begin
            phase_turn <= turn_q1_31_s;      // wrapped Q1.31 (0..1)
            phase_idx  <= turn_q1_31_s[30:25] + turn_q1_31_s[24]; // use wrapped fraction for index, round
        end
    end

    // ----- valid / busy -------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid <= 1'b0;
            busy  <= 1'b0;
        end else begin
            valid <= v_e;
            if (start) busy <= 1'b1;
            else if (v_e) busy <= 1'b0;
        end
    end
endmodule
