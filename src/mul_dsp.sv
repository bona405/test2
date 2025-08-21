// mul_dsp.sv
// 3-part pipeline: input delay (IDLY) → DSP stage (1) → output delay (ODLY)
// Signed multiply; set IDLY/ODLY as needed (IDLY=1, ODLY=2 by default).
module mul_dsp #(
    parameter int WA   = 16,   // width of a
    parameter int WB   = 16,   // width of b
    parameter int IDLY = 1,    // input delay cycles (before DSP)
    parameter int ODLY = 2     // output delay cycles (after DSP)
)(
    input  logic                     clk,
    input  logic                     rst_n,

    input  logic                     valid_in,   // accepts new data each cycle
    input  logic signed [WA-1:0]     a,
    input  logic signed [WB-1:0]     b,

    output logic                     valid_out,  // delayed by IDLY+1+ODLY
    output logic signed [WA+WB-1:0]  p           // product
);
    localparam int WP = WA + WB;

    // ------------------------
    // Input delay alignment
    // ------------------------
    logic signed [WA-1:0] a_d;
    logic signed [WB-1:0] b_d;
    logic                 v_d;

    delay #(.W(WA), .N(IDLY)) u_dly_a (.clk(clk), .rst_n(rst_n), .din(a),        .dout(a_d));
    delay #(.W(WB), .N(IDLY)) u_dly_b (.clk(clk), .rst_n(rst_n), .din(b),        .dout(b_d));
    delay #(.W(1),  .N(IDLY)) u_dly_v (.clk(clk), .rst_n(rst_n), .din(valid_in), .dout(v_d));

    // ------------------------
    // DSP multiply stage (1 clk)
    // ------------------------
    (* use_dsp = "yes" *) logic signed [48-1:0] m1;
    logic v1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m1 <= '0;
            v1 <= 1'b0;
        end else begin
            m1 <= a_d * b_d;  // single DSP48 multiply
            v1 <= v_d;
        end
    end

    // ------------------------
    // Output delay alignment
    // ------------------------
    delay #(.W(WP), .N(ODLY)) u_dly_p  (.clk(clk), .rst_n(rst_n), .din(m1), .dout(p));
    delay #(.W(1),  .N(ODLY)) u_dly_vo (.clk(clk), .rst_n(rst_n), .din(v1), .dout(valid_out));

endmodule
