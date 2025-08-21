// Generic delay line
// - W: data width
// - N: number of cycles to delay (0 = passthrough)
module delay #(
    parameter int W = 1,
    parameter int N = 1
)(
    input  logic           clk,
    input  logic           rst_n,
    input  logic [W-1:0]   din,
    output logic [W-1:0]   dout
);
    generate
        if (N == 0) begin : g_passthru
            assign dout = din;
        end else begin : g_pipe
            logic [W-1:0] pipe [0:N-1];
            integer i;
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    for (i = 0; i < N; i++) pipe[i] <= '0;
                end else begin
                    pipe[0] <= din;
                    for (i = 1; i < N; i++) pipe[i] <= pipe[i-1];
                end
            end
            assign dout = pipe[N-1];
        end
    endgenerate
endmodule
