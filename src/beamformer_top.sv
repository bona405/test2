// ============================================================================
// beamformer_top_spi
//   - Instantiates 8 beamformer_calc_unit blocks (SPI_ID = 0..7)
//   - Each instance generates its own 5-byte command stream for its SPI lane
//   - COL_BASE is assigned as (7 - SPI_ID) * 4, so lanes cover all 32 columns
//   - az/el/isTX are common; start is broadcast to all lanes
//   - Per-lane spi_* handshake and per-lane busy/done are exposed
// ============================================================================

module beamformer_top #(
    parameter int NUM_SPI = 8
)(
    input  logic clk,
    input  logic rst_n,

    // control (common to all lanes)
    input  logic start,        // 1-cycle pulse: start a full sweep on all lanes
    input  logic isTX,         // 1: TX, 0: RX (shared)
    input  logic [15:0]  az_deg,       // Q9.7 [deg] (shared)
    input  logic [15:0]  el_deg,       // Q9.7 [deg] (shared)

    output logic [NUM_SPI-1:0] spi_sclk,
    output logic [NUM_SPI-1:0] spi_cs_n,
    output logic [NUM_SPI-1:0] spi_mosi,

    // per-lane status
    output logic [NUM_SPI-1:0] busy,       
    output logic [NUM_SPI-1:0] done,

    // optional debug taps (directly from calc units)
    output logic [31:0] phase_turn [NUM_SPI-1:0],  // Q1.31 wrapped
    output logic [5:0]  phase_idx  [NUM_SPI-1:0]
);
    logic spi_wready [NUM_SPI-1:0];
    logic [7:0] spi_wdata  [NUM_SPI-1:0];
    logic spi_wvalid [NUM_SPI-1:0];
    logic spi_wlast  [NUM_SPI-1:0];

    logic start_d;
    delay #(.W(1), .N(1)) start_edge_ (.clk(clk), .rst_n(rst_n), .din(start), .dout(start_d));
    logic start_edge = start & ~start_d;
    // ------------------------------------------------------------------------
    // 8 beamformer_calc_unit instances
    //   SPI_ID = i
    //   COL_BASE = (7 - i) * 4   → lanes map to 32 columns without overlap
    // ------------------------------------------------------------------------
    genvar i;
        for (i = 0; i < NUM_SPI; i++) begin : g_lane
            localparam int LP_SPI_ID   = i;
            localparam int LP_COL_BASE = (7 - i) * 4;

            beamformer_calc_unit #(
                .SPI_ID   (LP_SPI_ID),
                .ROWS     (32),
                .COL_BASE (LP_COL_BASE)
            ) u_calc (
                .clk         (clk),
                .rst_n       (rst_n),

                .start       (start_edge),          // broadcast start to all lanes
                .isTX        (isTX),           // shared TX/RX mode
                .az_deg      (az_deg),         // shared az (Q9.7)
                .el_deg      (el_deg),         // shared el (Q9.7)

                .spi_wdata   (spi_wdata [i]),
                .spi_wvalid  (spi_wvalid[i]),
                .spi_wready  (spi_wready[i]),
                .spi_wlast   (spi_wlast [i]),

                .busy        (busy      [i]),
                .done        (done      [i]),

                .phase_turn  (phase_turn[i]),  // debug
                .phase_idx   (phase_idx [i])   // debug
            );
        end

        for (i = 0; i < NUM_SPI; i++) begin : g_spi
            spi_master_stream #(
                .DIV(4)          // SCLK = clk/(2*DIV) → here clk/8
            ) u_spi (
                .clk        (clk),
                .rst_n      (rst_n),
                .spi_wdata  (spi_wdata[i]),
                .spi_wvalid (spi_wvalid[i]),
                .spi_wready (spi_wready[i]),   // backpressure to your byte generator
                .spi_wlast  (spi_wlast[i]),
                .sclk       (spi_sclk[i]),
                .mosi       (spi_mosi[i]),
                .cs_n       (spi_cs_n[i])
            );
        end

endmodule
