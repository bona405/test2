module beamformer_top #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter NUM_FIFO   = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Control
    input  wire                  start,
    input  wire [15:0]           az_value,
    input  wire [15:0]           el_value,
    input  wire                  isTX,

    // FIFO write interface for 8 SPI channels
    output wire [ADDR_WIDTH-1:0] fifo_addr   [NUM_FIFO-1:0], // FIFO address for each channel
    output wire [DATA_WIDTH-1:0] fifo_wdata  [NUM_FIFO-1:0], // FIFO write data for each channel
    output wire [NUM_FIFO-1:0]   fifo_wen,                   // FIFO write enable for each channel

    // (Optional) FIFO status
    input  wire [NUM_FIFO-1:0]   fifo_full,                  // FIFO full flags

    // (Optional) Done signal
    output wire                  done
);

    localparam ROWS = 32;
    localparam COLS_PER_SPI = 4; // Each SPI handles 4 columns
    
    reg [5:0] row; // 0~31
    reg [1:0] col; // 0~3 (local column for this SPI)
    reg [2:0] spi_id; // Provided as input/parameter

    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : calc_unit_gen
            beamformer_calc_unit #(
                .ADDR_WIDTH(32),
                .DATA_WIDTH(32),
                .SPI_ID(i)
            ) u_calc_unit (
                .clk        (clk),
                .rst_n      (rst_n),
                .start      (start),
                .isTX       (isTX),
                .az_value   (az_value),
                .el_value   (el_value),
                .spi_id     (i[2:0]),
    
                .fifo_addr  (fifo_addr[i]),
                .fifo_wdata (fifo_wdata[i]),
                .fifo_wen   (fifo_wen[i]),
                .fifo_full  (fifo_full[i]),
                .done       (/* connect to done logic if needed */)
            );
        end
    endgenerate

endmodule