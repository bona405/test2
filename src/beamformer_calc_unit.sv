module beamformer_calc_unit #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter SPI_ID     = 0,
    parameter ROWS       = 32,
    parameter COL_BASE   = (7 - SPI_ID) * 4
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        isTX,          // 1: TX, 0: RX
    input  wire [31:0] az_deg,        // [deg], Q16.16
    input  wire [31:0] el_deg,        // [deg], Q16.16
    output reg  [31:0] fifo_addr,
    output reg  [31:0] fifo_wdata,
    output reg         fifo_wen,
    input  wire        fifo_full,
    output reg         done
);
    // spacing (Q16.16 mm)
    localparam [31:0] DX_TX = 32'd327680;  // 5.0 * 65536
    localparam [31:0] DX_RX = 32'd491520;  // 7.5 * 65536
    localparam [31:0] DY_TX = 32'd327680;
    localparam [31:0] DY_RX = 32'd491520;
    wire [31:0] dx = isTX ? DX_TX : DX_RX;
    wire [31:0] dy = isTX ? DY_TX : DY_RX;

    // indexing
    reg  [5:0] row;
    reg  [1:0] col_idx;
    wire [5:0] col     = COL_BASE + col_idx;
    wire [5:0] chip_id = (col_idx < 2) ? (6'(16 + (row>>1))) : (row>>1);

    // channels (TX/RX tables)
    localparam logic [7:0] TX_EVEN [0:3] = '{8'h27,8'h3F,8'h47,8'h5F};
    localparam logic [7:0] TX_ODD  [0:3] = '{8'h5F,8'h47,8'h3F,8'h27};
    localparam logic [7:0] RX_EVEN [0:3] = '{8'h22,8'h3A,8'h42,8'h5A};
    localparam logic [7:0] RX_ODD  [0:3] = '{8'h5A,8'h42,8'h3A,8'h22};
    wire [7:0] channel_id = (col[0]==1'b0)
      ? (isTX ? TX_EVEN[row[1:0]] : RX_EVEN[row[1:0]])
      : (isTX ? TX_ODD [row[1:0]] : RX_ODD [row[1:0]]);

    // offsets (Q16.16 mm)
    wire [31:0] x_offset = col * dx;
    wire [31:0] y_offset = row * dy;

    // -------- poles in radians (Q3.29) --------
    localparam [31:0] P_30_RAD_Q3_29  = 32'h10C15238; //  π/6
    localparam [31:0] P_120_RAD_Q3_29 = 32'h430548E1; // 2π/3
    localparam [31:0] P_210_RAD_Q3_29 = 32'h75493F89; // 7π/6
    localparam [31:0] P_300_RAD_Q3_29 = 32'hA78D3632; // 5π/3

    wire [31:0] poles_rad = ({row[0], col[0]} == 2'b00) ? P_120_RAD_Q3_29 :
                            ({row[0], col[0]} == 2'b01) ? P_30_RAD_Q3_29  :
                            ({row[0], col[0]} == 2'b10) ? P_210_RAD_Q3_29 :
                                                          P_300_RAD_Q3_29 ;

    // -------- phase_calc --------
    wire [31:0] phase_turn; // Q1.31
    wire [5:0]  phase_idx;
    phase_calc #( //p = k0 * ( x*cθ*cφ + y*cθ*sφ ), x,y in mm
        .CORDIC_DW (16)   // cordic_dds DW
//        .CORDIC_LAT(14)   // cordic_dds PIPE_DEPTH
    ) u_phase_calc (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),       // 1-cycle pulse per element
        .is_tx      (isTX),        // map your isTX → is_tx
    
        .x_offset   (x_offset),    // Q16.16 [mm]
        .y_offset   (y_offset),    // Q16.16 [mm]
        .az_deg     (az_deg),    // Q16.16 [deg]
        .el_deg     (el_deg),    // Q16.16 [deg]
    
        .phase_turn (phase_turn), // Q1.31
        .phase_idx  (phase_idx),      // 0..63 (5.625° steps)
        .busy       (phase_busy),
        .valid      (phase_valid)
    );

    // TODO: convert poles_rad → turns and add to phase_turn_no_pole
    //       then quantize & pack to fifo_wdata/value exactly like your C++.
endmodule
