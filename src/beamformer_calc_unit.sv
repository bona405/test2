// ============================================================================
// beamformer_calc_unit (SPI byte-stream generator; 5 bytes per element)
// ----------------------------------------------------------------------------
// For each element (row 0..31, col_idx 0..3 → col=COL_BASE+col_idx):
//   - x=col*dx, y=row*dy (Q9.7 mm; dx,dy=5.0/7.5 mm in Q9.7)
//   - phase = phase_calc(...), phase_idx in 0..63
//   - Emit 5 bytes over spi_* (one command):
//       b0=0x28, b1=chip_id(6b, zero-extended), b2=channel_id,
//       b3=value_hi, b4=value_lo
//     where value16 is:
//       TX: [15:10]=phase_idx, [9:8]=2'b11, [7:1]=7'h7F, [0]=0
//       RX: [15:10]=phase_idx, [9:4]=6'd63, [3]=1, other low bits 0
//
// Handshake:
//   spi_wdata[7:0], spi_wvalid -> spi_wready, spi_wlast(=1 on 5th byte)
//   'busy' asserted during sweep; 'done' pulses when the LAST byte of the
//   LAST element is accepted.
//
// Q-formats (unchanged):
//   x_offset, y_offset : Q9.7 [mm]
//   az_deg, el_deg     : Q9.7 [deg]
//   phase_turn         : Q1.31 (signed) from phase_calc
//   phase_idx          : 6-bit
// ============================================================================
module beamformer_calc_unit #(
    parameter int SPI_ID     = 0,
    parameter int ROWS       = 32,
    parameter int COL_BASE   = (7 - SPI_ID) * 4
)(
    input  logic        clk,
    input  logic        rst_n,

    // control
    input  logic        start,          // pulse to start full sweep (32*4 elems)
    input  logic        isTX,           // 1: TX, 0: RX
    input  logic [15:0] az_deg,         // Q9.7 [deg]
    input  logic [15:0] el_deg,         // Q9.7 [deg]

    // SPI byte stream (5 bytes per element)
    output logic [7:0]  spi_wdata,
    output logic        spi_wvalid,
    input  logic        spi_wready,
    output logic        spi_wlast,      // 1 on the 5th byte of each element

    // status / debug
    output logic        busy,           // sweep in progress
    output logic        done,           // pulse when last byte of last elem accepted
    output logic [31:0] phase_turn,     // Q1.31 (from phase_calc), for debug
    output logic [5:0]  phase_idx       // for debug
);

    // ------------------------------------------------------------------------
    // Spacing (Q9.7 mm)
    // ------------------------------------------------------------------------
    localparam logic [15:0] DX_TX = 16'd640;  //  5.0 * 2^7
    localparam logic [15:0] DX_RX = 16'd960;  //  7.5 * 2^7
    localparam logic [15:0] DY_TX = 16'd640;
    localparam logic [15:0] DY_RX = 16'd960;

    wire [15:0] dx = isTX ? DX_TX : DX_RX;
    wire [15:0] dy = isTX ? DY_TX : DY_RX;

    // ------------------------------------------------------------------------
    // Indexing
    // ------------------------------------------------------------------------
    logic [5:0] row;
    logic [1:0] col_idx;
    wire  [5:0] col = (6'(COL_BASE)) + col_idx;       // 0..31

    // chip_id = (col_idx < 2) ? (16 + row/2) : (row/2)
    wire [5:0] chip_id = (col_idx < 2) ? (6'(16) + row[5:1]) : row[5:1];

    // Channel mapping (TX/RX; even/odd column)
    localparam logic [7:0] TX_EVEN [0:3] = '{8'h27,8'h3F,8'h47,8'h5F};
    localparam logic [7:0] TX_ODD  [0:3] = '{8'h5F,8'h47,8'h3F,8'h27};
    localparam logic [7:0] RX_EVEN [0:3] = '{8'h22,8'h3A,8'h42,8'h5A};
    localparam logic [7:0] RX_ODD  [0:3] = '{8'h5A,8'h42,8'h3A,8'h22};

    wire [7:0] channel_id = (col[0] == 1'b0)
                          ? (isTX ? TX_EVEN[row[1:0]] : RX_EVEN[row[1:0]])
                          : (isTX ? TX_ODD [row[1:0]] : RX_ODD [row[1:0]]);

    // -------------------------------------------------------------------------
    // ROMs for x_offset(col) and y_offset(row) in Q9.7 mm (16-bit)
    //   X_ROM_TX[i] = i * 640  (5.0 mm * 2^7)
    //   X_ROM_RX[i] = i * 960  (7.5 mm * 2^7)
    //   Y_ROM_TX[r] = r * 640
    //   Y_ROM_RX[r] = r * 960
    // These are compile-time constants → LUT ROMs (no multipliers).
    // -------------------------------------------------------------------------
    localparam logic [15:0] X_ROM_TX [0:31] = '{
        16'd0,    16'd640,  16'd1280, 16'd1920, 16'd2560, 16'd3200, 16'd3840, 16'd4480,
        16'd5120, 16'd5760, 16'd6400, 16'd7040, 16'd7680, 16'd8320, 16'd8960, 16'd9600,
        16'd10240,16'd10880,16'd11520,16'd12160,16'd12800,16'd13440,16'd14080,16'd14720,
        16'd15360,16'd16000,16'd16640,16'd17280,16'd17920,16'd18560,16'd19200,16'd19840
    };

    localparam logic [15:0] X_ROM_RX [0:31] = '{
        16'd0,    16'd960,  16'd1920, 16'd2880, 16'd3840, 16'd4800, 16'd5760, 16'd6720,
        16'd7680, 16'd8640, 16'd9600, 16'd10560,16'd11520,16'd12480,16'd13440,16'd14400,
        16'd15360,16'd16320,16'd17280,16'd18240,16'd19200,16'd20160,16'd21120,16'd22080,
        16'd23040,16'd24000,16'd24960,16'd25920,16'd26880,16'd27840,16'd28800,16'd29760
    };

    localparam logic [15:0] Y_ROM_TX [0:31] = '{
        16'd0,    16'd640,  16'd1280, 16'd1920, 16'd2560, 16'd3200, 16'd3840, 16'd4480,
        16'd5120, 16'd5760, 16'd6400, 16'd7040, 16'd7680, 16'd8320, 16'd8960, 16'd9600,
        16'd10240,16'd10880,16'd11520,16'd12160,16'd12800,16'd13440,16'd14080,16'd14720,
        16'd15360,16'd16000,16'd16640,16'd17280,16'd17920,16'd18560,16'd19200,16'd19840
    };

    localparam logic [15:0] Y_ROM_RX [0:31] = '{
        16'd0,    16'd960,  16'd1920, 16'd2880, 16'd3840, 16'd4800, 16'd5760, 16'd6720,
        16'd7680, 16'd8640, 16'd9600, 16'd10560,16'd11520,16'd12480,16'd13440,16'd14400,
        16'd15360,16'd16320,16'd17280,16'd18240,16'd19200,16'd20160,16'd21120,16'd22080,
        16'd23040,16'd24000,16'd24960,16'd25920,16'd26880,16'd27840,16'd28800,16'd29760
    };
    wire logic [15:0] x_offset = isTX ? X_ROM_TX[col] : X_ROM_RX[col];
    wire logic [15:0] y_offset = isTX ? Y_ROM_TX[row] : Y_ROM_RX[row];
    
    // ------------------------------------------------------------------------
    // Phase calculation
    // ------------------------------------------------------------------------
    logic phase_start, phase_valid;

    phase_calc #(
        .CORDIC_DW (16),
        .CORDIC_LAT(16),
        .M_IDLY    (1),
        .M_ODLY    (1),
        .DP_IDLY   (1),
        .DP_ODLY   (1)
    ) u_phase_calc (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (phase_start),
        .is_tx      (isTX),
        .x_offset   ($signed(x_offset)),
        .y_offset   ($signed(y_offset)),
        .az_deg     (az_deg),
        .el_deg     (el_deg),
        .phase_turn (phase_turn),
        .phase_idx  (phase_idx),
        .busy       (/* unused */),
        .valid      (phase_valid)
    );

    // ------------------------------------------------------------------------
    // Simple sweep controller:
    //   - Start computation for an element
    //   - Wait for phase_valid
    //   - Emit 5 bytes (respecting spi_wready)
    //   - Then advance to next element and repeat
    //   - 'done' pulses when the *last* byte of the last element is accepted
    // ------------------------------------------------------------------------
    typedef enum logic [1:0] {IDLE, WAIT_PHASE, SEND5} state_e;
    state_e state;

    // Byte assembly for current element
    logic [7:0] b0, b1, b2, b3, b4;
    logic [2:0] byte_idx;      // 0..4
    logic [15:0] value16;

    // Last-element detector
    wire last_elem = (row == ROWS-1) && (col_idx == 2'd3);

    // Compose 16-bit value from phase_idx and isTX
    always_comb begin
        if (isTX) begin
            // TX: [15:10]=phase_idx, [9:8]=3, [7:1]=127, [0]=0
            value16 = (16'(phase_idx) << 10)
                    | (16'(2'b11)     << 8)
                    | (16'(7'h7F)     << 1); //gain
        end else begin
            // RX: [15:10]=phase_idx, [9:4]=63, [3]=1
            value16 = (16'(phase_idx) << 10)
                    | (16'(6'd63)     << 4) //gain 
                    | (16'(1'b1)      << 3);
        end
    end

    // Output handshake defaults
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            phase_start <= 1'b0;

            row         <= '0;
            col_idx     <= '0;

            spi_wdata  <= 8'h00;
            spi_wvalid <= 1'b0;
            spi_wlast  <= 1'b0;

            b0 <= '0; b1 <= '0; b2 <= '0; b3 <= '0; b4 <= '0;
            byte_idx <= 3'd0;
        end else begin
            // defaults
            spi_wvalid  <= 1'b0;
            spi_wlast   <= 1'b0;
            phase_start <= 1'b0;
            done        <= 1'b0;

            unique case (state)
            IDLE: begin
                if (start) begin
                    busy        <= 1'b1;
                    row         <= 6'd0;
                    col_idx     <= 2'd0;
                    phase_start <= 1'b1;         // kick compute for first element
                    state       <= WAIT_PHASE;
                end
            end

            WAIT_PHASE: begin
                if (phase_valid) begin
                    // Latch 5 bytes for this element
                    b0 <= 8'h28;
                    b1 <= {2'b00, chip_id};      // chip_id is 6-bit
                    b2 <= channel_id;
                    b3 <= value16[15:8];
                    b4 <= value16[7:0];
                    byte_idx <= 3'd0;
                    state    <= SEND5;
                end
            end

            SEND5: begin
                // Present current byte when SPI is ready to accept it
                if (spi_wready) begin
                    // Select byte
                    case (byte_idx)
                        3'd0: spi_wdata <= b0;
                        3'd1: spi_wdata <= b1;
                        3'd2: spi_wdata <= b2;
                        3'd3: spi_wdata <= b3;
                        default: spi_wdata <= b4;
                    endcase
                    spi_wvalid <= 1'b1;
                    spi_wlast  <= (byte_idx == 3'd4);

                    // Advance byte pointer when we actually transfer
                    byte_idx <= (byte_idx == 3'd4) ? 3'd0 : (byte_idx + 3'd1);

                    if (byte_idx == 3'd4) begin
                        // Finished this element; advance indices
                        if (last_elem) begin
                            // This was the last element of the sweep
                            busy <= 1'b0;
                            done <= 1'b1;     // pulse 'done' on last byte accepted
                            state <= IDLE;
                        end else begin
                            // Move to next element and kick its compute
                            if (col_idx == 2'd3) begin
                                col_idx     <= 2'd0;
                                row         <= row + 6'd1;
                            end else begin
                                col_idx     <= col_idx + 2'd1;
                            end
                            phase_start <= 1'b1;   // start next element compute
                            state       <= WAIT_PHASE;
                        end
                    end
                end
            end

            endcase
        end
    end

endmodule
