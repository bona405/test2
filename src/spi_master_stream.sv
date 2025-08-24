// ============================================================================
// spi_master_stream.sv
// ----------------------------------------------------------------------------
// Consumes a byte stream (wdata/wvalid/wready/wlast) and emits SPI signals
//   - Mode 0 by default (CPOL=0, CPHA=0)  MSB-first
//   - One-byte load per handshake; keeps CS_n low across a frame (until wlast)
//   - Parameterizable serial clock divider: SCLK = clk / (2*DIV)
// ----------------------------------------------------------------------------
// Inputs
//   clk, rst_n
//   spi_wdata  [7:0]   : byte to send (MSB first)
//   spi_wvalid         : valid when a new byte is offered
//   spi_wlast          : marks the last byte of the current frame
// Outputs
//   spi_wready         : high when ready to accept a byte
//   sclk, mosi, cs_n   : SPI pins (no MISO here)
// ----------------------------------------------------------------------------

module spi_master_stream #(
    parameter int unsigned DIV   = 4,   // >= 2 : SCLK = clk/(2*DIV)
    parameter bit          CPOL  = 1'b0,
    parameter bit          CPHA  = 1'b0,
    parameter bit          MSB_FIRST = 1'b1
)(
    input  logic       clk,
    input  logic       rst_n,

    // Stream-in (from beamformer_top_spi lane)
    input  logic [7:0] spi_wdata,
    input  logic       spi_wvalid,
    output logic       spi_wready,
    input  logic       spi_wlast,

    // SPI
    output logic       sclk,
    output logic       mosi,
    output logic       cs_n
);

    // ------------- guards / locals -------------
    localparam int C_BITS = (DIV <= 2) ? 1 : $clog2(DIV);
    initial begin
        // Basic param sanity (ignored by most synth, fine for simulation)
        if (DIV < 2) $error("DIV must be >= 2");
        if (CPHA != 1'b0) $warning("Only CPHA=0 is implemented in this RTL.");
        if (!MSB_FIRST)   $warning("Only MSB_FIRST=1 is implemented in this RTL.");
    end

    typedef enum logic [1:0] {ST_IDLE, ST_LOAD, ST_SHIFT} state_e;
    state_e state;

    logic [C_BITS-1:0] cdiv;
    logic              tick;          // SCLK half-period strobe
    logic [7:0]        sh;            // shift register (MSB-first)
    logic [3:0]        bits_left;     // counts rising edges remaining (0..8)
    logic              last_byte_l;   // latched 'wlast' with the current byte
    logic              busy;          // shifting a byte

    // SCLK and MOSI
    logic sclk_r, mosi_r;
    assign sclk = sclk_r;
    assign mosi = mosi_r;

    // Divider: only runs in ST_SHIFT
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cdiv <= '0;
            tick <= 1'b0;
        end else begin
            if (state == ST_SHIFT) begin
                if (cdiv == DIV-1) begin
                    cdiv <= '0;
                    tick <= 1'b1;
                end else begin
                    cdiv <= cdiv + 1'b1;
                    tick <= 1'b0;
                end
            end else begin
                cdiv <= '0;
                tick <= 1'b0;
            end
        end
    end

    // SCLK generation (CPOL idle level) only in ST_SHIFT
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_r <= CPOL;
        end else if (state == ST_SHIFT) begin
            if (tick) sclk_r <= ~sclk_r;
        end else begin
            sclk_r <= CPOL; // idle level when not shifting
        end
    end

    // Rising/falling edge detect for the *output* SCLK (with CPOL baseline)
    wire edge_pos = (tick && (state == ST_SHIFT) && (sclk_r == ~CPOL)); // about to rise to CPOL^1
    wire edge_neg = (tick && (state == ST_SHIFT) && (sclk_r ==  CPOL)); // about to fall to CPOL^0

    // State machine + datapath
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            sh           <= 8'h00;
            bits_left    <= 4'd0;
            last_byte_l  <= 1'b0;
            mosi_r       <= 1'b0;
            cs_n         <= 1'b1;
            busy         <= 1'b0;
            spi_wready   <= 1'b1;
        end else begin
            // defaults
            spi_wready <= 1'b0;

            unique case (state)
            // ------------------------------------------------------------
            ST_IDLE: begin
                cs_n       <= 1'b1;
                busy       <= 1'b0;
                spi_wready <= 1'b1;            // ready for the first byte of a frame
                if (spi_wvalid) begin
                    // Latch first byte
                    sh          <= spi_wdata;
                    last_byte_l <= spi_wlast;
                    bits_left   <= 4'd8;
                    mosi_r      <= spi_wdata[7]; // MSB first, CPHA=0
                    cs_n        <= 1'b0;         // assert CS for the frame
                    busy        <= 1'b1;
                    state       <= ST_SHIFT;
                end
            end

            // ------------------------------------------------------------
            ST_LOAD: begin
                // Waiting for a subsequent byte within the same frame (CS held low)
                cs_n       <= 1'b0;
                busy       <= 1'b0;
                spi_wready <= 1'b1;
                if (spi_wvalid) begin
                    sh          <= spi_wdata;
                    last_byte_l <= spi_wlast;
                    bits_left   <= 4'd8;
                    mosi_r      <= spi_wdata[7];
                    busy        <= 1'b1;
                    state       <= ST_SHIFT;
                end
            end

            // ------------------------------------------------------------
            ST_SHIFT: begin
                // CPHA=0:
                //   - MOSI must be stable before SCLK rising edge (sampling edge)
                //   - Shift the register on SCLK *falling* edge
                if (edge_pos) begin
                    // A bit has just been sampled by the slave
                    if (bits_left == 4'd1) begin
                        // Byte complete on this rising edge
                        busy <= 1'b0;
                        if (last_byte_l) begin
                            // End of frame
                            cs_n  <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            // More bytes to come in this frame
                            state <= ST_LOAD;  // keep CS low, pause SCLK, request next byte
                        end
                    end else begin
                        bits_left <= bits_left - 1'b1;
                    end
                end

                if (edge_neg) begin
                    // Prepare next bit on falling edge (CPHA=0)
                    if (bits_left > 4'd1) begin
                        sh     <= {sh[6:0], 1'b0};
                        mosi_r <= sh[6];
                    end
                end
            end

            endcase
        end
    end

endmodule
