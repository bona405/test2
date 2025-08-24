`timescale 1ns/1ps

module tb_beamformer_top;

    // ------------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------------
    localparam int NUM_SPI = 8;

    // ------------------------------------------------------------------------
    // Clock / Reset
    // ------------------------------------------------------------------------
    logic clk, rst_n;
    initial clk = 1'b0;
    always #5 clk = ~clk;  // 100 MHz

    initial begin
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
    end

    // ------------------------------------------------------------------------
    // DUT control
    // ------------------------------------------------------------------------
    logic        start;
    logic        isTX;
    logic [15:0] az_deg_q9_7;  // Q9.7 degrees
    logic [15:0] el_deg_q9_7;  // Q9.7 degrees

    // Q9.7 helper
    function automatic logic [15:0] deg_to_q9_7(input real deg);
        deg_to_q9_7 = $rtoi(deg * 128.0); // 2^7
    endfunction

    // ------------------------------------------------------------------------
    // DUT SPI pins (per lane)
    // ------------------------------------------------------------------------
    logic spi_sclk [NUM_SPI-1:0];
    logic spi_cs_n [NUM_SPI-1:0];
    logic spi_mosi [NUM_SPI-1:0];

    // (optional status taps from DUT)
    logic        busy [NUM_SPI-1:0];
    logic        done [NUM_SPI-1:0];
    logic [31:0] phase_turn [NUM_SPI-1:0];
    logic  [5:0] phase_idx  [NUM_SPI-1:0];

    // ------------------------------------------------------------------------
    // DUT instance
    // ------------------------------------------------------------------------
    beamformer_top #(
        .NUM_SPI(NUM_SPI)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),

        .start      (start),
        .isTX       (isTX),
        .az_deg     (az_deg_q9_7),
        .el_deg     (el_deg_q9_7),

        .spi_sclk   (spi_sclk),
        .spi_cs_n   (spi_cs_n),
        .spi_mosi   (spi_mosi),

        .busy       (busy),
        .done       (done),

        .phase_turn (phase_turn),
        .phase_idx  (phase_idx)
    );

    // ------------------------------------------------------------------------
    // Simple SPI monitors per lane (Mode 0: sample MOSI on SCLK rising)
    //   - Reconstruct 5-byte frames while CS_N is low
    //   - Print first N frames per lane
    // ------------------------------------------------------------------------
    localparam int MAX_PRINT_PER_LANE = 6;

    typedef struct packed {
        byte b0, b1, b2, b3, b4;
    } frame5_t;

    // Per-lane collectors
    generate
        for (genvar i = 0; i < NUM_SPI; i++) begin : g_mon
            frame5_t cur;
            int      byte_idx;
            int      bit_idx;
            int      printed;

            initial begin
                cur      = '{default:0};
                byte_idx = 0;
                bit_idx  = 7;
                printed  = 0;
            end

            // Detect frame start/end by cs_n transitions
            logic cs_n_q;
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) cs_n_q <= 1'b1;
                else        cs_n_q <= spi_cs_n[i];
            end

            // On CS_N falling: reset byte/bit counters
            always_ff @(posedge clk) begin
                if (cs_n_q && !spi_cs_n[i]) begin
                    byte_idx <= 0;
                    bit_idx  <= 7;
                    cur      <= '{default:0};
                end
            end

            // Sample on rising edge of SCLK while CS is low (SPI mode 0)
            always_ff @(posedge clk) begin
                if (!rst_n) begin
                    // already initialized
                end else if (!spi_cs_n[i]) begin
                    // look for SCLK rising edge
                    // create a simple edge detect for sclk
                    logic sclk_q;
                    sclk_q <= spi_sclk[i];

                    // On rising edge of SCLK: shift in MOSI (MSB-first)
                    if (!sclk_q && spi_sclk[i]) begin
                        case (byte_idx)
                            0: cur.b0[bit_idx] <= spi_mosi[i];
                            1: cur.b1[bit_idx] <= spi_mosi[i];
                            2: cur.b2[bit_idx] <= spi_mosi[i];
                            3: cur.b3[bit_idx] <= spi_mosi[i];
                            4: cur.b4[bit_idx] <= spi_mosi[i];
                            default: ; // ignore extra
                        endcase

                        if (bit_idx == 0) begin
                            bit_idx <= 7;
                            if (byte_idx < 4)
                                byte_idx <= byte_idx + 1;
                            else begin
                                // Completed 5 bytes
                                // Decode value16 and phase_idx for readability
                                logic [15:0] value16;
                                logic [5:0]  phidx;
                                value16 = {cur.b3, cur.b4};
                                phidx   = value16[15:10];

                                if (printed < MAX_PRINT_PER_LANE) begin
                                    $display("[%0t] LANE%0d FRAME%0d  b0=0x%02h b1=0x%02h b2=0x%02h b3=0x%02h b4=0x%02h  value16=0x%04h  phase_idx=%0d",
                                             $time, i, printed,
                                             cur.b0, cur.b1, cur.b2, cur.b3, cur.b4,
                                             value16, phidx);
                                end
                                printed <= printed + 1;

                                // Prepare for next frame (still under same CS if master continues)
                                byte_idx <= 0;
                                cur      <= '{default:0};
                            end
                        end else begin
                            bit_idx <= bit_idx - 1;
                        end
                    end
                end
            end
        end
    endgenerate

    // ------------------------------------------------------------------------
    // Stimulus
    // ------------------------------------------------------------------------
    initial begin
        start       = 1'b0;
        isTX        = 1'b0;
        az_deg_q9_7 = '0;
        el_deg_q9_7 = '0;

        // Wait for reset
        @(posedge rst_n);
        @(posedge clk);

        // Sweep #1: TX, az=30°, el=60°
        isTX        <= 1'b1;
        az_deg_q9_7 <= deg_to_q9_7(30.0);
        el_deg_q9_7 <= deg_to_q9_7(60.0);

        start <= 1'b1; @(posedge clk); start <= 1'b0;

        // Let it run; you can also wait for all 'done[i]' to assert
        repeat (80000) @(posedge clk);

        // Sweep #2: RX, az=35°, el=10°
        isTX        <= 1'b0;
        az_deg_q9_7 <= deg_to_q9_7(35.0);
        el_deg_q9_7 <= deg_to_q9_7(10.0);

        start <= 1'b1; @(posedge clk); start <= 1'b0;

        repeat (80000) @(posedge clk);

        $display("TB finished.");
        $finish;
    end

    // ------------------------------------------------------------------------
    // Optional VCD
    // ------------------------------------------------------------------------
    initial begin
        if ($test$plusargs("dump")) begin
            $dumpfile("tb_beamformer_top.vcd");
            $dumpvars(0, tb_beamformer_top);
        end
    end

endmodule
