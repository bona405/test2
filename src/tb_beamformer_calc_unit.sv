`timescale 1ns/1ps

module tb_beamformer_calc_unit;

    // --------- tune for quick sim ----------
    localparam int ROWS_TB   = 4;     // use 32 for full sweep
    localparam int SPI_ID_TB = 7;     // COL_BASE = (7-7)*4 = 0

    // --------- clock / reset ----------
    logic clk, rst_n;
    initial clk = 1'b0;
    always #5 clk = ~clk;              // 100 MHz

    initial begin
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
    end

    // --------- DUT I/O ----------
    logic        start;
    logic        isTX;
    logic [15:0] az_deg_q9_7;          // Q9.7 degrees
    logic [15:0] el_deg_q9_7;          // Q9.7 degrees

    // simple SPI write channel (byte-stream)
    logic [7:0]  spi_wdata;
    logic        spi_wvalid;
    logic        spi_wready;
    logic        spi_wlast;

    // optional observability
    logic        busy, valid, done;    // if your DUT doesn't have 'done', ignore it
    logic [31:0] phase_turn_q1_31;
    logic [5:0]  phase_idx_out;

    // --------- instantiate DUT ----------
    beamformer_calc_unit #(
        .SPI_ID (0),
        .ROWS   (32)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .isTX       (isTX),
        .az_deg     (az_deg_q9_7),
        .el_deg     (el_deg_q9_7),

        // SPI byte stream
        .spi_wdata  (spi_wdata),
        .spi_wvalid (spi_wvalid),
        .spi_wready (spi_wready),
        .spi_wlast  (spi_wlast),

        // status / observability
//        .busy       (busy),
//        .valid      (valid),
//        .done       (done),               // if not present in DUT, comment out
        .phase_turn (phase_turn_q1_31),
        .phase_idx  (phase_idx_out)
    );

    // always ready (no back-pressure)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) spi_wready <= 1'b0;
        else        spi_wready <= 1'b1;
    end

    // --------- helpers ----------
    localparam real Q9_7 = 128.0;

    function automatic [15:0] deg_to_q9_7 (real d);
        real r; begin
            r = d * Q9_7;
            if (r >= 0.0) deg_to_q9_7 = $rtoi(r + 0.5);
            else          deg_to_q9_7 = $rtoi(r - 0.5);
        end
    endfunction

    // --------- simple SPI frame sniffer (5 bytes/frame) ----------
    integer frame_cnt;
    integer byte_cnt;
    reg [7:0] b0, b1, b2, b3, b4;   // last captured frame
    reg [15:0] value16;
    reg [5:0]  chip_id, phase_idx;
    // capture bytes on handshake and print per frame
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_cnt <= 0;
            byte_cnt  <= 0;
            b0 <= '0; b1 <= '0; b2 <= '0; b3 <= '0; b4 <= '0;
        end else begin
            if (spi_wvalid && spi_wready) begin
                case (byte_cnt)
                    0: b0 <= spi_wdata;
                    1: b1 <= spi_wdata;
                    2: b2 <= spi_wdata;
                    3: b3 <= spi_wdata;
                    4: b4 <= spi_wdata;
                    default: ;
                endcase
                if (byte_cnt == 4) begin
                    // full 5-byte frame received - print a concise line
                    frame_cnt <= frame_cnt + 1;
                    byte_cnt  <= 0;

                    // decode a few interesting fields
                    value16   = {b3, b4};
                    chip_id   = b1[5:0];
                    phase_idx = value16[15:10];

                    $display("F%0d : b0=%02h chip=%0d ch=%02h val=%04h idx=%0d  (phase_idx_out=%0d)",
                              frame_cnt, b0, chip_id, b2, value16, phase_idx, phase_idx_out);

                    if (b0 !== 8'h28) begin
                        $display("  NOTE: header != 0x28 (got %02h)", b0);
                    end
                end else begin
                    byte_cnt <= byte_cnt + 1;
                end
            end
        end
    end

    // --------- stimulus tasks ----------
    task run_sweep (bit tx, real az_deg, real el_deg);
        integer exp_frames;
        begin
            isTX        = tx;
            az_deg_q9_7 = deg_to_q9_7(az_deg);
            el_deg_q9_7 = deg_to_q9_7(el_deg);

            // reset counters
            frame_cnt = 0;
            byte_cnt  = 0;

            // start pulse
            start = 1'b1; @(posedge clk); start = 1'b0;

            // wait until expected number of frames observed
            exp_frames = ROWS_TB * 4; // 4 columns per SPI
            while (frame_cnt < exp_frames) @(posedge clk);

            // small gap
            repeat (10) @(posedge clk);
        end
    endtask

    // --------- test sequence ----------
    initial begin
        // defaults
        start       = 1'b0;
        isTX        = 1'b0;
        az_deg_q9_7 = '0;
        el_deg_q9_7 = '0;

        // wait reset
        wait (rst_n);
        @(posedge clk);

        // TX sweep
        $display("\n--- TX sweep: az=30, el=60 ---");
        run_sweep(1'b1, 30.0, 60.0);
        run_sweep(1'b1, 30.0, 60.0);
        
        // RX sweep
        $display("\n--- RX sweep: az=30, el=60 ---");
        run_sweep(1'b0, 30.0, 60.0);

        $display("\nTB done. Frames per sweep = %0d\n", ROWS_TB*4);
        #50;
        $finish;
    end

    // optional waves
    // initial begin
    //     $dumpfile("tb_beamformer_calc_unit_simple.vcd");
    //     $dumpvars(0, tb_beamformer_calc_unit_simple);
    // end

endmodule
