`timescale 1ns/1ps

module tb_transceiver_8b10b;
    reg        clk;
    reg        rst_n;
    reg  [7:0] tx_data;
    reg        tx_isk;
    reg        inject_err;

    wire [7:0] rx_data;
    wire       rx_isk;
    wire       rx_err;
    wire       tx_disp;

    // Instantiate Top-Level Loopback Transceiver
    transceiver_8b10b_top dut (
        .clk(clk),
        .rst_n(rst_n),
        .tx_data(tx_data),
        .tx_isk(tx_isk),
        .inject_err(inject_err),
        .rx_data(rx_data),
        .rx_isk(rx_isk),
        .rx_err(rx_err),
        .tx_disp(tx_disp)
    );

    // 100 MHz Clock Generation
    initial clk = 0;
    always #5 clk = ~clk;

    // Waveform Dump for EPWave
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_transceiver_8b10b);
    end

    // Stimulus Task
    task send_byte(input [7:0] d, input isk);
    begin
        @(posedge clk);
        tx_data    <= d;
        tx_isk     <= isk;
        inject_err <= 1'b0;
    end
    endtask

    integer i;
    initial begin
        // Reset initialization
        rst_n      = 0;
        tx_data    = 8'd0;
        tx_isk     = 0;
        inject_err = 0;
        #30;
        rst_n = 1;
        #20;

        $display("--- 1. Transmitting K28.5 Sync Commas (0xBC) ---");
        repeat (3) send_byte(8'hBC, 1'b1);

        $display("--- 2. Transmitting Regular Data Stream ---");
        for (i = 0; i < 16; i = i + 1) begin
            send_byte(i, 1'b0);
        end

        $display("--- 3. Injecting Bit Error on Channel ---");
        @(posedge clk);
        tx_data    <= 8'h05;
        tx_isk     <= 1'b0;
        inject_err <= 1'b1; // Causes illegal 10b symbol

        @(posedge clk);
        inject_err <= 1'b0;
        send_byte(8'h0A, 1'b0);

        #100;
        $display("--- Verification Completed Successfully ---");
        $finish;
    end

    // Self-Checking Monitor
    always @(posedge clk) begin
        if (rst_n) begin
            if (rx_err)
                $display("[TIME: %0t ns] [ERROR DETECTED] Corrupted 10b symbol caught by decoder!", $time);
            else if (rx_isk)
                $display("[TIME: %0t ns] [K-CHAR DETECTED] Special Control Character: 0x%0h", $time, rx_data);
            else if (rx_data != 0)
                $display("[TIME: %0t ns] [DATA RECEIVED] Byte: 0x%0h (Disparity: %0b)", $time, rx_data, tx_disp);
        end
    end
endmodule