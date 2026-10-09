//============================================================
// tb_sdram_ctrl.v - testbench for sdram_ctrl
// Phases: init, functional, random, stress, corner, reset, performance
// Run:  make sim      (iverilog)   or   make stress  (fast refresh)
//============================================================
`timescale 1ns/1ps
module tb_sdram_ctrl;

    parameter REF_INTERVAL = 1500;
    parameter INIT_CYCLES  = 200;     // short power-up delay for sim

    reg         clk = 0;
    reg         rst_n = 0;
    reg         req = 0, wr = 0;
    reg  [21:0] addr = 0;
    reg  [63:0] wdata = 0;
    wire        ready, done;
    wire [63:0] rdata;

    wire        cke, cs_n, ras_n, cas_n, we_n;
    wire [1:0]  ba, dqm;
    wire [11:0] a;
    wire [15:0] dq;

    always #5 clk = ~clk;       // 100 MHz

    sdram_ctrl #(.INIT_CYCLES(INIT_CYCLES), .REF_INTERVAL(REF_INTERVAL)) dut (
        .clk(clk), .rst_n(rst_n),
        .req(req), .wr(wr), .addr(addr), .wdata(wdata),
        .ready(ready), .done(done), .rdata(rdata),
        .sdr_cke(cke), .sdr_cs_n(cs_n), .sdr_ras_n(ras_n), .sdr_cas_n(cas_n),
        .sdr_we_n(we_n), .sdr_ba(ba), .sdr_a(a), .sdr_dqm(dqm), .sdr_dq(dq)
    );

    sdram_model #(.T_REF_MAX(REF_INTERVAL + 60)) mdl (
        .clk(clk), .rst_n(rst_n), .cke(cke),
        .cs_n(cs_n), .ras_n(ras_n), .cas_n(cas_n), .we_n(we_n),
        .ba(ba), .a(a), .dqm(dqm), .dq(dq)
    );

    // ---------------- reference memory (scoreboard) ----------------
    reg [63:0] ref_mem [0:1048575];   // indexed by addr[21:2]
    reg        ref_v   [0:1048575];
    integer k;
    initial for (k = 0; k < 1048576; k = k + 1) ref_v[k] = 0;

    // ---------------- counters ----------------
    integer cycle = 0;
    always @(posedge clk) cycle <= cycle + 1;

    integer pass_cnt = 0, fail_cnt = 0;
    integer wr_lat_sum = 0, wr_n = 0, rd_lat_sum = 0, rd_n = 0;
    integer ref_cnt = 0;
    integer data_cycles = 0;
    reg     count_data = 0;

    always @(posedge clk) begin
        if (count_data && (dut.dq_oe | mdl.dq_oe)) data_cycles <= data_cycles + 1;
        if ({cs_n, ras_n, cas_n, we_n} == 4'b0001 && mdl.init_done) ref_cnt <= ref_cnt + 1;
    end

    always @(posedge clk) if ({cs_n,ras_n,cas_n,we_n} == 4'b0001 && mdl.init_done) $display("REF command at %0t", $time);
    integer seed = 12345;

    // ---------------- one transfer ----------------
    // returns read data in last_rdata, latency in last_lat
    reg [63:0] last_rdata;
    integer    last_lat;
    integer    t_req;

    task xfer(input w, input [21:0] ad, input [63:0] d);
        begin
            @(posedge clk); #1;
            while (!ready) begin @(posedge clk); #1; end
            req = 1; wr = w; addr = ad; wdata = d;
            t_req = cycle;
            @(posedge clk); #1;
            req = 0;
            while (!done) begin @(posedge clk); #1; end
            last_lat   = cycle - t_req;
            last_rdata = rdata;
            if (w) begin
                ref_mem[ad[21:2]] = d;
                ref_v[ad[21:2]]   = 1;
                wr_lat_sum = wr_lat_sum + last_lat; wr_n = wr_n + 1;
            end else begin
                rd_lat_sum = rd_lat_sum + last_lat; rd_n = rd_n + 1;
            end
        end
    endtask

    task check_read(input [21:0] ad);
        begin
            xfer(0, ad, 64'd0);
            if (ref_v[ad[21:2]]) begin
                if (last_rdata === ref_mem[ad[21:2]]) pass_cnt = pass_cnt + 1;
                else begin
                    fail_cnt = fail_cnt + 1;
                    $display("FAIL: addr=%h got=%h exp=%h", ad, last_rdata, ref_mem[ad[21:2]]);
                end
            end
        end
    endtask

    task write_then_read(input [21:0] ad, input [63:0] d);
        begin
            xfer(1, ad, d);
            check_read(ad);
        end
    endtask

    function [21:0] rand_addr(input dummy);
        reg [31:0] r;
        begin
            r = $random(seed);
            rand_addr = r[21:0] & 22'h3FFFFC;
        end
    endfunction

    function [63:0] rand_data(input dummy);
        begin
            rand_data = {$random(seed), $random(seed)};
        end
    endfunction

    // ---------------- main test ----------------
    integer i, start_cyc, end_cyc;
    reg [21:0] pool [0:63];
    reg [21:0] ta;

    initial begin
        $dumpfile("../results/waves.vcd");
        $dumpvars(0, tb_sdram_ctrl);

        // ===== 1. reset + init =====
        $display("== Phase 1: reset and init ==");
        rst_n = 0;
        repeat (5) @(posedge clk);
        #1 rst_n = 1;
        t_req = cycle;
        while (!ready) begin @(posedge clk); #1; end
        $display("init finished after %0d cycles, model init_done=%0d", cycle - t_req, mdl.init_done);
        if (mdl.init_done) pass_cnt = pass_cnt + 1; else fail_cnt = fail_cnt + 1;

        // ===== 2. functional =====
        $display("== Phase 2: functional ==");
        write_then_read(22'h000000, 64'h1111_2222_3333_4444);
        write_then_read(22'h000004, 64'hAAAA_BBBB_CCCC_DDDD);
        write_then_read(22'h100010, 64'h0123_4567_89AB_CDEF);   // bank 1
        write_then_read(22'h200020, 64'hFEDC_BA98_7654_3210);   // bank 2
        write_then_read(22'h300030, 64'hDEAD_BEEF_CAFE_F00D);   // bank 3
        write_then_read(22'h012344, 64'h0000_0001_0000_0002);   // some other row
        // re-read everything again to make sure nothing was damaged
        check_read(22'h000000); check_read(22'h000004);
        check_read(22'h100010); check_read(22'h200020); check_read(22'h300030);

        // ===== 3. random =====
        $display("== Phase 3: randomized ==");
        for (i = 0; i < 64; i = i + 1) pool[i] = rand_addr(0);
        for (i = 0; i < 300; i = i + 1) begin
            ta = pool[$random(seed) & 63];
            if ($random(seed) & 1) xfer(1, ta, rand_data(0));
            else                   check_read(ta);
        end

        // ===== 4. stress =====
        $display("== Phase 4: stress (back to back, many refreshes) ==");
        for (i = 0; i < 1000; i = i + 1) begin
            ta = pool[$random(seed) & 63];
            if (($random(seed) & 3) == 0) xfer(1, ta, rand_data(0));
            else                          check_read(ta);
        end
        // bank / row thrash
        for (i = 0; i < 100; i = i + 1) begin
            ta = {i[1:0], i[11:0] + 12'd7, 8'h00};
            write_then_read(ta, rand_data(0));
        end

        // ===== 5. corner cases =====
        $display("== Phase 5: corner cases ==");
        write_then_read(22'h000000, 64'h0);
        write_then_read(22'h3FFFFC, 64'hFFFF_FFFF_FFFF_FFFF);
        write_then_read({2'd0, 12'hFFF, 8'hFC}, 64'h5555_AAAA_5555_AAAA);
        write_then_read({2'd3, 12'h000, 8'h00}, 64'h8000_4000_2000_1000);
        write_then_read({2'd1, 12'hFFF, 8'h00}, 64'h0001_0002_0004_0008);
        xfer(1, 22'h0ABCD0, 64'h1);                       // overwrite same address
        xfer(1, 22'h0ABCD0, 64'h2);
        check_read(22'h0ABCD0);
        for (i = 0; i < 4; i = i + 1)                     // walking ones
            write_then_read({i[1:0], 12'h123, 8'h08}, 64'h1 << (i*16 + 5));

        // ===== 6. reset testing =====
        $display("== Phase 6: reset in the middle of operations ==");
        // 6a: reset during a write (row is open)
        @(posedge clk); #1;
        while (!ready) begin @(posedge clk); #1; end
        req = 1; wr = 1; addr = 22'h055550; wdata = 64'h1234_5678_9ABC_DEF0;
        @(posedge clk); #1; req = 0;
        repeat (6) @(posedge clk);
        #1 rst_n = 0;
        ref_v[22'h055550 >> 2] = 0;               // we don't know what was stored
        repeat (3) @(posedge clk);
        #1 rst_n = 1;
        while (!ready) begin @(posedge clk); #1; end
        $display("recovered from reset during write, model init_done=%0d", mdl.init_done);
        // 6b: reset during a read
        req = 1; wr = 0; addr = 22'h000000;
        @(posedge clk); #1; req = 0;
        repeat (5) @(posedge clk);
        #1 rst_n = 0;
        repeat (2) @(posedge clk);
        #1 rst_n = 1;
        while (!ready) begin @(posedge clk); #1; end
        // data written before the resets must still be there, new data must work
        check_read(22'h000004);
        check_read(22'h300030);
        write_then_read(22'h055550, 64'hABCD_ABCD_ABCD_ABCD);

        // ===== 7. performance =====
        $display("== Phase 7: performance ==");
        @(posedge clk); #1;
        while (!ready) begin @(posedge clk); #1; end
        wr_lat_sum = 0; wr_n = 0; rd_lat_sum = 0; rd_n = 0;
        ref_cnt = 0; data_cycles = 0; count_data = 1;
        start_cyc = cycle;
        for (i = 0; i < 200; i = i + 1) xfer(1, {2'd0, 12'd5, i[5:0], 2'b00} , rand_data(0));
        for (i = 0; i < 200; i = i + 1) check_read({2'd0, 12'd5, i[5:0], 2'b00});
        end_cyc = cycle;
        count_data = 0;

        $display("---------------- RESULTS ----------------");
        $display("total transfers      : %0d writes, %0d reads", wr_n, rd_n);
        $display("total cycles         : %0d", end_cyc - start_cyc);
        $display("avg write latency    : %0d cycles", wr_lat_sum / wr_n);
        $display("avg read  latency    : %0d cycles", rd_lat_sum / rd_n);
        $display("data bus busy cycles : %0d", data_cycles);
        $display("bus utilization      : %0d.%0d %%", (data_cycles*100)/(end_cyc-start_cyc),
                                                      ((data_cycles*1000)/(end_cyc-start_cyc)) % 10);
        $display("refreshes issued     : %0d", ref_cnt);
        $display("throughput           : %0d words/1000 cycles", ((wr_n+rd_n)*4*1000)/(end_cyc-start_cyc));
        $display("checks passed        : %0d", pass_cnt);
        $display("checks failed        : %0d", fail_cnt);
        $display("model protocol errors: %0d", mdl.errors);
        if (fail_cnt == 0 && mdl.errors == 0) $display("*** TEST PASSED ***");
        else                                  $display("*** TEST FAILED ***");
        $finish;
    end

    // watchdog
    initial begin
        #50_000_000;
        $display("TIMEOUT - something is stuck");
        $finish;
    end

endmodule
