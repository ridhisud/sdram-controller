//============================================================
// sdram_model.v - behavioural SDR SDRAM + simple protocol checker
// 4 banks x 4096 rows x 256 cols x 16 bit, CL=2, BL=4 only
// This is NOT a vendor model, just enough to test my controller.
// Every protocol mistake increments "errors" and prints a message.
//============================================================
module sdram_model #(
    parameter CL        = 2,
    parameter T_RP      = 2,
    parameter T_RCD     = 2,
    parameter T_RFC     = 7,
    parameter T_WR      = 2,
    parameter T_REF_MAX = 1600     // max cycles between refreshes
)(
    input  wire        clk,
    input  wire        rst_n,      // only used to restart the checker
    input  wire        cke,
    input  wire        cs_n, ras_n, cas_n, we_n,
    input  wire [1:0]  ba,
    input  wire [11:0] a,
    input  wire [1:0]  dqm,
    inout  wire [15:0] dq
);
    localparam CMD_ACT = 4'b0011;
    localparam CMD_RD  = 4'b0101;
    localparam CMD_WR  = 4'b0100;
    localparam CMD_PRE = 4'b0010;
    localparam CMD_REF = 4'b0001;
    localparam CMD_MRS = 4'b0000;

    reg [15:0] mem [0:4194303];       // 22 bit address

    reg        row_open  [0:3];
    reg [11:0] open_row  [0:3];
    integer    last_act  [0:3];
    integer    last_pre  [0:3];
    integer    last_wr   [0:3];
    integer    last_ref;       // last REF command (for tRFC)
    integer    last_ref_chk;   // last REF/MRS/reset (for the refresh deadline)
    integer    cyc;
    integer    errors;
    reg        init_done;
    integer    i;

    // read pipeline
    integer    rd_wait;
    integer    rd_beat;
    reg [1:0]  rd_ba;
    reg [11:0] rd_row;
    reg [7:0]  rd_col;
    reg [15:0] dq_out;
    reg        dq_oe;
    assign dq = dq_oe ? dq_out : 16'bz;

    // write burst in progress
    reg        wr_active;
    integer    wr_beat;
    reg [1:0]  wr_ba;
    reg [11:0] wr_row;
    reg [7:0]  wr_col;

    wire [3:0] cmd = {cs_n, ras_n, cas_n, we_n};

    task err(input [255:0] msg);
        begin
            errors = errors + 1;
            $display("[SDRAM MODEL ERROR] t=%0t cyc=%0d : %0s", $time, cyc, msg);
        end
    endtask

    initial begin
        errors = 0; cyc = 0; init_done = 0; last_ref = -100; last_ref_chk = 0;
        rd_wait = 0; rd_beat = 0; wr_active = 0; dq_oe = 0; dq_out = 0;
        for (i = 0; i < 4; i = i + 1) begin
            row_open[i] = 0; open_row[i] = 0;
            last_act[i] = -100; last_pre[i] = -100; last_wr[i] = -100;
        end
    end

    always @(posedge clk) begin
        cyc = cyc + 1;

        if (!rst_n) begin
            init_done = 0;
            last_ref_chk = cyc;
        end

        // ---------- continue a write burst (beats 1..3) ----------
        if (wr_active) begin
            mem[{wr_ba, wr_row, wr_col[7:2], wr_col[1:0] + wr_beat[1:0]}] = dq;
            wr_beat = wr_beat + 1;
            if (wr_beat == 4) wr_active = 0;
        end

        // ---------- read data pipeline ----------
        dq_oe <= 0;
        if (rd_wait > 1) begin
            rd_wait = rd_wait - 1;
        end else if (rd_wait == 1) begin
            dq_out <= mem[{rd_ba, rd_row, rd_col[7:2], rd_col[1:0] + rd_beat[1:0]}];
            dq_oe  <= 1;
            rd_beat = rd_beat + 1;
            if (rd_beat == 4) rd_wait = 0;
        end

        // ---------- commands ----------
        if (cke) begin
            case (cmd)
            CMD_MRS: begin
                if (a !== 12'b0000_0010_0010) err("mode register is not BL4/CL2");
                init_done = 1;
                last_ref_chk = cyc;
            end

            CMD_ACT: begin
                if (!init_done)          err("ACT before init finished");
                if (row_open[ba])        err("ACT to a bank that is already open");
                if (cyc - last_pre[ba] < T_RP)
                                         err("tRP violated (PRE -> ACT)");
                if (cyc - last_ref < T_RFC) err("tRFC violated (REF -> ACT)");
                row_open[ba] = 1;
                open_row[ba] = a;
                last_act[ba] = cyc;
            end

            CMD_RD: begin
                if (!row_open[ba])       err("READ to closed bank");
                if (cyc - last_act[ba] < T_RCD) err("tRCD violated (ACT -> READ)");
                rd_ba = ba; rd_row = open_row[ba]; rd_col = a[7:0];
                rd_wait = CL; rd_beat = 0;
            end

            CMD_WR: begin
                if (!row_open[ba])       err("WRITE to closed bank");
                if (cyc - last_act[ba] < T_RCD) err("tRCD violated (ACT -> WRITE)");
                wr_ba = ba; wr_row = open_row[ba]; wr_col = a[7:0];
                mem[{ba, open_row[ba], a[7:0]}] = dq;     // word 0 is with the command
                wr_beat = 1; wr_active = 1;
                last_wr[ba] = cyc;
            end

            CMD_PRE: begin
                if (a[10]) begin          // precharge all
                    for (i = 0; i < 4; i = i + 1) begin
                        if (cyc - last_wr[i] < 3 + T_WR) err("tWR violated (WRITE -> PRE)");
                        row_open[i] = 0; last_pre[i] = cyc;
                    end
                end else begin
                    if (cyc - last_wr[ba] < 3 + T_WR) err("tWR violated (WRITE -> PRE)");
                    row_open[ba] = 0; last_pre[ba] = cyc;
                end
            end

            CMD_REF: begin
                for (i = 0; i < 4; i = i + 1) begin
                    if (row_open[i])                  err("REF with a bank still open");
                    if (cyc - last_pre[i] < T_RP)     err("tRP violated (PRE -> REF)");
                end
                if (cyc - last_ref < T_RFC)           err("tRFC violated (REF -> REF)");
                last_ref = cyc;
                last_ref_chk = cyc;
            end

            default: ;   // NOP / others
            endcase
        end

        // ---------- refresh deadline ----------
        if (init_done && (cyc - last_ref_chk > T_REF_MAX)) begin
            err("refresh deadline missed - data would be lost");
            last_ref_chk = cyc;
        end
    end
endmodule
