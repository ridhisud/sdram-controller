//============================================================
// sdram_ctrl.v  -  Simple SDR SDRAM controller
// Assignment 2: High-Speed SDRAM Memory Controller
//
// Target device : 64Mbit SDR SDRAM (4 banks x 4096 rows x 256 cols x 16 bit)
// Clock         : 100 MHz (10 ns)  -> all timings below are in clock cycles
// CAS latency   : 2
// Burst length  : 4 (sequential)
// Page policy   : close page (every access = ACT -> RD/WR -> PRE)
//
// Host address map (22 bit word address):
//   addr[21:20] = bank
//   addr[19:8]  = row
//   addr[7:0]   = column   (addr[1:0] should be 0, burst is 4 words)
//
// Host data: one request moves a whole 4 word burst (64 bit)
//============================================================
module sdram_ctrl #(
    parameter INIT_CYCLES  = 10000,  // 100 us at 100 MHz
    parameter REF_INTERVAL = 1500,   // 15 us (spec is 15.6 us per row)
    parameter T_RP  = 2,
    parameter T_RCD = 2,
    parameter T_RFC = 7,
    parameter T_MRD = 2,
    parameter T_WR  = 2
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---------------- host side ----------------
    input  wire        req,        // request a transfer
    input  wire        wr,         // 1 = write, 0 = read
    input  wire [21:0] addr,
    input  wire [63:0] wdata,      // 4 words, word0 = wdata[15:0]
    output wire        ready,      // controller can take a request
    output reg         done,       // pulse: transfer finished
    output reg  [63:0] rdata,      // valid when done is high (reads)

    // ---------------- SDRAM side ----------------
    output reg         sdr_cke,
    output reg         sdr_cs_n,
    output reg         sdr_ras_n,
    output reg         sdr_cas_n,
    output reg         sdr_we_n,
    output reg  [1:0]  sdr_ba,
    output reg  [11:0] sdr_a,
    output wire [1:0]  sdr_dqm,
    inout  wire [15:0] sdr_dq
);

    // ---------- commands {cs_n, ras_n, cas_n, we_n} ----------
    localparam CMD_NOP = 4'b0111;
    localparam CMD_ACT = 4'b0011;
    localparam CMD_RD  = 4'b0101;
    localparam CMD_WR  = 4'b0100;
    localparam CMD_PRE = 4'b0010;
    localparam CMD_REF = 4'b0001;
    localparam CMD_MRS = 4'b0000;

    // mode register: BL=4, sequential, CL=2, normal write burst
    localparam [11:0] MODE = 12'b0000_0010_0010;

    // ---------- states ----------
    localparam S_INIT_WAIT = 4'd0;
    localparam S_INIT_PRE  = 4'd1;
    localparam S_INIT_REF1 = 4'd2;
    localparam S_INIT_REF2 = 4'd3;
    localparam S_INIT_MRS  = 4'd4;
    localparam S_IDLE      = 4'd5;
    localparam S_ACT       = 4'd6;
    localparam S_READ      = 4'd7;
    localparam S_RD_DATA   = 4'd8;
    localparam S_WRITE     = 4'd9;
    localparam S_WR_DATA   = 4'd10;
    localparam S_PRE       = 4'd11;
    localparam S_REFRESH   = 4'd12;
    localparam S_WAIT      = 4'd13;

    reg [3:0]  state;
    reg [3:0]  next_state;     // where S_WAIT goes when it finishes
    reg [15:0] wait_cnt;       // counter for S_WAIT and init delay
    reg [3:0]  cnt;            // beat counter for read/write bursts

    // latched request
    reg        l_wr;
    reg [21:0] l_addr;
    reg [63:0] l_wdata;

    // refresh timer
    reg [15:0] ref_timer;
    reg        ref_pending;

    // data bus
    reg [15:0] dq_out;
    reg        dq_oe;
    assign sdr_dq  = dq_oe ? dq_out : 16'bz;
    assign sdr_dqm = 2'b00;     // we never mask

    wire [1:0]  l_bank = l_addr[21:20];
    wire [11:0] l_row  = l_addr[19:8];
    wire [7:0]  l_col  = l_addr[7:0];

    assign ready = (state == S_IDLE) && !ref_pending;

    // helper: put a command on the pins
    task set_cmd(input [3:0] c);
        begin
            {sdr_cs_n, sdr_ras_n, sdr_cas_n, sdr_we_n} <= c;
        end
    endtask

    // ---------------- refresh timer ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ref_timer   <= 0;
            ref_pending <= 0;
        end else begin
            if (state == S_REFRESH) begin
                ref_timer   <= 0;
                ref_pending <= 0;
            end else if (ref_timer == REF_INTERVAL) begin
                ref_pending <= 1;
            end else begin
                ref_timer <= ref_timer + 1;
            end
        end
    end

    // ---------------- main FSM ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_INIT_WAIT;
            next_state <= S_INIT_WAIT;
            wait_cnt   <= 0;
            cnt        <= 0;
            done       <= 0;
            rdata      <= 0;
            l_wr       <= 0;
            l_addr     <= 0;
            l_wdata    <= 0;
            dq_out     <= 0;
            dq_oe      <= 0;
            sdr_cke    <= 0;
            sdr_ba     <= 0;
            sdr_a      <= 0;
            {sdr_cs_n, sdr_ras_n, sdr_cas_n, sdr_we_n} <= CMD_NOP;
        end else begin
            // defaults every cycle
            set_cmd(CMD_NOP);
            done  <= 0;
            dq_oe <= 0;
            sdr_cke <= 1;

            case (state)

            // ===== power up sequence =====
            S_INIT_WAIT: begin
                if (wait_cnt == INIT_CYCLES - 1) begin
                    wait_cnt <= 0;
                    state    <= S_INIT_PRE;
                end else
                    wait_cnt <= wait_cnt + 1;
            end

            S_INIT_PRE: begin           // precharge all banks (A10 = 1)
                set_cmd(CMD_PRE);
                sdr_a <= 12'b0100_0000_0000;
                sdr_ba <= 0;
                state <= S_WAIT; wait_cnt <= T_RP - 2; next_state <= S_INIT_REF1;
            end

            S_INIT_REF1: begin
                set_cmd(CMD_REF);
                state <= S_WAIT; wait_cnt <= T_RFC - 2; next_state <= S_INIT_REF2;
            end

            S_INIT_REF2: begin
                set_cmd(CMD_REF);
                state <= S_WAIT; wait_cnt <= T_RFC - 2; next_state <= S_INIT_MRS;
            end

            S_INIT_MRS: begin           // load mode register
                set_cmd(CMD_MRS);
                sdr_ba <= 0;
                sdr_a  <= MODE;
                state <= S_WAIT; wait_cnt <= T_MRD - 2; next_state <= S_IDLE;
            end

            // ===== idle, wait for something to do =====
            S_IDLE: begin
                if (ref_pending) begin
                    state <= S_REFRESH;          // refresh wins over host
                end else if (req) begin
                    l_wr    <= wr;
                    l_addr  <= addr;
                    l_wdata <= wdata;
                    state   <= S_ACT;
                end
            end

            // ===== open the row =====
            S_ACT: begin
                set_cmd(CMD_ACT);
                sdr_ba <= l_bank;
                sdr_a  <= l_row;
                state <= S_WAIT; wait_cnt <= T_RCD - 2;
                next_state <= l_wr ? S_WRITE : S_READ;
            end

            // ===== read burst =====
            S_READ: begin
                set_cmd(CMD_RD);
                sdr_ba <= l_bank;
                sdr_a  <= {4'b0000, l_col};      // A10 = 0 -> no auto precharge
                cnt    <= 1;
                state  <= S_RD_DATA;
            end

            S_RD_DATA: begin
                // READ cmd was k=0. With CL=2 and the registered pins the
                // 4 words show up on the bus at k = 4,5,6,7
                cnt <= cnt + 1;
                if (cnt >= 4 && cnt <= 7)
                    rdata <= {sdr_dq, rdata[63:16]};  // shift in, word0 ends in [15:0]
                if (cnt == 7) begin
                    done  <= 1;
                    state <= S_PRE;
                end
            end

            // ===== write burst =====
            S_WRITE: begin
                set_cmd(CMD_WR);
                sdr_ba <= l_bank;
                sdr_a  <= {4'b0000, l_col};
                dq_out <= l_wdata[15:0];         // first word goes with the command
                dq_oe  <= 1;
                cnt    <= 1;
                state  <= S_WR_DATA;
            end

            S_WR_DATA: begin
                dq_oe  <= 1;
                case (cnt)
                    4'd1: dq_out <= l_wdata[31:16];
                    4'd2: dq_out <= l_wdata[47:32];
                    default: dq_out <= l_wdata[63:48];
                endcase
                cnt <= cnt + 1;
                if (cnt == 3) begin
                    state <= S_WAIT; wait_cnt <= T_WR - 2; next_state <= S_PRE;
                end
            end

            // ===== close the row =====
            S_PRE: begin
                set_cmd(CMD_PRE);
                sdr_ba <= l_bank;
                sdr_a  <= 12'd0;                 // A10 = 0 -> only this bank
                if (l_wr) done <= 1;
                state <= S_WAIT; wait_cnt <= T_RP - 2; next_state <= S_IDLE;
            end

            // ===== auto refresh =====
            S_REFRESH: begin
                set_cmd(CMD_REF);
                state <= S_WAIT; wait_cnt <= T_RFC - 2; next_state <= S_IDLE;
            end

            // ===== just wait N cycles =====
            S_WAIT: begin
                if (wait_cnt == 0)
                    state <= next_state;
                else
                    wait_cnt <= wait_cnt - 1;
            end

            default: state <= S_INIT_WAIT;
            endcase
        end
    end

endmodule
