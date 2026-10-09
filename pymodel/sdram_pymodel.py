#!/usr/bin/env python3
"""
sdram_pymodel.py - cycle-by-cycle Python copy of rtl/sdram_ctrl.v + sim/sdram_model.v
+ the testbench phases.  Only exists because no Verilog simulator was available.
Run:  python3 sdram_pymodel.py [REF_INTERVAL]
Makes: results/waves.vcd, results/sim_log.txt, results/snaps.json
"""
import sys, random, json

NOP, ACT, RD, WR, PRE, REF, MRS = 0b0111, 0b0011, 0b0101, 0b0100, 0b0010, 0b0001, 0b0000
CMD_NAME = {NOP:"NOP", ACT:"ACT", RD:"READ", WR:"WRITE", PRE:"PRE", REF:"REF", MRS:"MRS"}
(S_INIT_WAIT, S_INIT_PRE, S_INIT_REF1, S_INIT_REF2, S_INIT_MRS, S_IDLE, S_ACT, S_READ,
 S_RD_DATA, S_WRITE, S_WR_DATA, S_PRE, S_REFRESH, S_WAIT) = range(14)
STATE_NAME = ["INIT_WAIT","INIT_PRE","INIT_REF1","INIT_REF2","INIT_MRS","IDLE","ACT","READ",
              "RD_DATA","WRITE","WR_DATA","PRE","REFRESH","WAIT"]
MODE = 0b000000100010
M16, M64 = 0xFFFF, (1 << 64) - 1


class Ctrl:
    def __init__(s, INIT, REFI, T_RP=2, T_RCD=2, T_RFC=7, T_MRD=2, T_WR=2):
        s.INIT, s.REFI, s.T_RP, s.T_RCD, s.T_RFC, s.T_MRD, s.T_WR = INIT, REFI, T_RP, T_RCD, T_RFC, T_MRD, T_WR
        s.reset()

    def reset(s):
        s.r = dict(state=S_INIT_WAIT, next_state=S_INIT_WAIT, wait_cnt=0, cnt=0, done=0, rdata=0,
                   l_wr=0, l_addr=0, l_wdata=0, dq_out=0, dq_oe=0, cke=0, ba=0, a=0, cmd=NOP,
                   ref_timer=0, ref_pending=0)

    @property
    def ready(s):
        return int(s.r["state"] == S_IDLE and not s.r["ref_pending"])

    def step(s, req, wr, addr, wdata, dq):
        c = s.r
        n = dict(c)
        # refresh timer
        if c["state"] == S_REFRESH:
            n["ref_timer"] = 0; n["ref_pending"] = 0
        elif c["ref_timer"] == s.REFI:
            n["ref_pending"] = 1
        else:
            n["ref_timer"] = c["ref_timer"] + 1
        # defaults
        n["cmd"] = NOP; n["done"] = 0; n["dq_oe"] = 0; n["cke"] = 1
        st = c["state"]

        def wait(cycles_minus2, nxt):
            n["state"] = S_WAIT; n["wait_cnt"] = cycles_minus2; n["next_state"] = nxt

        lb, lr, lc = (c["l_addr"] >> 20) & 3, (c["l_addr"] >> 8) & 0xFFF, c["l_addr"] & 0xFF
        if st == S_INIT_WAIT:
            if c["wait_cnt"] == s.INIT - 1:
                n["wait_cnt"] = 0; n["state"] = S_INIT_PRE
            else:
                n["wait_cnt"] = c["wait_cnt"] + 1
        elif st == S_INIT_PRE:
            n["cmd"] = PRE; n["a"] = 0b010000000000; n["ba"] = 0; wait(s.T_RP - 2, S_INIT_REF1)
        elif st == S_INIT_REF1:
            n["cmd"] = REF; wait(s.T_RFC - 2, S_INIT_REF2)
        elif st == S_INIT_REF2:
            n["cmd"] = REF; wait(s.T_RFC - 2, S_INIT_MRS)
        elif st == S_INIT_MRS:
            n["cmd"] = MRS; n["ba"] = 0; n["a"] = MODE; wait(s.T_MRD - 2, S_IDLE)
        elif st == S_IDLE:
            if c["ref_pending"]:
                n["state"] = S_REFRESH
            elif req:
                n["l_wr"] = wr; n["l_addr"] = addr; n["l_wdata"] = wdata; n["state"] = S_ACT
        elif st == S_ACT:
            n["cmd"] = ACT; n["ba"] = lb; n["a"] = lr
            wait(s.T_RCD - 2, S_WRITE if c["l_wr"] else S_READ)
        elif st == S_READ:
            n["cmd"] = RD; n["ba"] = lb; n["a"] = lc; n["cnt"] = 1; n["state"] = S_RD_DATA
        elif st == S_RD_DATA:
            n["cnt"] = (c["cnt"] + 1) & 0xF
            if 4 <= c["cnt"] <= 7:
                n["rdata"] = (((dq if dq is not None else 0) << 48) | (c["rdata"] >> 16)) & M64
                if dq is None: n["rdata"] |= 0  # z/x -> treated as 0
            if c["cnt"] == 7:
                n["done"] = 1; n["state"] = S_PRE
        elif st == S_WRITE:
            n["cmd"] = WR; n["ba"] = lb; n["a"] = lc
            n["dq_out"] = c["l_wdata"] & M16; n["dq_oe"] = 1; n["cnt"] = 1; n["state"] = S_WR_DATA
        elif st == S_WR_DATA:
            n["dq_oe"] = 1
            n["dq_out"] = (c["l_wdata"] >> (16 * {1: 1, 2: 2}.get(c["cnt"], 3))) & M16
            n["cnt"] = (c["cnt"] + 1) & 0xF
            if c["cnt"] == 3:
                wait(s.T_WR - 2, S_PRE)
        elif st == S_PRE:
            n["cmd"] = PRE; n["ba"] = lb; n["a"] = 0
            if c["l_wr"]: n["done"] = 1
            wait(s.T_RP - 2, S_IDLE)
        elif st == S_REFRESH:
            n["cmd"] = REF; wait(s.T_RFC - 2, S_IDLE)
        elif st == S_WAIT:
            if c["wait_cnt"] == 0: n["state"] = c["next_state"]
            else: n["wait_cnt"] = c["wait_cnt"] - 1
        s.r = n


class Sdram:
    def __init__(s, T_REF_MAX, CL=2, T_RP=2, T_RCD=2, T_RFC=7, T_WR=2):
        s.CL, s.T_RP, s.T_RCD, s.T_RFC, s.T_WR, s.T_REF_MAX = CL, T_RP, T_RCD, T_RFC, T_WR, T_REF_MAX
        s.mem = {}
        s.row_open = [0] * 4; s.open_row = [0] * 4
        s.last_act = [-100] * 4; s.last_pre = [-100] * 4; s.last_wr = [-100] * 4
        s.last_ref = -100; s.last_ref_chk = 0; s.cyc = 0; s.errors = 0; s.init_done = 0; s.log = []
        s.rd_wait = 0; s.rd_beat = 0; s.rd = (0, 0, 0)
        s.dq_out = 0; s.dq_oe = 0
        s.wr_active = 0; s.wr_beat = 0; s.wr = (0, 0, 0)

    def err(s, msg):
        s.errors += 1
        s.log.append(f"[SDRAM MODEL ERROR] cyc={s.cyc} : {msg}")

    @staticmethod
    def addr(b, row, col, beat):
        return (b << 20) | (row << 8) | ((col & 0xFC) | ((col + beat) & 3))

    def step(s, cmd, ba, a, cke, rst_n, dq):
        s.cyc += 1; cyc = s.cyc
        if not rst_n:
            s.init_done = 0; s.last_ref_chk = cyc
        if s.wr_active:
            b, row, col = s.wr
            s.mem[s.addr(b, row, col, s.wr_beat)] = dq
            s.wr_beat += 1
            if s.wr_beat == 4: s.wr_active = 0
        n_oe, n_out = 0, s.dq_out
        if s.rd_wait > 1:
            s.rd_wait -= 1
        elif s.rd_wait == 1:
            b, row, col = s.rd
            n_out = s.mem.get(s.addr(b, row, col, s.rd_beat)); n_oe = 1
            s.rd_beat += 1
            if s.rd_beat == 4: s.rd_wait = 0
        if cke:
            if cmd == MRS:
                if a != MODE: s.err("mode register is not BL4/CL2")
                s.init_done = 1; s.last_ref_chk = cyc
            elif cmd == ACT:
                if not s.init_done: s.err("ACT before init finished")
                if s.row_open[ba]: s.err("ACT to a bank that is already open")
                if cyc - s.last_pre[ba] < s.T_RP: s.err("tRP violated (PRE -> ACT)")
                if cyc - s.last_ref < s.T_RFC: s.err("tRFC violated (REF -> ACT)")
                s.row_open[ba] = 1; s.open_row[ba] = a; s.last_act[ba] = cyc
            elif cmd == RD:
                if not s.row_open[ba]: s.err("READ to closed bank")
                if cyc - s.last_act[ba] < s.T_RCD: s.err("tRCD violated (ACT -> READ)")
                s.rd = (ba, s.open_row[ba], a & 0xFF); s.rd_wait = s.CL; s.rd_beat = 0
            elif cmd == WR:
                if not s.row_open[ba]: s.err("WRITE to closed bank")
                if cyc - s.last_act[ba] < s.T_RCD: s.err("tRCD violated (ACT -> WRITE)")
                s.wr = (ba, s.open_row[ba], a & 0xFF)
                s.mem[s.addr(ba, s.open_row[ba], a & 0xFF, 0)] = dq
                s.wr_beat = 1; s.wr_active = 1; s.last_wr[ba] = cyc
            elif cmd == PRE:
                banks = range(4) if (a >> 10) & 1 else [ba]
                for i in banks:
                    if cyc - s.last_wr[i] < 3 + s.T_WR: s.err("tWR violated (WRITE -> PRE)")
                    s.row_open[i] = 0; s.last_pre[i] = cyc
            elif cmd == REF:
                for i in range(4):
                    if s.row_open[i]: s.err("REF with a bank still open")
                    if cyc - s.last_pre[i] < s.T_RP: s.err("tRP violated (PRE -> REF)")
                if cyc - s.last_ref < s.T_RFC: s.err("tRFC violated (REF -> REF)")
                s.last_ref = cyc; s.last_ref_chk = cyc
        if s.init_done and cyc - s.last_ref_chk > s.T_REF_MAX:
            s.err("refresh deadline missed - data would be lost"); s.last_ref_chk = cyc
        s.dq_oe, s.dq_out = n_oe, n_out


class Sim:
    """testbench side: tb code is a generator, every 'yield' = @(posedge clk); #1"""
    def __init__(s, REFI, INIT=200):
        s.ctrl = Ctrl(INIT, REFI)
        s.mdl = Sdram(REFI + 60)
        s.rst_n = 0; s.req = 0; s.wr = 0; s.addr = 0; s.wdata = 0
        s.cycle = 0; s.snaps = []
        s.count_data = 0; s.data_cycles = 0; s.ref_cnt = 0

    def dq(s):
        if s.ctrl.r["dq_oe"] and s.mdl.dq_oe:
            s.mdl.err("bus contention on DQ")
        if s.ctrl.r["dq_oe"]: return s.ctrl.r["dq_out"]
        if s.mdl.dq_oe: return s.mdl.dq_out
        return None

    def snap(s):
        r = s.ctrl.r
        s.snaps.append(dict(cyc=s.cycle, rst_n=s.rst_n, req=s.req, wr=s.wr, addr=s.addr, wdata=s.wdata,
                            ready=s.ctrl.ready, done=r["done"], rdata=r["rdata"], cmd=r["cmd"], ba=r["ba"],
                            a=r["a"], dq=s.dq(), state=r["state"], cnt=r["cnt"], ref_pending=r["ref_pending"]))

    def edge(s):
        s.snap()
        if not s.rst_n:
            s.ctrl.reset()
        r = s.ctrl.r
        dq = s.dq()
        if s.count_data and (r["dq_oe"] or s.mdl.dq_oe): s.data_cycles += 1
        if r["cmd"] == REF and s.mdl.init_done: s.ref_cnt += 1
        s.mdl.step(r["cmd"], r["ba"], r["a"], r["cke"], s.rst_n, dq)
        if s.rst_n:
            s.ctrl.step(s.req, s.wr, s.addr, s.wdata, dq)
        s.cycle += 1

    def run(s, gen):
        for _ in gen:
            s.edge()


def write_vcd(snaps, path):
    sigs = [("clk",1),("rst_n",1),("req",1),("wr",1),("addr",22),("wdata",64),("ready",1),("done",1),
            ("rdata",64),("cs_n",1),("ras_n",1),("cas_n",1),("we_n",1),("ba",2),("a",12),("dq",16),
            ("state",4),("cnt",4),("ref_pending",1)]
    ids = {n: chr(33 + i) for i, (n, _) in enumerate(sigs)}
    with open(path, "w") as f:
        f.write("$timescale 1ps $end\n$scope module tb_sdram_ctrl $end\n")
        for n, w in sigs:
            f.write(f"$var wire {w} {ids[n]} {n} $end\n")
        f.write("$upscope $end\n$enddefinitions $end\n")
        def val(n, w, v):
            if v is None: return f"b{'z'*w} {ids[n]}\n" if w > 1 else f"z{ids[n]}\n"
            return f"b{v:0{w}b} {ids[n]}\n" if w > 1 else f"{v}{ids[n]}\n"
        for sn in snaps:
            t = 5000 + 10000 * sn["cyc"]          # posedge time in ps
            f.write(f"#{t}\n"); f.write(val("clk", 1, 1))
            c = sn["cmd"]
            vals = dict(rst_n=sn["rst_n"], req=sn["req"], wr=sn["wr"], addr=sn["addr"], wdata=sn["wdata"],
                        ready=sn["ready"], done=sn["done"], rdata=sn["rdata"],
                        cs_n=(c >> 3) & 1, ras_n=(c >> 2) & 1, cas_n=(c >> 1) & 1, we_n=c & 1,
                        ba=sn["ba"], a=sn["a"], dq=sn["dq"], state=sn["state"], cnt=sn["cnt"],
                        ref_pending=sn["ref_pending"])
            for n, w in sigs:
                if n != "clk": f.write(val(n, w, vals[n]))
            f.write(f"#{t + 5000}\n"); f.write(val("clk", 1, 0))


# ======================= testbench (same phases as tb_sdram_ctrl.v) =======================
def make_tb(S, out):
    rnd = random.Random(12345)
    ref = {}
    st = dict(pass_cnt=0, fail_cnt=0, wr_sum=0, wr_n=0, rd_sum=0, rd_n=0, last_rdata=0, last_lat=0)
    P = lambda *a: out.append(" ".join(str(x) for x in a))
    rand_addr = lambda: rnd.getrandbits(22) & 0x3FFFFC
    rand_data = lambda: rnd.getrandbits(64)

    def wait_ready():
        yield
        while not S.ctrl.ready:
            yield

    def xfer(w, ad, d):
        yield from wait_ready()
        S.req, S.wr, S.addr, S.wdata = 1, w, ad, d
        t_req = S.cycle
        yield
        S.req = 0
        while not S.ctrl.r["done"]:
            yield
        st["last_lat"] = S.cycle - t_req
        st["last_rdata"] = S.ctrl.r["rdata"]
        if w:
            ref[ad >> 2] = d
            st["wr_sum"] += st["last_lat"]; st["wr_n"] += 1
        else:
            st["rd_sum"] += st["last_lat"]; st["rd_n"] += 1

    def check_read(ad):
        yield from xfer(0, ad, 0)
        if (ad >> 2) in ref:
            if st["last_rdata"] == ref[ad >> 2]: st["pass_cnt"] += 1
            else:
                st["fail_cnt"] += 1
                P(f"FAIL: addr={ad:06x} got={st['last_rdata']:016x} exp={ref[ad >> 2]:016x}")

    def write_then_read(ad, d):
        yield from xfer(1, ad, d)
        yield from check_read(ad)

    def tb():
        P("== Phase 1: reset and init ==")
        S.rst_n = 0
        for _ in range(5): yield
        S.rst_n = 1
        t0 = S.cycle
        while not S.ctrl.ready: yield
        P(f"init finished after {S.cycle - t0} cycles, model init_done={S.mdl.init_done}")
        st["pass_cnt" if S.mdl.init_done else "fail_cnt"] += 1

        P("== Phase 2: functional ==")
        for ad, d in [(0x000000, 0x1111222233334444), (0x000004, 0xAAAABBBBCCCCDDDD),
                      (0x100010, 0x0123456789ABCDEF), (0x200020, 0xFEDCBA9876543210),
                      (0x300030, 0xDEADBEEFCAFEF00D), (0x012344, 0x0000000100000002)]:
            yield from write_then_read(ad, d)
        for ad in (0x000000, 0x000004, 0x100010, 0x200020, 0x300030):
            yield from check_read(ad)

        P("== Phase 3: randomized ==")
        pool = [rand_addr() for _ in range(64)]
        for _ in range(300):
            ta = rnd.choice(pool)
            if rnd.getrandbits(1): yield from xfer(1, ta, rand_data())
            else: yield from check_read(ta)

        P("== Phase 4: stress (back to back, many refreshes) ==")
        for _ in range(1000):
            ta = rnd.choice(pool)
            if rnd.getrandbits(2) == 0: yield from xfer(1, ta, rand_data())
            else: yield from check_read(ta)
        for i in range(100):
            yield from write_then_read(((i & 3) << 20) | (((i + 7) & 0xFFF) << 8), rand_data())

        P("== Phase 5: corner cases ==")
        yield from write_then_read(0x000000, 0)
        yield from write_then_read(0x3FFFFC, M64)
        yield from write_then_read((0 << 20) | (0xFFF << 8) | 0xFC, 0x5555AAAA5555AAAA)
        yield from write_then_read((3 << 20), 0x8000400020001000)
        yield from write_then_read((1 << 20) | (0xFFF << 8), 0x0001000200040008)
        yield from xfer(1, 0x0ABCD0, 1); yield from xfer(1, 0x0ABCD0, 2)
        yield from check_read(0x0ABCD0)
        for i in range(4):
            yield from write_then_read((i << 20) | (0x123 << 8) | 0x08, 1 << (i * 16 + 5))

        P("== Phase 6: reset in the middle of operations ==")
        yield from wait_ready()
        S.req, S.wr, S.addr, S.wdata = 1, 1, 0x055550, 0x123456789ABCDEF0
        yield
        S.req = 0
        for _ in range(6): yield
        S.rst_n = 0
        ref.pop(0x055550 >> 2, None)
        for _ in range(3): yield
        S.rst_n = 1
        while not S.ctrl.ready: yield
        P(f"recovered from reset during write, model init_done={S.mdl.init_done}")
        S.req, S.wr, S.addr = 1, 0, 0
        yield
        S.req = 0
        for _ in range(5): yield
        S.rst_n = 0
        for _ in range(2): yield
        S.rst_n = 1
        while not S.ctrl.ready: yield
        yield from check_read(0x000004)
        yield from check_read(0x300030)
        yield from write_then_read(0x055550, 0xABCDABCDABCDABCD)

        P("== Phase 7: performance ==")
        yield from wait_ready()
        st.update(wr_sum=0, wr_n=0, rd_sum=0, rd_n=0)
        S.ref_cnt = 0; S.data_cycles = 0; S.count_data = 1
        start = S.cycle
        for i in range(200): yield from xfer(1, (5 << 8) | ((i & 63) << 2), rand_data())
        for i in range(200): yield from check_read((5 << 8) | ((i & 63) << 2))
        end = S.cycle
        S.count_data = 0
        tot = end - start
        st.update(total=tot, start=start, end=end)
        P("---------------- RESULTS ----------------")
        P(f"total transfers      : {st['wr_n']} writes, {st['rd_n']} reads")
        P(f"total cycles         : {tot}")
        P(f"avg write latency    : {st['wr_sum'] // st['wr_n']} cycles")
        P(f"avg read  latency    : {st['rd_sum'] // st['rd_n']} cycles")
        P(f"data bus busy cycles : {S.data_cycles}")
        P(f"bus utilization      : {S.data_cycles * 100 / tot:.1f} %")
        P(f"refreshes issued     : {S.ref_cnt}")
        P(f"throughput           : {(st['wr_n'] + st['rd_n']) * 4 * 1000 // tot} words/1000 cycles")
        P(f"checks passed        : {st['pass_cnt']}")
        P(f"checks failed        : {st['fail_cnt']}")
        P(f"model protocol errors: {S.mdl.errors}")
        for l in S.mdl.log: P(l)
        P("*** TEST PASSED ***" if st["fail_cnt"] == 0 and S.mdl.errors == 0 else "*** TEST FAILED ***")
    return tb(), st


if __name__ == "__main__":
    refi = int(sys.argv[1]) if len(sys.argv) > 1 else 1500
    tag = "" if refi == 1500 else "_stress"
    S = Sim(refi)
    out = [f"(Python cycle model, REF_INTERVAL={refi})"]
    gen, st = make_tb(S, out)
    S.run(gen)
    print("\n".join(out))
    open(f"../results/sim_log{tag}.txt", "w").write("\n".join(out) + "\n")
    if not tag:
        write_vcd(S.snaps, "../results/waves.vcd")
        json.dump(dict(snaps=S.snaps[:6000], perf={k: v for k, v in st.items()},
                       data_cycles=S.data_cycles, ref_cnt=S.ref_cnt), open("../results/snaps.json", "w"))
