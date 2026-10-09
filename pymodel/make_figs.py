import json, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import sdram_pymodel as m

D = json.load(open("../results/snaps.json")); sn = D["snaps"]

def wave(c0, c1, title, fn):
    w = sn[c0:c1]; n = len(w)
    rows = ["clk", "state", "command", "BA", "A", "DQ", "done"]
    fig, ax = plt.subplots(figsize=(13, 5.2)); ax.set_xlim(0, n); ax.set_ylim(-len(rows), 0.3)
    for i, name in enumerate(rows):
        y = -i - 0.5; ax.text(-0.15, y, name, ha="right", va="center", fontsize=10, weight="bold")
        for t, s in enumerate(w):
            if name == "clk":
                ax.plot([t, t, t + .5, t + .5, t + 1], [y - .3, y + .3, y + .3, y - .3, y - .3], "k", lw=1)
                continue
            if name == "done":
                v = s["done"]; ax.plot([t, t + 1], [y - .3 + .6 * v] * 2, "g", lw=1.5); continue
            txt = {"state": m.STATE_NAME[s["state"]], "command": m.CMD_NAME.get(s["cmd"], "?"),
                   "BA": f'{s["ba"]}', "A": f'{s["a"]:03x}',
                   "DQ": "z" if s["dq"] is None else f'{s["dq"]:04x}'}[name]
            hot = (name == "command" and txt != "NOP") or (name == "DQ" and txt != "z")
            ax.add_patch(plt.Rectangle((t + .03, y - .3), .94, .6, fc="#ffd9a0" if hot else "#e8eef7", ec="gray", lw=.6))
            ax.text(t + .5, y, txt, ha="center", va="center", fontsize=7 if name != "state" else 6.5)
    ax.set_xticks(range(0, n + 1, 2)); ax.set_yticks([]); ax.set_xlabel("clock cycle (relative)")
    ax.set_title(title); ax.grid(axis="x", alpha=.2)
    plt.tight_layout(); plt.savefig(fn, dpi=130); plt.close()

def find(cmd, after=0):
    return next(i for i, s in enumerate(sn) if i > after and s["cmd"] == cmd)

# init sequence
i0 = next(i for i, s in enumerate(sn) if s["state"] == m.S_INIT_PRE) - 2
wave(i0, i0 + 38, "SDRAM initialisation: PRECHARGE ALL, 2x REFRESH, MODE REGISTER SET (then IDLE)", "../docs/wave_init.png")
w = find(m.WR, 300); a = w - 4
wave(a, a + 22, "Write burst (ACT -> WRITE -> 4 words -> tWR -> PRECHARGE), 100 MHz", "../docs/wave_write.png")
r = find(m.RD, 300); a = r - 4
wave(a, a + 26, "Read burst, CAS latency 2 (ACT -> READ -> 4 words -> PRECHARGE)", "../docs/wave_read.png")
rf = next(i for i, s in enumerate(sn) if i > 400 and s["cmd"] == m.REF)
wave(rf - 4, rf + 14, "Auto refresh (REF -> tRFC wait -> IDLE)", "../docs/wave_refresh.png")

# ---------- block diagram ----------
fig, ax = plt.subplots(figsize=(12, 5)); ax.set_xlim(0, 12); ax.set_ylim(0, 5); ax.axis("off")
def box(x, y, w, h, t, c="#dce8fa"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.05", fc=c, ec="k")); ax.text(x + w / 2, y + h / 2, t, ha="center", va="center", fontsize=9)
def arr(x1, y1, x2, y2, t=""):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="<->" if "dq" in t.lower() else "->", mutation_scale=12, lw=1.3))
    ax.text((x1 + x2) / 2, (y1 + y2) / 2 + .12, t, ha="center", fontsize=8)
box(0.2, 1.5, 2, 2, "HOST /\nTESTBENCH\n(req, wr, addr,\nwdata, rdata)", "#e6f5e6")
box(3.2, 0.3, 5.4, 4.4, "", "#fff6dc"); ax.text(5.9, 4.4, "sdram_ctrl", ha="center", weight="bold")
box(3.5, 3.0, 2.3, 1.0, "Address mapper\nbank=[21:20] row=[19:8]\ncol=[7:0]")
box(6.0, 3.0, 2.3, 1.0, "Request latch")
box(3.5, 1.7, 2.3, 1.1, "Main FSM\n(init / idle / act / rd /\nwr / pre / refresh / wait)")
box(6.0, 1.7, 2.3, 1.1, "Command encoder\n+ wait counter\n(tRP tRCD tRFC tWR)")
box(3.5, 0.5, 2.3, 0.9, "Refresh timer\n(every 1500 cycles)")
box(6.0, 0.5, 2.3, 0.9, "Data path\n64-bit burst <-> 16-bit DQ")
box(9.4, 1.5, 2.4, 2, "SDR SDRAM\n4 banks x 4096 rows\nx 256 cols x 16b\n(behavioural model)", "#fde3e3")
arr(2.3, 2.5, 3.2, 2.5, "req/addr/data"); arr(3.2, 1.9, 2.3, 1.9, "ready/done/rdata")
arr(8.6, 2.9, 9.4, 2.9, "CKE CS RAS CAS WE BA A"); arr(8.6, 2.1, 9.4, 2.1, "DQ[15:0]")
ax.set_title("Block diagram - SDRAM controller"); plt.savefig("../docs/block_diagram.png", dpi=130, bbox_inches="tight"); plt.close()

# ---------- FSM diagram ----------
fig, ax = plt.subplots(figsize=(12, 6.5)); ax.set_xlim(0, 12); ax.set_ylim(0, 6.5); ax.axis("off")
pos = {"INIT_WAIT": (1, 5.6), "INIT_PRE": (3, 5.6), "INIT_REF x2": (5, 5.6), "INIT_MRS": (7, 5.6),
       "IDLE": (6, 3.6), "REFRESH": (10, 3.6), "ACTIVATE": (3, 3.6), "READ": (1.5, 1.5), "WRITE": (4.5, 1.5),
       "PRECHARGE": (8, 1.5), "WAIT": (10.5, 1.5)}
for k, (x, y) in pos.items():
    ax.add_patch(plt.Circle((x, y), .6, fc="#dce8fa" if k != "IDLE" else "#fff0b3", ec="k")); ax.text(x, y, k, ha="center", va="center", fontsize=8)
def tr(a, b, t="", rad=0):
    (x1, y1), (x2, y2) = pos[a], pos[b]
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="->", mutation_scale=12, shrinkA=22, shrinkB=22, connectionstyle=f"arc3,rad={rad}"))
    ax.text((x1 + x2) / 2, (y1 + y2) / 2 + .15, t, fontsize=7, ha="center", color="darkred")
tr("INIT_WAIT", "INIT_PRE", "100us"); tr("INIT_PRE", "INIT_REF x2"); tr("INIT_REF x2", "INIT_MRS"); tr("INIT_MRS", "IDLE", "tMRD")
tr("IDLE", "ACTIVATE", "req"); tr("IDLE", "REFRESH", "ref_pending"); tr("REFRESH", "IDLE", "tRFC", .3)
tr("ACTIVATE", "READ", "rd, tRCD"); tr("ACTIVATE", "WRITE", "wr, tRCD"); tr("READ", "PRECHARGE", "4 words in", -.25)
tr("WRITE", "PRECHARGE", "4 words out + tWR"); tr("PRECHARGE", "WAIT", "tRP"); tr("WAIT", "IDLE", "done", .4)
ax.set_title("Controller FSM (WAIT is a shared counter state that returns to the next state)")
plt.savefig("../docs/fsm_diagram.png", dpi=130, bbox_inches="tight"); plt.close()
print("figs done", i0, w, r, rf)
