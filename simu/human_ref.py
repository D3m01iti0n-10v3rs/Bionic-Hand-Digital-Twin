"""human_ref.py - so sanh tay robot (MuJoCo) voi tham chieu tay nguoi (OpenSim, model AAH).

Chay (trong thu muc simu, da bat osim_env):
    python human_ref.py

Ra: human_grasp.mot, compare.csv, compare.png, ctrl_table.csv + bao cao in tren man hinh.
  - compare.csv   : goc toi da, %ROM (robot/nguoi), RMSE truoc va sau bu
  - ctrl_table.csv: bang lenh servo (do) theo r de robot bam tay nguoi (da bu vung chet)
  - Dung bang: ctrl = np.interp(r, bang['r'], bang[ngon])

GIA DINH (sua o HUMAN_MAX): quy dao nguoi tuyen tinh theo r (0 = mo, 1 = nam), dich la goc nam
dien hinh, cat theo gioi han khop cua model OpenSim. La quy dao tu dung, khong phai du lieu do that.
Dau goc ngon cai chua kiem chung.
"""
import os, math
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import opensim as osim
import mujoco
from grasp_test import Sim, OPEN, CLOSED, DIGITS

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "ms_arm_and_hand-main", "AAH Model", "RightArmAndHand_NoMuscles.osim")

FINGER_N = {"index": 2, "middle": 3, "ring": 4, "pinky": 5}
HUMAN_MAX = {}
for dg, n in FINGER_N.items():
    HUMAN_MAX[(dg, "mcp")] = (f"ra_mcp{n}_e_f", 90.0)
    HUMAN_MAX[(dg, "pip")] = (f"ra_pip{n}_e_f", 100.0)
    HUMAN_MAX[(dg, "dip")] = (f"ra_dip{n}_e_f", 70.0)
HUMAN_MAX[("thumb", "cmc")] = ("ra_cmc1_f_e", 40.0)
HUMAN_MAX[("thumb", "pip")] = ("ra_mcp1_e_f", 35.0)
HUMAN_MAX[("thumb", "ip")] = ("ra_ip1_e_f", 60.0)
KEYS = list(HUMAN_MAX)

# ---------------- 1. Nguoi ----------------
model = osim.Model(MODEL)
model.initSystem()
cs = model.getCoordinateSet()


def coord_range_deg(name):
    c = cs.get(name)
    lo, hi = c.getRangeMin(), c.getRangeMax()
    if c.getMotionType() == 1:
        lo, hi = math.degrees(lo), math.degrees(hi)
    return lo, hi


r_grid = np.linspace(0.0, 1.0, 101)
human = {}
for k, (coord, qmax) in HUMAN_MAX.items():
    lo, hi = coord_range_deg(coord)
    human[k] = np.clip(r_grid * qmax, lo, hi)

T_HUMAN = 1.5
cols = [HUMAN_MAX[k][0] for k in KEYS]
with open(os.path.join(HERE, "human_grasp.mot"), "w") as f:
    f.write("human_grasp\nversion=1\n")
    f.write(f"nRows={len(r_grid)}\nnColumns={len(cols) + 1}\ninDegrees=yes\nendheader\n")
    f.write("time\t" + "\t".join(cols) + "\n")
    for i, r in enumerate(r_grid):
        f.write(f"{r * T_HUMAN:.4f}\t" + "\t".join(f"{human[k][i]:.4f}" for k in KEYS) + "\n")

# ---------------- 2. Robot ----------------
sim = Sim("sphere", (50,))
m, d = sim.m, sim.d
dt = m.opt.timestep
adr_obj = m.jnt_qposadr[m.joint("obj_free").id]
adr = {k: m.jnt_qposadr[m.joint(f"{k[0]}_{k[1]}").id] for k in KEYS}


def settle(max_t=6.0):
    """Doi cho den khi tay dung yen (van toc ~ 0) de lay tu the goc cho dung."""
    for _ in range(int(max_t / 0.1)):
        sim.step(int(0.1 / dt))
        if np.max(np.abs(d.qvel)) < 1e-3:
            break


def run_ramp(ctrl_of_s, T=8.0, hold=2.0):
    """Dong tay tu mo den nam khong co vat the. ctrl_of_s(ngon, s) -> lenh servo (do), s: 0..1."""
    sim.reset()
    m.opt.gravity[:] = 0
    d.qpos[adr_obj:adr_obj + 3] = [5.0, 5.0, 5.0]      # dua vat the ra xa
    mujoco.mj_forward(m, d)
    settle()
    q0 = {k: math.degrees(d.qpos[adr[k]]) for k in KEYS}
    print("Tu the mo (do) ngon giua mcp/pip/dip:", *[round(q0[("middle", j)], 1) for j in ("mcp", "pip", "dip")])
    n = int(T / dt)
    s_log, q_log = [], {k: [] for k in KEYS}
    for i in range(n + 1):
        s = i / n
        for dg in DIGITS:
            d.ctrl[sim.act[dg]] = ctrl_of_s(dg, s)
        sim.step(1)
        if i % 10 == 0:
            s_log.append(s)
            for k in KEYS:
                q_log[k].append(math.degrees(d.qpos[adr[k]]) - q0[k])
    sim.step(int(hold / dt))                      # giu o vi tri nam het, doi servo on dinh
    settle()
    q_final = {k: math.degrees(d.qpos[adr[k]]) - q0[k] for k in KEYS}
    return np.array(s_log), {k: np.array(v) for k, v in q_log.items()}, q_final


# --- lan 1: lenh tuyen tinh (nhu hien tai) ---
s_log, q_raw, qf_raw = run_ramp(lambda dg, s: OPEN[dg] + s * (CLOSED[dg] - OPEN[dg]))

# chan doan: khop nao cham gioi han, bo phan nao dang va cham nhau (luc nam het)
print("\n== CHAN DOAN (luc nam het) ==")
at_limit = []
names = [f"{k[0]}_{k[1]}" for k in KEYS] + [f"{dg}_servo" for dg in DIGITS]
for nm in names:
    j = m.joint(nm)
    q = d.qpos[m.jnt_qposadr[j.id]]
    lo, hi = m.jnt_range[j.id]
    if m.jnt_limited[j.id] and (q - lo < math.radians(1.0) or hi - q < math.radians(1.0)):
        at_limit.append(f"{nm} ({math.degrees(q):.1f} do, gioi han {math.degrees(lo):.0f}..{math.degrees(hi):.0f})")
print("Khop cham gioi han:", "; ".join(at_limit) if at_limit else "khong co")
pairs = {}
for i in range(d.ncon):
    c = d.contact[i]
    if sim.obj_geom in (c.geom1, c.geom2):
        continue
    a = m.body(m.geom_bodyid[c.geom1]).name
    b = m.body(m.geom_bodyid[c.geom2]).name
    pairs[(a, b)] = pairs.get((a, b), 0) + 1
print("Cac cap bo phan va cham:", ", ".join(f"{a}<->{b}" for a, b in pairs) if pairs else "khong co")

sign = {k: (1.0 if q_raw[k][-1] >= 0 else -1.0) for k in KEYS}
raw = {k: np.interp(r_grid, s_log, sign[k] * q_raw[k]) for k in KEYS}

# ---------------- 3. Bu vung chet: tim lenh servo de tong goc gap robot bam nguoi ----------------
ctrl_tab = {}
for dg in DIGITS:
    ks = [k for k in KEYS if k[0] == dg]
    R = np.maximum.accumulate(sum(sign[k] * q_raw[k] for k in ks)) + 1e-6 * s_log   # tong gap robot theo s
    H = sum(human[k] for k in ks)                                                    # tong gap nguoi theo r
    target = np.minimum(H, R[-1])
    s_of_r = np.interp(target, R, s_log)
    ctrl_tab[dg] = OPEN[dg] + s_of_r * (CLOSED[dg] - OPEN[dg])
pd.DataFrame({"r": r_grid, **{dg: ctrl_tab[dg] for dg in DIGITS}}).round(2).to_csv(
    os.path.join(HERE, "ctrl_table.csv"), index=False)

# --- lan 2: chay lai voi lenh da bu ---
_, q_cmp, qf_cmp = run_ramp(lambda dg, s: float(np.interp(s, r_grid, ctrl_tab[dg])))
comp = {k: np.interp(r_grid, s_log, sign[k] * q_cmp[k]) for k in KEYS}

# ---------------- 4. Bao cao ----------------
rows = []
for k in KEYS:
    rows.append(dict(
        digit=k[0], joint=k[1], human_max=human[k][-1], robot_max=sign[k] * qf_raw[k],
        pct_ROM=100.0 * sign[k] * qf_raw[k] / human[k][-1],
        rmse_truoc=float(np.sqrt(np.mean((raw[k] - human[k]) ** 2))),
        rmse_sau_bu=float(np.sqrt(np.mean((comp[k] - human[k]) ** 2)))))
tab = pd.DataFrame(rows).round(1)
print("\n== BANG SO SANH (do) ==")
print(tab.to_string(index=False))
tab.to_csv(os.path.join(HERE, "compare.csv"), index=False)

print("\n== TONG GOC GAP MOI NGON (MCP+PIP+DIP) ==")
for dg in DIGITS:
    ks = [k for k in KEYS if k[0] == dg]
    ht = sum(human[k] for k in ks)
    rt = sum(raw[k] for k in ks)
    ct = sum(comp[k] for k in ks)
    print(f"{dg:<7} nguoi {ht[-1]:6.1f} | robot {rt[-1]:6.1f} ({100*rt[-1]/ht[-1]:.0f}%) | "
          f"sai lech TB truoc bu {np.mean(np.abs(rt-ht)):5.1f} -> sau bu {np.mean(np.abs(ct-ht)):5.1f}")

fig, axs = plt.subplots(2, 5, figsize=(20, 7), sharey="row")
colors = {"mcp": "C0", "pip": "C1", "dip": "C2", "cmc": "C0", "ip": "C2"}
for col, dg in enumerate(DIGITS):
    ax = axs[0, col]
    for k in KEYS:
        if k[0] != dg:
            continue
        ax.plot(r_grid, human[k], "-", color=colors[k[1]], label=f"nguoi {k[1]}")
        ax.plot(r_grid, raw[k], "--", color=colors[k[1]], label=f"robot {k[1]}")
    ax.set_title(f"{dg} - tung khop")
    ax.grid(alpha=0.3)
    ax = axs[1, col]
    ks = [k for k in KEYS if k[0] == dg]
    ax.plot(r_grid, sum(human[k] for k in ks), "k-", label="nguoi")
    ax.plot(r_grid, sum(raw[k] for k in ks), "--", color="gray", label="robot (chua bu)")
    ax.plot(r_grid, sum(comp[k] for k in ks), ":", color="red", lw=2, label="robot (da bu)")
    ax.set_title(f"{dg} - tong goc gap")
    ax.set_xlabel("r")
    ax.grid(alpha=0.3)
axs[0, 0].set_ylabel("goc gap (do)")
axs[1, 0].set_ylabel("tong goc gap (do)")
axs[0, 0].legend(fontsize=7)
axs[1, 0].legend(fontsize=7)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "compare.png"), dpi=150)
print("\nDa luu: human_grasp.mot, compare.csv, compare.png, ctrl_table.csv")