"""diagnose.py v2 - tim cai gi chan ngon robot trong mo phong (ngon giua).

Chay:   python diagnose.py
Noi rong khoang quay servo (bo gioi han XML), quay servo cham tu cuoi mo den xa hon cuoi nam.
Tai moi diem ghi lai: 5 khop, tong goc gap, KHOP NAO DANG BI GIOI HAN, CAP BO PHAN NAO DANG VA CHAM.
"""
import math, numpy as np, mujoco
from grasp_test import Sim, OPEN, DIGITS

FINGER = "middle"
sim = Sim("sphere", (50,))
m, d = sim.m, sim.d
dt = m.opt.timestep
deg = math.degrees
NAMES = ["mcp", "pip", "dip", "tj", "lj"]
jid = {n: m.joint(f"{FINGER}_{n}").id for n in NAMES + ["servo"]}
qa = {n: m.jnt_qposadr[jid[n]] for n in jid}
rng = {n: (deg(m.jnt_range[jid[n]][0]), deg(m.jnt_range[jid[n]][1])) for n in jid}
jname = {m.joint(i).id: m.joint(i).name for i in range(m.njnt)}
act = sim.act[FINGER]
gain, b0, b1 = m.actuator_gainprm[act][0], m.actuator_biasprm[act][0], m.actuator_biasprm[act][1]


def ctrl_for(theta_deg):
    return (-b1 * math.radians(theta_deg) - b0) / gain


def settle(max_t=6.0):
    for _ in range(int(max_t / 0.1)):
        sim.step(int(0.1 / dt))
        if np.max(np.abs(d.qvel)) < 1e-3:
            break


def active_limits():
    out = []
    for k in range(d.nefc):
        if d.efc_type[k] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT and abs(d.efc_force[k]) > 1e-6:
            out.append(jname[int(d.efc_id[k])])
    return sorted(set(out))


def active_contacts():
    out = {}
    f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        if sim.obj_geom in (c.geom1, c.geom2):
            continue
        mujoco.mj_contactForce(m, d, i, f6)
        a, b = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
        key = " <-> ".join(sorted((a, b)))
        out[key] = max(out.get(key, 0.0), float(f6[0]))
    return out


xml_lo, xml_hi = rng["servo"]
print(f"Khoang quay servo trong XML: {xml_lo:.1f} .. {xml_hi:.1f} do")

m.jnt_range[jid["servo"]] = [math.radians(-100), math.radians(100)]
m.actuator_ctrlrange[act] = [-400, 400]
sim.reset()
m.opt.gravity[:] = 0
a = m.jnt_qposadr[m.joint("obj_free").id]
d.qpos[a:a + 3] = [5.0, 5.0, 5.0]
mujoco.mj_forward(m, d)
for dg in DIGITS:
    d.ctrl[sim.act[dg]] = OPEN[dg]
d.ctrl[act] = ctrl_for(xml_lo)
settle()
q0 = {n: deg(d.qpos[qa[n]]) for n in NAMES}
print("Tu the mo that (do):", {n: round(v, 1) for n, v in q0.items()})

th_start, th_end, rate = xml_lo, 95.0, 8.0
n_steps = int((th_end - th_start) / rate / dt)
rows = []
for i in range(n_steps + 1):
    th = th_start + (th_end - th_start) * i / n_steps
    d.ctrl[act] = ctrl_for(th)
    sim.step(1)
    if i % 25 == 0:
        q = {n: deg(d.qpos[qa[n]]) for n in jid}
        fl = [q0[n] - q[n] for n in ("mcp", "pip", "dip")]
        rows.append(dict(th=th, servo=q["servo"], q=q, flex=fl, tot=sum(fl),
                         lim=active_limits(), con=active_contacts()))

print("\n== BANG: lenh servo | servo thuc | gap mcp pip dip tong | KHOP BI GIOI HAN | VA CHAM ==")
for target in (-60, -45, -30, -22, -15, 0, 20, 40, 47, 55, 60, 63, 66, 70, 80, 95):
    r = min(rows, key=lambda x: abs(x["th"] - target))
    con = ", ".join(f"{k} ({v:.2f}N)" for k, v in r["con"].items()) or "-"
    print(f"{r['th']:6.1f} | {r['servo']:6.1f} | {r['flex'][0]:5.1f} {r['flex'][1]:5.1f} {r['flex'][2]:5.1f} {r['tot']:6.1f} | "
          f"{', '.join(r['lim']) or '-'} | {con}")

print("\n== PHAN TICH ==")
# 1) khi nao ngon bat dau gap that su
start = next((r for r in rows if r["tot"] > 5.0), None)
if start:
    print(f"Ngon bat dau gap that su (tong > 5 do) khi lenh servo = {start['th']:.0f} do "
          f"(cach dau mo {start['th'] - xml_lo:.0f} do).")
    r0 = rows[0]
    print("  Luc con o dau mo - khop bi gioi han:", r0["lim"] or "khong", "| va cham:", list(r0["con"]) or "khong")
# 2) khi nao ngon dung gap
mx = max(r["tot"] for r in rows)
stop = next(r for r in rows if r["tot"] >= mx - 1.0)
print(f"Tong gap toi da {mx:.0f} do, dat tai lenh servo ~{stop['th']:.0f} do (servo thuc {stop['servo']:.0f}).")
print("  Tai diem dung - khop bi gioi han:", stop["lim"] or "khong khop nao",
      "| va cham:", {k: round(v, 2) for k, v in stop["con"].items()} or "khong")
at_xml = min(rows, key=lambda x: abs(x["th"] - xml_hi))
print(f"Tai gioi han servo trong XML ({xml_hi:.0f} do): tong gap = {at_xml['tot']:.0f} do.")
print(f"=> Neu bo gioi han servo trong XML, ngon gap them {mx - at_xml['tot']:.0f} do (con {100*(mx-at_xml['tot'])/max(mx,1):.0f}% tong toi da).")
print("\nGui nguyen toan bo chu in ra o tren cho minh.")