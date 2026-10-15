"""static_check.py - kiem tra "vung chet" 30 do dau cua servo: do cua co cau hay do dong luc hoc.

Chay:   python static_check.py
Giu servo ngon giua o tung goc co dinh, doi tay dung yen (tinh), roi do goc gap.
Neu vung chet van con khi tay dung yen => do hinh hoc co cau (khong phai do quay nhanh/tre).
Cot ball_x = do dich chuyen (mm) cua dau thanh truyen theo goc servo, tinh tu toa do trong XML.
"""
import math, numpy as np, mujoco
from grasp_test import Sim, OPEN, DIGITS

FINGER = "middle"
sim = Sim("sphere", (50,))
m, d = sim.m, sim.d
dt = m.opt.timestep
deg = math.degrees
NAMES = ["mcp", "pip", "dip"]
jid = {n: m.joint(f"{FINGER}_{n}").id for n in NAMES + ["servo"]}
qa = {n: m.jnt_qposadr[jid[n]] for n in jid}
act = sim.act[FINGER]
gain, b0, b1 = m.actuator_gainprm[act][0], m.actuator_biasprm[act][0], m.actuator_biasprm[act][1]
xml_lo = deg(m.jnt_range[jid["servo"]][0])


def ctrl_for(theta_deg):
    return (-b1 * math.radians(theta_deg) - b0) / gain


def settle(max_t=5.0):
    for _ in range(int(max_t / 0.1)):
        sim.step(int(0.1 / dt))
        if np.max(np.abs(d.qvel)) < 1e-3:
            break


# vi tri dau thanh truyen (tam khop cau) khi servo quay theta (Rodrigues quanh truc servo)
H = m.jnt_pos[jid["servo"]]
axis = m.jnt_axis[jid["servo"]]
B0 = m.jnt_pos[m.joint(f"{FINGER}_rod_ball").id]


def ball(theta_deg):
    v, t = B0 - H, math.radians(theta_deg)
    return H + v * math.cos(t) + np.cross(axis, v) * math.sin(t) + axis * np.dot(axis, v) * (1 - math.cos(t))


m.jnt_range[jid["servo"]] = [math.radians(-100), math.radians(100)]
m.actuator_ctrlrange[act] = [-400, 400]
sim.reset()
m.opt.gravity[:] = 0
a = m.jnt_qposadr[m.joint("obj_free").id]
d.qpos[a:a + 3] = [5.0, 5.0, 5.0]
mujoco.mj_forward(m, d)
for dg in DIGITS:
    d.ctrl[sim.act[dg]] = OPEN[dg]

# diem chet cua tay quay: goc servo ma x cua dau thanh truyen cuc tri
ths = np.arange(-100, 100.1, 0.5)
xs = np.array([ball(t)[0] for t in ths])
print(f"Dau thanh truyen (truc x) cuc tri o goc servo ~ {ths[np.argmin(xs)]:.1f} do (cuc tieu) va {ths[np.argmax(xs)]:.1f} do (cuc dai). "
      f"Gioi han mo cua servo trong XML: {xml_lo:.1f} do.")

d.ctrl[act] = ctrl_for(xml_lo)
settle()
q0 = {n: deg(d.qpos[qa[n]]) for n in NAMES}
x0 = ball(xml_lo)[0]
print("\nlenh_servo | servo_thuc | ball_x (mm) |  gap mcp  pip  dip  tong")
for th in list(range(-60, -10, 5)) + [-10, 0, 10, 20, 30, 40, 47]:
    d.ctrl[act] = ctrl_for(th)
    settle()
    q = {n: deg(d.qpos[qa[n]]) for n in jid}
    fl = [q0[n] - q[n] for n in NAMES]
    print(f"{th:8.1f} | {q['servo']:9.1f} | {1000*(ball(q['servo'])[0]-x0):9.2f} | {fl[0]:9.1f} {fl[1]:5.1f} {fl[2]:5.1f} {sum(fl):6.1f}")
print("\nGui nguyen toan bo chu in ra o tren cho minh.")