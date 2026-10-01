"""Grasp simulation (#7).  ctrl = servo angle in degrees (0-180), see HANDOFF.
  python grasp_test.py --shape sphere --size 50            one trial (mm), prints metrics
  python grasp_test.py --shape cylinder --size 40 80 --view   watch it in the MuJoCo viewer
  python grasp_test.py --sweep                             all shapes x sizes -> grasp_results.csv
Size (mm): sphere [diameter]; cylinder [diameter height]; box [lx ly lz]."""
import argparse, csv, sys, time
import numpy as np, mujoco as m
from make_scene import make_scene

SERVOS = ["thumb_servo", "index_servo", "middle_servo", "ring_servo", "pinky_servo"]
OPEN   = np.array([0, 180, 180, 180, 180.])     # fully extended
CLOSED = np.array([180, 60, 60, 60, 60.])       # fully retracted (limit of the linkage)
RATE   = 150.0     # deg/s servo slew (SG90 ~ 0.1 s / 60 deg = 600; 150 is conservative)
STALL_V = 10.0     # deg/s: below this speed the servo counts as stopped
LAG    = 8.0      # deg: servo "blocked" when it lags the command by more than this
                   #   -> grip torque ~ 0.4*LAG*pi/180 N.m (stall is 0.17)
STRATEGY = {       # which servos close, and delay (s) before they start
    "power": {0: 0.3, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0},
    "pinch": {0: 0.0, 1: 0.0, 2: 0.0},               # thumb + index + middle
}
DEFAULT_STRATEGY = {"sphere": "power", "cylinder": "power", "box": "power"}

class Hand:
    def __init__(s, shape, size_mm, mass):
        size = [x / 1000 for x in size_mm]
        ey = size[0] / 2 if shape in ("sphere", "cylinder") else size[1] / 2   # half extent along Y
        pos = (0.108, 0.073 - ey, 0.110)        # just in front of the palm face, under the fingers
        s.M = m.MjModel.from_xml_path(make_scene(shape, size, mass, pos=pos)); s.D = m.MjData(s.M)
        s.act = [m.mj_name2id(s.M, m.mjtObj.mjOBJ_ACTUATOR, n) for n in SERVOS]
        s.jadr = [s.M.jnt_qposadr[s.M.actuator_trnid[a, 0]] for a in s.act]
        s.obj = m.mj_name2id(s.M, m.mjtObj.mjOBJ_BODY, "obj")
        s.og = m.mj_name2id(s.M, m.mjtObj.mjOBJ_GEOM, "obj_geom")
        s.qa = s.M.jnt_qposadr[m.mj_name2id(s.M, m.mjtObj.mjOBJ_JOINT, "obj_free")]
        # open the hand first with the object parked far away, then place it in the pocket
        s.M.opt.gravity[:] = 0
        s.D.qpos[s.qa:s.qa + 3] = (0.5, 0.5, 0.11); s.D.ctrl[:] = OPEN
        for _ in range(600): m.mj_step(s.M, s.D)
        s.D.qpos[s.qa:s.qa + 7] = (*pos, 1, 0, 0, 0); s.D.qvel[:] = 0
        m.mj_forward(s.M, s.D)
    def angle(s):                     # real servo angle (deg) read back from the joint
        g = s.M.actuator_gainprm[s.act, 0]; b = s.M.actuator_biasprm[s.act]
        q = s.D.qpos[s.jadr]
        return (-b[:, 1] * q - b[:, 0]) / g
    def objpos(s): return s.D.qpos[s.qa:s.qa + 3].copy()
    def contacts(s):                  # list of (body_name, normal_force) touching the object
        out = []; f = np.zeros(6)
        for i in range(s.D.ncon):
            c = s.D.contact[i]
            if s.og in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == s.og else c.geom1
                m.mj_contactForce(s.M, s.D, i, f)
                out.append((m.mj_id2name(s.M, m.mjtObj.mjOBJ_BODY, s.M.geom_bodyid[other]), f[0]))
        return out
    def step(s, n=1, viewer=None):
        for _ in range(n):
            m.mj_step(s.M, s.D)
            if viewer is not None:
                viewer.sync(); time.sleep(s.M.opt.timestep)

def trial(shape, size_mm, mass=0.1, strategy=None, lag=LAG, view=False):
    strategy = strategy or DEFAULT_STRATEGY[shape]
    H = Hand(shape, size_mm, mass); dt = H.M.opt.timestep
    plan = STRATEGY[strategy]
    viewer = None
    if view:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(H.M, H.D)
        time.sleep(1.0)
    H.M.opt.gravity[:] = [0, 9.81, 0]   # closing with the palm facing up (object rests on the palm)
    cmd = OPEN.copy(); H.D.ctrl[:] = cmd
    N = lambda t: int(t / dt)
    H.step(N(0.4), viewer)                                  # object settles on the open palm
    p_start = H.objpos(); maxc = 0; t = 0.0
    blocked = {i: False for i in plan}
    stop_at = {}
    while t < 3.0:                                          # ---- closing, closed loop
        for i, delay in plan.items():
            if t < delay or blocked[i]: continue
            target = CLOSED[i]; step = RATE * dt * 10
            if abs(target - cmd[i]) > step: cmd[i] += np.sign(target - cmd[i]) * step
            else: cmd[i] = target
            if abs(cmd[i] - H.angle()[i]) > lag and abs(H.D.qvel[H.M.jnt_dofadr[H.M.actuator_trnid[H.act[i], 0]]]) < np.radians(STALL_V): blocked[i] = True; stop_at[i] = cmd[i]
        H.D.ctrl[:] = cmd
        H.step(10, viewer); t += 10 * dt
        if all(blocked[i] or cmd[i] == CLOSED[i] for i in plan) and t > 1.0: break
    H.step(N(0.5), viewer)
    grasp_pos = H.objpos(); nb = [b for b, f in H.contacts() if f > 0.01]
    # ---- hold test: flip gravity so the palm faces down, then side loads
    drift = 0.0; held = True; minc = 99; fsum = []
    for g in ([0, -9.81, 0], [0, -9.81, 0], [9.81, 0, 0], [0, 0, 9.81], [0, 0, -9.81]):
        H.M.opt.gravity[:] = g
        for _ in range(10):
            H.step(N(0.15), viewer)
            d = np.linalg.norm(H.objpos() - grasp_pos); drift = max(drift, d)
            cs = [f for b, f in H.contacts() if f > 0.01]; minc = min(minc, len(cs)); fsum.append(sum(cs))
            if d > 0.04: held = False; break
        if not held: break
        # (first entry repeated on purpose: palm-down is the main test, ~3 s)
    if viewer is not None: viewer.close()
    return dict(shape=shape, size_mm="x".join(str(int(x)) for x in size_mm), mass_g=int(mass * 1000),
                strategy=strategy, success=int(held), drift_mm=round(drift * 1000, 1),
                n_contact_bodies=len(set(nb)), min_contacts=minc, grip_N=round(float(np.mean(fsum)), 2),
                servo_deg=" ".join("%d" % round(c) for c in cmd), contacts=",".join(sorted(set(nb))))

SWEEP = [("sphere", [d]) for d in (40, 50, 60, 70)] + \
        [("cylinder", [d, 80]) for d in (30, 40, 50, 60)] + \
        [("box", s) for s in ([40, 40, 40], [50, 50, 50], [40, 30, 80], [60, 40, 60])]

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--shape", default="sphere"); ap.add_argument("--size", type=float, nargs="+", default=[50])
    ap.add_argument("--mass", type=float, default=0.1); ap.add_argument("--strategy")
    ap.add_argument("--view", action="store_true"); ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()
    if a.sweep:
        rows = []
        for shape, size in SWEEP:
            for mass in (0.05, 0.2):
                r = trial(shape, size, mass); rows.append(r); print(r["shape"], r["size_mm"], r["mass_g"], "OK" if r["success"] else "FAIL", r["drift_mm"], r["servo_deg"], flush=True)
        with open("grasp_results.csv", "w", newline="") as f:
            w = csv.DictWriter(f, rows[0].keys()); w.writeheader(); w.writerows(rows)
        print("saved grasp_results.csv", sum(r["success"] for r in rows), "/", len(rows))
    elif a.view:
        print(trial(a.shape, a.size, a.mass, a.strategy, view=True))
    else:
        print(trial(a.shape, a.size, a.mass, a.strategy))
