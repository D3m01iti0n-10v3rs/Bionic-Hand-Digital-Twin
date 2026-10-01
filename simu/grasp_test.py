"""Grasp simulation (#7).  python grasp_test.py --shape sphere|cylinder|box --size ... [--view] | --sweep
ctrl = real servo angle (deg): fingers 180=open..60=closed, thumb 0=open..180=closed."""

import argparse, csv, os, time, numpy as np, mujoco

OPEN = {'thumb': 0.0, 'index': 180.0, 'middle': 180.0, 'ring': 180.0, 'pinky': 180.0}
CLOSED = {'thumb': 180.0, 'index': 60.0, 'middle': 60.0, 'ring': 60.0, 'pinky': 60.0}
DIGITS = ['thumb', 'index', 'middle', 'ring', 'pinky']
SIGN = {d: (1.0 if d == 'thumb' else -1.0) for d in DIGITS}      # +1 = ctrl increases when closing


import argparse, os, numpy as np, trimesh
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
DIGITS = ['thumb', 'index', 'middle', 'ring', 'pinky']
PALM_Y = 0.0626           # palm-side face of plate1 (SolidWorks Y, m)
PALM_X = (0.019, 0.084); PALM_Z = (0.064, 0.134)

def _slab_meshes(stl, k, thin_axis):
    """Split a thin curved plate into k convex pieces (hulls of surface-point slabs)."""
    m = trimesh.load(stl)
    pts, _ = trimesh.sample.sample_surface(m, 60000, seed=0)
    ax = [a for a in range(3) if a != thin_axis]
    p2 = pts[:, ax] - pts[:, ax].mean(0)
    u, s, vt = np.linalg.svd(p2, full_matrices=False)
    t = p2 @ vt[0]
    edges = np.linspace(t.min(), t.max(), k + 1)
    ov = 0.08 * (t.max() - t.min()) / k
    out = []
    for i in range(k):
        sel = pts[(t >= edges[i] - ov) & (t <= edges[i + 1] + ov)]
        if len(sel) > 30:
            out.append(trimesh.convex.convex_hull(sel))
    return out

# mesh -> (pieces, thin axis).  Thin axis: fingers z(2), thumb x(0)
COL = {'f_prox': (4, 2), 'f_mid': (4, 2), 'f_dist': (4, 2),
       't_base': (3, 0), 't_prox': (4, 0), 't_dist': (4, 0)}

def make_collision_assets():
    names = {}
    for mesh, (k, ax) in COL.items():
        stl = os.path.join(HERE, 'assets', mesh + '.stl')
        ps = _slab_meshes(stl, k, ax)
        names[mesh] = []
        for i, h in enumerate(ps):
            n = f'col_{mesh}_{i}'
            h.export(os.path.join(HERE, 'assets', n + '.stl'))
            names[mesh].append(n)
    return names

def build_scene(shape='sphere', size=(50,), pos=None, mass=0.1, friction=0.9,
                gravity_dir='up', out='hand_grasp.xml', table=True, tilt=None):
    """size (mm): sphere (d,) | cylinder (d,h) | box (lx,ly,lz).  pos (mm) = object centre
    (x,y,z) in hand frame; None -> auto (touching below the palm).
    gravity_dir: 'down' (palm faces the floor, object on table below) | 'up' (palm up) | 'none'"""
    cn = make_collision_assets()
    tree = ET.parse(os.path.join(HERE, 'hand.xml')); root = tree.getroot()
    opt = root.find('option')
    g = '0 0 -9.81'
    opt.set('gravity', g); opt.set('timestep', '0.001'); opt.set('cone', 'elliptic')
    opt.set('impratio', '10'); opt.set('noslip_iterations', '3')
    # assets for collision hulls
    asset = root.find('asset')
    for lst in cn.values():
        for n in lst:
            ET.SubElement(asset, 'mesh', name=n, file=n + '.stl', scale='0.001 0.001 0.001')
    fr = f'{friction} 0.02 0.005'
    # turn phalanx visuals into non-colliding visuals, add hull pieces to same body
    meshmap = {'f_prox': 'f_prox', 'f_mid': 'f_mid', 'f_dist': 'f_dist',
               't_base': 't_base', 't_prox': 't_prox', 't_dist': 't_dist'}
    for body in root.iter('body'):
        for geom in list(body.findall('geom')):
            mn = geom.get('mesh')
            if mn in meshmap and geom.get('group') == '0':
                geom.set('contype', '0'); geom.set('conaffinity', '0'); geom.set('group', '1')
                for n in cn[mn]:
                    ET.SubElement(body, 'geom', type='mesh', mesh=n, group='3', density='0',
                                  friction=fr, solref='0.004 1', solimp='0.9 0.95 0.001',
                                  rgba='1 .3 .3 .3', condim='4')
    wb = root.find('worldbody')
    # palm up: rotate the whole hand so its palm side (-Y in CAD) points to world +Z
    hand = ET.Element('body', name='hand', euler='-90 0 0')
    for ch in list(wb): wb.remove(ch); hand.append(ch)
    wb.append(hand)
    # palm (fixed body so we can <exclude> it)
    palm = ET.Element('body', name='palm')
    cx = (PALM_X[0]+PALM_X[1])/2; cz = (PALM_Z[0]+PALM_Z[1])/2
    ET.SubElement(palm, 'geom', name='palm_geom', type='box', pos=f'{cx} {PALM_Y+0.0025} {cz}',
                  size=f'{(PALM_X[1]-PALM_X[0])/2} 0.0025 {(PALM_Z[1]-PALM_Z[0])/2}',
                  group='3', density='0', friction=fr, rgba='1 .3 .3 .3', condim='4')
    hand.insert(0, palm)
    # object geometry
    s = [v/1000 for v in size]
    if shape == 'sphere':
        gtype, gsize, half_y = 'sphere', f'{s[0]/2}', s[0]/2
    elif shape == 'cylinder':       # axis along Z (across the palm), like a handle/bar
        gtype, gsize, half_y = 'cylinder', f'{s[0]/2} {s[1]/2}', s[0]/2
    elif shape == 'cylinder_y':     # upright, axis along Y
        gtype, gsize, half_y = 'cylinder', f'{s[0]/2} {s[1]/2}', s[1]/2
    elif shape == 'box':
        gtype, gsize, half_y = 'box', f'{s[0]/2} {s[1]/2} {s[2]/2}', s[1]/2
    else: raise ValueError(shape)
    if pos is None:
        pos = (60.0, None, 98.0)
    ox, oy, oz = pos[0]/1000, pos[1]/1000 if pos[1] is not None else None, pos[2]/1000
    sgn = -1 if gravity_dir != 'up' else -1   # object always on the palm side (-Y)
    if oy is None: oy = PALM_Y - half_y - 0.002
    ob = ET.SubElement(wb, 'body', name='obj', pos=f'{ox} {oz} {-oy}', euler='-90 0 0')   # CAD (x,y,z) -> world (x,z,-y)
    ET.SubElement(ob, 'freejoint', name='obj_free')
    geom_attr = dict(name='obj_geom', type=gtype, size=gsize, mass=str(mass), friction=fr,
                     rgba='0.95 0.55 0.1 1', condim='4', solref='0.004 1', solimp='0.9 0.95 0.001')
    if shape == 'cylinder_y': geom_attr['zaxis'] = '0 1 0'
    ET.SubElement(ob, 'geom', **geom_attr)
    # contact excludes (links of the same finger that cannot really touch + palm vs thumb base)
    ct = root.find('contact')
    if ct is None: ct = ET.SubElement(root, 'contact')
    for d in DIGITS:
        p = f'{d}_prox' if d != 'thumb' else 'thumb_prox'
        if d != 'thumb':
            ET.SubElement(ct, 'exclude', body1=f'{d}_prox', body2=f'{d}_dist')
        else:
            ET.SubElement(ct, 'exclude', body1='thumb_prox', body2='thumb_cmc')
    ET.SubElement(ct, 'exclude', body1='palm', body2='thumb_cmc')
    ET.SubElement(ct, 'exclude', body1='palm', body2='thumb_prox')
    # sensors: servo angle + actuator force are handy for logging
    tree.write(os.path.join(HERE, out))
    return os.path.join(HERE, out)

if False:
    ap = argparse.ArgumentParser()
    ap.add_argument('--shape', default='sphere'); ap.add_argument('--size', type=float, nargs='+', default=[50])
    ap.add_argument('--mass', type=float, default=0.1); ap.add_argument('--friction', type=float, default=0.9)
    ap.add_argument('--gravity', default='down')
    a = ap.parse_args()
    print(build_scene(a.shape, tuple(a.size), mass=a.mass, friction=a.friction, gravity_dir=a.gravity))

class Sim:
    def __init__(self, shape, size, pos=None, mass=0.1, friction=0.9, gravity='up', tilt=None):
        self.xml = build_scene(shape, tuple(size), pos=pos, mass=mass, friction=friction,
                               gravity_dir=gravity, tilt=tilt)
        self.m = mujoco.MjModel.from_xml_path(self.xml); self.d = mujoco.MjData(self.m)
        m = self.m
        self.act = {dg: m.actuator(f'{dg}_servo').id for dg in DIGITS}
        self.obj_body = m.body('obj').id; self.obj_geom = m.geom('obj_geom').id
        self.obj_qadr = m.jnt_qposadr[m.joint('obj_free').id]
        self.digit_of_body = {}
        for b in range(m.nbody):
            n = m.body(b).name
            for dg in DIGITS:
                if n.startswith(dg + '_'): self.digit_of_body[b] = dg
        self.view = None
        self.reset()
    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        for dg in DIGITS: self.d.ctrl[self.act[dg]] = OPEN[dg]
        mujoco.mj_forward(self.m, self.d)
        self.obj0 = self.d.xpos[self.obj_body].copy()
        self.t_log = []
    def step(self, n=1):
        for _ in range(n):
            mujoco.mj_step(self.m, self.d)
            if self.view is not None and self.d.ncon >= 0 and int(round(self.d.time / self.m.opt.timestep)) % 10 == 0:
                self.view.sync(); time.sleep(self.m.opt.timestep * 10)
    def obj_pos(self): return self.d.xpos[self.obj_body].copy()
    def contacts(self):
        """-> {digit or 'palm': normal force (N)} on the object right now"""
        out = {}; f6 = np.zeros(6)
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if self.obj_geom not in (c.geom1, c.geom2): continue
            other = c.geom2 if c.geom1 == self.obj_geom else c.geom1
            b = self.m.geom_bodyid[other]
            if b == self.m.body('palm').id: key = 'palm'
            elif b in self.digit_of_body: key = self.digit_of_body[b]
            else: continue
            mujoco.mj_contactForce(self.m, self.d, i, f6)
            out[key] = out.get(key, 0.0) + abs(f6[0])
        return out

def run_grasp(sim, tilt=90.0, rate=90.0, preload=8.0, f_touch=0.15, hold=3.0, digits=DIGITS, strategy='contact',
              targets=None, verbose=False):
    """strategy 'contact': each digit closes at `rate` deg/s until it touches the object (> f_touch N),
    then goes `preload` more degrees (that sets the squeeze force of the position-controlled servo).
    strategy 'preset': drive every digit straight to targets[digit] (angles from a lookup)."""
    m, d = sim.m, sim.d
    dt = m.opt.timestep
    ctrl = dict(OPEN); stop = {dg: None for dg in DIGITS}      # stop angle once touched
    t0 = d.time; closing_s = 0
    # phase 0: settle open hand 0.2 s
    sim.step(int(0.2 / dt))
    # phase 1: close
    tmax = 4.0; touch_angle = {}
    while d.time - t0 < tmax + 0.2:
        cf = sim.contacts()
        for dg in digits:
            tgt = (targets or CLOSED)[dg] if strategy == 'preset' else CLOSED[dg]
            if strategy == 'contact' and stop[dg] is None and cf.get(dg, 0) > f_touch:
                stop[dg] = ctrl[dg] + SIGN[dg] * preload
                touch_angle[dg] = ctrl[dg]
            if stop[dg] is not None: tgt = stop[dg]
            step = rate * dt
            diff = tgt - ctrl[dg]
            ctrl[dg] += np.clip(diff, -step, step)
            d.ctrl[sim.act[dg]] = ctrl[dg]
        sim.step(1)
        done = all(abs(ctrl[dg] - ((targets or CLOSED)[dg] if stop[dg] is None else stop[dg])) < 0.01
                   for dg in digits)
        if done and strategy == 'contact' and all(stop[dg] is not None or abs(ctrl[dg]-CLOSED[dg])<0.01 for dg in digits):
            break
        if done and strategy == 'preset': break
    closing_s = d.time - t0
    sim.step(int(0.5 / dt))                                      # let it squeeze/settle
    pre = sim.obj_pos(); cf_pre = sim.contacts()
    # phase 2: roll the hand sideways to `tilt` degrees over 1 s (gravity vector rotates), then hold
    n = int(1.0 / dt)
    for i in range(n):
        a = np.radians(tilt * (i + 1) / n)
        m.opt.gravity[:] = [0, 9.81 * np.sin(a), -9.81 * np.cos(a)]
        sim.step(1)
    track = []; nh = int(hold / dt)
    for i in range(nh):
        sim.step(1)
        if i % 20 == 0: track.append(sim.obj_pos())
    end = sim.obj_pos(); track = np.array(track)
    m.opt.gravity[:] = [0, 0, -9.81]            # back to palm-up
    cf_end = sim.contacts()
    drift = np.linalg.norm(end - sim.obj0) * 1000
    max_drift = np.max(np.linalg.norm(track - sim.obj0, axis=1)) * 1000
    fell = drift > 30.0
    n_contact_digits = sum(1 for k, v in cf_end.items() if k != 'palm' and v > 0.05)
    success = (not fell) and drift < 15.0 and n_contact_digits >= 2
    return dict(success=bool(success), drift_mm=drift, max_drift_mm=max_drift, close_time_s=closing_s,
                n_digits_touch=n_contact_digits, palm=cf_end.get('palm', 0.0) > 0.05,
                total_force_N=sum(cf_end.values()),
                touch_angle=touch_angle, force=cf_end, final_pos_mm=(end * 1000).round(1).tolist(),
                ctrl_final={k: round(v, 1) for k, v in ctrl.items()})

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--shape', default='sphere'); ap.add_argument('--size', type=float, nargs='+', default=[50])
    ap.add_argument('--mass', type=float, default=0.1); ap.add_argument('--friction', type=float, default=0.9)
    ap.add_argument('--pos', type=float, nargs=3, default=None, help='object centre x y z in mm (y may be 0 = auto)')
    ap.add_argument('--preload', type=float, default=8.0); ap.add_argument('--rate', type=float, default=90.0)
    ap.add_argument('--tilt', type=float, default=90.0, help='roll test angle in deg (0 = palm up only)'); ap.add_argument('--view', action='store_true')
    ap.add_argument('--sweep', action='store_true', help='all shapes x sizes -> grasp_results.csv')
    a = ap.parse_args()
    if a.sweep:
        import itertools
        cases = ([('sphere', (d,)) for d in (40, 50, 60, 70)] + [('cylinder', (d, 70)) for d in (30, 40, 50, 60)] +
                 [('box', s) for s in ((40, 40, 40), (50, 40, 60), (50, 50, 50), (60, 50, 60))])
        rows = []
        for (sh, sz), mass, pl in itertools.product(cases, (0.05, 0.1, 0.2), (10, 20)):
            r = run_grasp(Sim(sh, sz, pos=(65, None, 95), mass=mass), preload=pl)
            rows.append(dict(shape=sh, size_mm='x'.join(map(str, sz)), mass_kg=mass, preload_deg=pl,
                             success=r['success'], drift_mm=round(float(r['drift_mm']), 1),
                             digits_touching=r['n_digits_touch'], total_force_N=round(float(r['total_force_N']), 2)))
            print(rows[-1], flush=True)
        with open('grasp_results.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, list(rows[0])); w.writeheader(); w.writerows(rows)
        raise SystemExit
    pos = None
    if a.pos: pos = (a.pos[0], a.pos[1] if a.pos[1] else None, a.pos[2])
    sim = Sim(a.shape, a.size, pos=pos, mass=a.mass, friction=a.friction, gravity='up')
    if a.view:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(sim.m, sim.d) as v:
            sim.view = v; r = run_grasp(sim, tilt=a.tilt, rate=a.rate, preload=a.preload); print(r)
            print('Done. Servos are free now: use the Control sliders in the viewer.')
            while v.is_running():            # keep the physics running so the sliders keep working
                sim.step(10)
    else:
        print(run_grasp(sim, tilt=a.tilt, rate=a.rate, preload=a.preload))
