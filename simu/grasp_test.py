"""Grasp simulation (#7).  python grasp_test.py --shape sphere|cylinder|box --size ... [--view] | --sweep
ctrl = real servo angle (deg): fingers 180=open..60=closed, thumb 0=open..180=closed."""

import argparse, csv, functools, itertools, os, time, numpy as np, mujoco

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
        self.shape = shape; self.size = tuple(size)
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

# ======================= Biological hand (OpenSim) -> robot imitation =======================
# 1) OpenSim: the human hand model (AAH, bones + joints + real joint limits) is simulated GRASPING the object:
#    forward dynamics with contact (Hunt-Crossley), joints driven by torque, no muscles.
#    The model has no collision shapes, so contact spheres are attached to the phalanges and the palm.
# 2) The joint-angle trajectories of that biological grasp are imitated by the robot: at every instant the servo
#    command whose free-hand joint angles (measured in MuJoCo) best fit the human angles is used.
# Computed live on every run (nothing cached on disk). Needs: pip install opensim, AAH model folder next to this file.
HUMAN_MODEL = os.path.join(HERE, 'ms_arm_and_hand-main', 'AAH Model', 'RightArmAndHand_NoMuscles.osim')
LONG = [d for d in DIGITS if d != 'thumb']
FINGER_N = {'index': 2, 'middle': 3, 'ring': 4, 'pinky': 5}
QMAX = {'mcp': 90.0, 'pip': 100.0, 'dip': 70.0}                       # full-fist flexion targets of the driving torques (deg)
THUMB_C = [('ra_cmc1_f_e', 40.0), ('ra_cmc1_opp', 30.0), ('ra_mcp1_e_f', 35.0), ('ra_ip1_e_f', 60.0)]
ROBOT_JOINTS = {dg: (('cmc', 'pip', 'ip') if dg == 'thumb' else ('mcp', 'pip', 'dip')) for dg in DIGITS}
T_CLOSE, T_END, TAU0 = 1.0, 1.6, 0.06                                 # s closing, s total, N.m driving torque scale
PAD_R, OBJ_MASS = 0.008, 0.1                                          # fingertip contact sphere radius (m), object mass (kg)
PALM_SHIFT, PALM_OFFSET = 0.5, 0.012                                  # object centre: toward wrist (x MCP-line width), above MCP axes (m)


def _robot_curves(T=8.0, hold=2.0):
    """Free-hand measurement in MuJoCo: for each digit, joint flexion (deg) vs servo command (slow ramp, no object)."""
    sim = Sim('sphere', (50,)); m, d = sim.m, sim.d; dt = m.opt.timestep
    def settle(max_t=6.0):
        for _ in range(int(max_t / 0.1)):
            sim.step(int(0.1 / dt))
            if np.max(np.abs(d.qvel)) < 1e-3: break
    curves, q_open, signs = {}, {}, {}
    for moving in (LONG, ['thumb']):
        sim.reset(); m.opt.gravity[:] = 0
        a = m.jnt_qposadr[m.joint('obj_free').id]; d.qpos[a:a + 3] = [5.0, 5.0, 5.0]    # park the object far away
        mujoco.mj_forward(m, d)
        adr = {(dg, j): m.jnt_qposadr[m.joint(f'{dg}_{j}').id] for dg in moving for j in ROBOT_JOINTS[dg]}
        for dg in DIGITS: d.ctrl[sim.act[dg]] = OPEN[dg]
        settle(); q0 = {k: np.degrees(d.qpos[v]) for k, v in adr.items()}
        n = int(T / dt); s_log, q_log = [], {k: [] for k in adr}
        for i in range(n + 1):
            s = i / n
            for dg in moving: d.ctrl[sim.act[dg]] = OPEN[dg] + s * (CLOSED[dg] - OPEN[dg])
            sim.step(1)
            if i % 10 == 0:
                s_log.append(s)
                for k, v in adr.items(): q_log[k].append(np.degrees(d.qpos[v]) - q0[k])
        sim.step(int(hold / dt)); settle()
        for k, v in adr.items(): q_log[k][-1] = np.degrees(d.qpos[v]) - q0[k]           # last point = settled value
        s_log = np.array(s_log)
        for dg in moving:
            fl = []
            for j in ROBOT_JOINTS[dg]:
                q = np.array(q_log[(dg, j)]); signs[(dg, j)] = 1.0 if q[-1] >= 0 else -1.0
                q_open[(dg, j)] = q0[(dg, j)]; fl.append(signs[(dg, j)] * q)
            curves[dg] = dict(ctrl=OPEN[dg] + s_log * (CLOSED[dg] - OPEN[dg]), flex=np.array(fl))
    return curves, q_open, signs


@functools.lru_cache(maxsize=None)
def _human_setup():
    """Open the OpenSim hand once: palm frame (from the 4 MCP joints) + robot free-hand curves."""
    try:
        import opensim as osim
    except ImportError:
        raise SystemExit('Can OpenSim: pip install opensim')
    if not os.path.exists(HUMAN_MODEL):
        raise SystemExit('Khong thay mo hinh tay nguoi: ' + HUMAN_MODEL)
    print('OpenSim: dang mo mo hinh ban tay sinh hoc...', flush=True)
    model = osim.Model(HUMAN_MODEL); state = model.initSystem(); cs = model.getCoordinateSet()
    v3 = lambda v: np.array([v.get(0), v.get(1), v.get(2)])
    def setq(name, deg):
        c = cs.get(name); lo, hi = c.getRangeMin(), c.getRangeMax(); rot = c.getMotionType() == 1
        if rot: lo, hi = np.degrees(lo), np.degrees(hi)
        v = float(np.clip(deg, lo, hi)); c.setValue(state, float(np.radians(v)) if rot else v, False)
    def tip_of(dip_coord):
        j = cs.get(dip_coord).getJoint(); c0 = v3(j.getChildFrame().getPositionInGround(state))
        body = osim.Body.safeDownCast(j.getChildFrame().findBaseFrame())
        com = v3(body.findStationLocationInGround(state, body.getMassCenter()))
        return c0 + 2.0 * (com - c0)                                   # fingertip ~ 2 x distal COM from the joint
    def pose(n, r):
        for j in ('mcp', 'pip', 'dip'): setq(f'ra_{j}{n}_e_f', r * QMAX[j])
        model.realizePosition(state)
    for n in FINGER_N.values(): pose(n, 0.0)
    for nm, _ in THUMB_C: setq(nm, 0.0)
    model.realizePosition(state)
    mcp = {n: v3(cs.get(f'ra_mcp{n}_e_f').getJoint().getChildFrame().getPositionInGround(state)) for n in FINGER_N.values()}
    c_mcp = np.mean(list(mcp.values()), axis=0); width = np.linalg.norm(mcp[2] - mcp[5])
    vlat = (mcp[2] - mcp[5]) / width
    u = tip_of('ra_dip3_e_f') - mcp[3]; u = u - np.dot(u, vlat) * vlat; u /= np.linalg.norm(u)
    nrm = np.cross(u, vlat); t0 = tip_of('ra_dip2_e_f'); pose(2, 1.0)
    if np.dot(nrm, tip_of('ra_dip2_e_f') - t0) < 0: nrm = -nrm          # normal points to where the fingers curl
    pose(2, 0.0); model.realizePosition(state)
    print('MuJoCo: dang do duong cong goc khop - lenh servo cua robot (tay khong, vai chuc giay)...', flush=True)
    curves, q_open, signs = _robot_curves()
    return dict(osim=osim, model=model, state=state, cs=cs, v3=v3, setq=setq, tip_of=tip_of, c_mcp=c_mcp, width=width,
                vlat=vlat, u=u, nrm=nrm, curves=curves, q_open=q_open, signs=signs)


def _write_obj(path, shape, sz, center, u, nrm, vlat):
    """Closed convex mesh (box or cylinder) with vertices as offsets from the object centre, axes = palm axes."""
    V, F = [], []
    if shape == 'box':
        h = [x / 2 for x in sz]
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz_ in (-1, 1): V.append(sx * h[0] * u + sy * h[1] * nrm + sz_ * h[2] * vlat)
        idx = lambda a, b, c: (a > 0) * 4 + (b > 0) * 2 + (c > 0)
        quads = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for ax in range(3):
            for sgn in (-1, 1):
                o = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
                pts = []
                for p, q in o:
                    a = [0, 0, 0]; a[ax] = sgn; a[(ax + 1) % 3] = p; a[(ax + 2) % 3] = q; pts.append(idx(*a))
                F += [(pts[0], pts[1], pts[2]), (pts[0], pts[2], pts[3])]
    else:
        R, hh, N = sz[0] / 2, sz[1] / 2, 24
        for k in range(N):
            th = 2 * np.pi * k / N
            for sgn in (-1, 1): V.append(np.cos(th) * R * u + np.sin(th) * R * nrm + sgn * hh * vlat)
        V.append(-hh * vlat); V.append(hh * vlat)
        for k in range(N):
            k2 = (k + 1) % N
            a0, a1, b0, b1 = 2 * k, 2 * k + 1, 2 * k2, 2 * k2 + 1
            F += [(a0, b0, b1), (a0, b1, a1), (2 * N, b0, a0), (2 * N + 1, a1, b1)]
    V = np.array(V)
    with open(path, 'w') as f:
        for v in V: f.write('v %.6f %.6f %.6f\n' % tuple(v))
        for a, b, c in F:
            n_t = np.cross(V[b] - V[a], V[c] - V[a])
            if np.dot(n_t, (V[a] + V[b] + V[c]) / 3) < 0: b, c = c, b          # outward-facing triangles
            f.write('f %d %d %d\n' % (a + 1, b + 1, c + 1))


@functools.lru_cache(maxsize=None)
def _human_grasp(shape, size):
    """OpenSim forward-dynamics grasp of the biological hand. Returns time t, flexion (deg) per digit joint,
    and the servo-command trajectory the robot uses to imitate it."""
    S = _human_setup(); osim = S['osim']; u, nrm, vlat = S['u'], S['nrm'], S['vlat']
    sz = [v / 1000.0 for v in size]
    thick = sz[1] if shape == 'box' else sz[0]
    center = S['c_mcp'] - u * (PALM_SHIFT * S['width']) + nrm * (PALM_OFFSET + thick / 2)
    stage = 'start'
    try:
        t_start = time.time()
        print(f'OpenSim: dang mo phong ban tay sinh hoc om {shape} {size} (dong luc hoc thuan co va cham)...', flush=True)
        stage = 'thumb direction'
        best = None
        for sg in itertools.product((1, -1), repeat=4):
            for (nm, q), s_ in zip(THUMB_C, sg): S['setq'](nm, s_ * q)
            S['model'].realizePosition(S['state'])
            dd = np.linalg.norm(S['tip_of']('ra_ip1_e_f') - center)
            if best is None or dd < best[0]: best = (dd, sg)
        for nm, _ in THUMB_C: S['setq'](nm, 0.0)
        S['model'].realizePosition(S['state'])
        tsign = best[1]
        stage = 'load model'
        model = osim.Model(HUMAN_MODEL); model.setGravity(osim.Vec3(0, 0, 0)); cs = model.getCoordinateSet()
        bset = model.getBodySet(); v3 = S['v3']
        stage = 'object body'
        r_obj = max(sz) / 2
        obj = osim.Body('obj', OBJ_MASS, osim.Vec3(0, 0, 0), osim.Inertia(0.4 * OBJ_MASS * r_obj ** 2, 0.4 * OBJ_MASS * r_obj ** 2, 0.4 * OBJ_MASS * r_obj ** 2))
        model.addBody(obj)
        fj = osim.FreeJoint('obj_free', model.getGround(), obj); model.addJoint(fj)
        for k in range(3): fj.upd_coordinates(3 + k).setDefaultValue(float(center[k]))
        stage = 'contact geometry'
        geoms = []
        if shape == 'sphere':
            model.addContactGeometry(osim.ContactSphere(sz[0] / 2, osim.Vec3(0, 0, 0), obj, 'obj_cg'))
        else:
            path = os.path.join(HERE, '_obj_contact.obj'); _write_obj(path, shape, sz, center, u, nrm, vlat)
            model.addContactGeometry(osim.ContactMesh(path, osim.Vec3(0, 0, 0), osim.Vec3(0, 0, 0), obj, 'obj_cg'))
        geoms.append('obj_cg')
        def body_of(coord): return bset.get(cs.get(coord).getJoint().getChildFrame().findBaseFrame().getName())
        def add_pad(coord, k, name, scale):
            b = body_of(coord); com = v3(b.get_mass_center())
            model.addContactGeometry(osim.ContactSphere(PAD_R, osim.Vec3(*(scale * com)), b, name)); geoms.append(name)
        for dg, n in FINGER_N.items():
            add_pad(f'ra_dip{n}_e_f', 0, f'pad_{dg}_dist', 1.5); add_pad(f'ra_pip{n}_e_f', 0, f'pad_{dg}_mid', 1.0)
            add_pad(f'ra_mcp{n}_e_f', 0, f'pad_{dg}_prox', 1.0)
        add_pad('ra_ip1_e_f', 0, 'pad_thumb_dist', 1.5); add_pad('ra_mcp1_e_f', 0, 'pad_thumb_prox', 1.0)
        palm = bset.get(S['cs'].get('ra_mcp3_e_f').getJoint().getParentFrame().findBaseFrame().getName())
        palm0 = S['model'].getBodySet().get(palm.getName())
        for i, (a, b) in enumerate(itertools.product((0.3, 0.6, 0.9), (-0.35, 0.0, 0.35))):
            pg = S['c_mcp'] - u * (a * S['width']) + vlat * (b * S['width']) + nrm * (PALM_OFFSET - 0.010)
            loc = v3(S['model'].getGround().findStationLocationInAnotherFrame(S['state'], osim.Vec3(*pg), palm0))
            model.addContactGeometry(osim.ContactSphere(0.010, osim.Vec3(*loc), palm, f'palm_{i}')); geoms.append(f'palm_{i}')
        hc = osim.HuntCrossleyForce(); hc.setName('contact')
        CP = getattr(osim, 'HuntCrossleyForce_ContactParameters', None) or osim.HuntCrossleyForce.ContactParameters
        prm = CP()
        for g_ in geoms: prm.addGeometry(g_)
        prm.setStiffness(1.0e6); prm.setDissipation(2.0); prm.setStaticFriction(0.8)
        prm.setDynamicFriction(0.6); prm.setViscousFriction(0.01); hc.addContactParameters(prm)
        model.addForce(hc)
        stage = 'actuators'
        drive = [(f'ra_{j}{n}_e_f', QMAX[j], 1.0) for n in FINGER_N.values() for j in ('mcp', 'pip', 'dip')]
        drive += [(nm, q, float(s_)) for (nm, q), s_ in zip(THUMB_C, tsign)]
        pc = osim.PrescribedController()
        for nm, q, s_ in drive:
            c = cs.get(nm); lo, hi = np.degrees(c.getRangeMin()), np.degrees(c.getRangeMax())
            tau = TAU0 * q / 100.0; b = tau * T_CLOSE / np.radians(q)
            model.addForce(osim.CoordinateLimitForce(nm, float(hi), 20.0, float(lo), 20.0, 0.0, 2.0))
            model.addForce(osim.ExpressionBasedCoordinateForce(nm, f'-{b:.6f}*qdot'))
            act = osim.CoordinateActuator(nm); act.setName('act_' + nm); act.setOptimalForce(1.0)
            act.setMinControl(-10.0); act.setMaxControl(10.0); model.addForce(act); pc.addActuator(act)
            f = osim.PiecewiseLinearFunction(); f.addPoint(0.0, 0.0); f.addPoint(0.05, s_ * tau); f.addPoint(T_END, s_ * tau)
            pc.prescribeControlForActuator('act_' + nm, f)
        model.addController(pc)
        rep = osim.StatesTrajectoryReporter(); rep.setName('rep'); rep.set_report_time_interval(0.01); model.addComponent(rep)
        stage = 'init system'
        state = model.initSystem()
        names = {d_[0] for d_ in drive}
        for i in range(cs.getSize()):
            c = cs.get(i)
            if c.getName() not in names and not c.getName().startswith('obj_free') and not c.isConstrained(state):
                c.setLocked(state, True)
        stage = 'integrate'
        manager = osim.Manager(model); manager.setIntegratorAccuracy(1e-3)
        state.setTime(0.0); manager.initialize(state); manager.integrate(T_END)
        stage = 'read results'
        traj = rep.getStates(); nst = traj.getSize()
        t = np.zeros(nst); val = {nm: np.zeros(nst) for nm in names}
        for i in range(nst):
            st = traj.get(i); model.realizePosition(st); t[i] = st.getTime()
            for nm in names:
                c = cs.get(nm); val[nm][i] = np.degrees(c.getValue(st)) if c.getMotionType() == 1 else c.getValue(st)
        end = traj.get(nst - 1); model.realizePosition(end)
        obj_end = v3(obj.getPositionInGround(end))
    except Exception as e:
        raise SystemExit(f'OpenSim loi o buoc "{stage}": {type(e).__name__}: {e}')
    flex = {}
    for dg, n in FINGER_N.items(): flex[dg] = np.array([val[f'ra_{j}{n}_e_f'] for j in ('mcp', 'pip', 'dip')])
    flex['thumb'] = np.array([tsign[0] * val['ra_cmc1_f_e'], tsign[2] * val['ra_mcp1_e_f'], tsign[3] * val['ra_ip1_e_f']])
    ctrl = {}
    for dg in DIGITS:
        cv = S['curves'][dg]
        idx = [int(np.argmin(((cv['flex'] - flex[dg][:, k:k + 1]) ** 2).sum(0) + 1e-6 * np.arange(cv['flex'].shape[1]))) for k in range(len(t))]
        ctrl[dg] = (t, cv['ctrl'][np.maximum.accumulate(idx)])           # robot servo command that best fits the human joints
    move = np.linalg.norm(obj_end - center) * 1000
    print(f'\n== OPENSIM: ban tay sinh hoc om {shape} {"x".join(f"{v:g}" for v in size)} mm '
          f'(mo phong {T_END:.1f} s trong {time.time() - t_start:.0f} s, vat dich chuyen {move:.1f} mm) ==')
    print('ngon       goc gap cuoi (do)  [MCP PIP DIP; ngon cai CMC MCP IP]   | lenh servo robot de bat chuoc')
    for dg in DIGITS:
        f_ = flex[dg][:, -1]
        print(f'{dg:<8}{f_[0]:12.1f}{f_[1]:7.1f}{f_[2]:7.1f}{"":14}| {ctrl[dg][1][-1]:8.1f}', flush=True)
    return dict(t=t, flex=flex, ctrl=ctrl, tsign=tsign)


def print_results(sim, res):
    """MuJoCo section: how closely the robot followed the biological hand, plus the grasp outcome."""
    print(f"\n== MUJOCO: robot ({res['strategy']}) ==")
    if res['strategy'] == 'human':
        S = _human_setup(); H = _human_grasp(sim.shape, tuple(float(v) for v in sim.size))
        jl = res['jlog']; tt = np.array([t for t, _ in jl])
        print('ngon     lenh cuoi | robot gap cuoi (do)        | sinh hoc gap cuoi (do)      | sai so TB theo thoi gian (do) | luc (N)')
        tot_err = []
        for dg in DIGITS:
            rob = np.array([[S['signs'][(dg, j)] * (q[(dg, j)] - S['q_open'][(dg, j)]) for _, q in jl] for j in ROBOT_JOINTS[dg]])
            hum = np.array([np.interp(tt, H['t'], H['flex'][dg][k]) for k in range(3)])
            rmse = float(np.sqrt(np.mean((rob - hum) ** 2))); tot_err.append(rmse)
            print(f"{dg:<8}{res['ctrl_final'][dg]:9.1f}  |{rob[0, -1]:9.1f}{rob[1, -1]:8.1f}{rob[2, -1]:8.1f}   |"
                  f"{hum[0, -1]:9.1f}{hum[1, -1]:8.1f}{hum[2, -1]:8.1f}   |{rmse:15.1f}                |{res['force'].get(dg, 0.0):7.2f}")
        print(f"Do giong tay sinh hoc: sai so goc khop TB {np.mean(tot_err):.1f} do (5 ngon)")
    else:
        print('ngon     lenh cuoi | luc (N)')
        for dg in DIGITS: print(f"{dg:<8}{res['ctrl_final'][dg]:9.1f}  |{res['force'].get(dg, 0.0):7.2f}")
    print(f"Ket qua: {'THANH CONG' if res['success'] else 'THAT BAI'} | troi {res['drift_mm']:.1f} mm | "
          f"{res['n_digits_touch']} ngon cham | tong luc {res['total_force_N']:.2f} N | dong tay {res['close_time_s']:.2f} s")


def run_grasp(sim, tilt=90.0, rate=90.0, preload=8.0, f_touch=0.15, hold=3.0, digits=DIGITS, strategy='human',
              targets=None, verbose=False):
    """strategy 'contact': each digit closes at `rate` deg/s until it touches the object (> f_touch N),
    then goes `preload` more degrees (that sets the squeeze force of the position-controlled servo).
    strategy 'preset': drive every digit straight to targets[digit] (angles from a lookup).
    strategy 'human' (default): a biological hand (OpenSim, forward dynamics with contact) grasps the same object;
    the servos follow the servo-command trajectory that best reproduces its joint angles (see _human_grasp)."""
    m, d = sim.m, sim.d
    traj = {}
    if strategy == 'human':
        if sim.shape == 'cylinder_y': raise SystemExit('Chua co mo phong sinh hoc cho cylinder_y (dung sphere, cylinder, box)')
        traj = _human_grasp(sim.shape, tuple(float(v) for v in sim.size))['ctrl']
    is_traj = lambda dg: dg in traj
    is_preset = lambda dg: strategy == 'preset'
    is_contact = lambda dg: strategy == 'contact'
    jadr = {(dg, j): m.jnt_qposadr[m.joint(f'{dg}_{j}').id] for dg in DIGITS for j in ROBOT_JOINTS[dg]}
    jlog = []
    lim = lambda dg, v: min(max(v, min(OPEN[dg], CLOSED[dg])), max(OPEN[dg], CLOSED[dg]))
    t_end_traj = max((tt[-1] for tt, _ in traj.values()), default=0.0)
    dt = m.opt.timestep
    ctrl = dict(OPEN); stop = {dg: None for dg in DIGITS}      # stop angle once touched
    t0 = d.time; closing_s = 0
    # phase 0: settle open hand 0.2 s
    sim.step(int(0.2 / dt))
    # phase 1: close
    tmax = 4.0; touch_angle = {}; t_close0 = d.time; k_log = 0
    while d.time - t0 < tmax + 0.2:
        cf = sim.contacts()
        for dg in digits:
            if is_traj(dg):                                    # follow the biological hand's trajectory
                tt, cc = traj[dg]; el = d.time - t_close0
                tgt = float(np.interp(el, tt, cc)) if el < tt[-1] else lim(dg, cc[-1] + SIGN[dg] * preload)
                step = 600.0 * dt
                ctrl[dg] += np.clip(tgt - ctrl[dg], -step, step)
                d.ctrl[sim.act[dg]] = ctrl[dg]
                continue
            tgt = (targets or CLOSED)[dg] if is_preset(dg) else CLOSED[dg]
            if is_contact(dg) and stop[dg] is None and cf.get(dg, 0) > f_touch:
                stop[dg] = ctrl[dg] + SIGN[dg] * preload
                touch_angle[dg] = ctrl[dg]
            if stop[dg] is not None: tgt = stop[dg]
            step = rate * dt
            diff = tgt - ctrl[dg]
            ctrl[dg] += np.clip(diff, -step, step)
            d.ctrl[sim.act[dg]] = ctrl[dg]
        sim.step(1)
        k_log += 1
        if traj and k_log % 20 == 0:
            jlog.append((d.time - t_close0, {k: float(np.degrees(d.qpos[v])) for k, v in jadr.items()}))
        done = all(abs(ctrl[dg] - (lim(dg, traj[dg][1][-1] + SIGN[dg] * preload) if is_traj(dg) else
                                   ((targets or CLOSED)[dg] if stop[dg] is None else stop[dg]))) < 0.01 for dg in digits)
        if done and d.time - t_close0 >= t_end_traj + 0.2 and all(
                (not is_contact(dg)) or stop[dg] is not None or abs(ctrl[dg]-CLOSED[dg])<0.01 for dg in digits):
            break
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
    return dict(strategy=strategy, success=bool(success), drift_mm=drift, max_drift_mm=max_drift, close_time_s=closing_s,
                n_digits_touch=n_contact_digits, palm=cf_end.get('palm', 0.0) > 0.05,
                total_force_N=sum(cf_end.values()),
                touch_angle=touch_angle, force=cf_end, final_pos_mm=(end * 1000).round(1).tolist(),
                ctrl_final={k: round(v, 1) for k, v in ctrl.items()}, jlog=jlog)

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
            sim.view = v; r = run_grasp(sim, tilt=a.tilt, rate=a.rate, preload=a.preload); print_results(sim, r)
            print('Done. Servos are free now: use the Control sliders in the viewer.')
            while v.is_running():            # keep the physics running so the sliders keep working
                sim.step(10)
    else:
        print_results(sim, run_grasp(sim, tilt=a.tilt, rate=a.rate, preload=a.preload))