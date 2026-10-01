"""Build a grasp scene from hand.xml (does not modify hand.xml).
make_scene(shape, size, mass) -> path of a generated xml (in the same folder)."""
import os, re
HERE = os.path.dirname(os.path.abspath(__file__))

# object centre in the palm pocket (SolidWorks frame, metres). Palm face is -Y.
OBJ_POS = (0.100, 0.045, 0.110)

def geom_xml(shape, size):
    """size (metres): sphere [diameter]; cylinder [diameter, height]; box [lx, ly, lz]"""
    if shape == "sphere":
        return 'type="sphere" size="%g"' % (size[0] / 2)
    if shape == "cylinder":          # axis along Z (parallel to the finger joint axes)
        return 'type="cylinder" size="%g %g"' % (size[0] / 2, size[1] / 2)
    if shape == "box":
        return 'type="box" size="%g %g %g"' % (size[0] / 2, size[1] / 2, size[2] / 2)
    raise ValueError(shape)

def make_scene(shape, size, mass=0.1, pos=OBJ_POS, out="_scene.xml", fric=1.0):
    s = open(os.path.join(HERE, "hand.xml")).read()
    s = s.replace('gravity="0 0 0"', 'gravity="0 9.81 0"')         # palm (-Y) is "up"
    s = s.replace('timestep="0.001"', 'timestep="0.002" cone="elliptic" impratio="5"')
    # friction for every collision geom of the hand (group 0) + palm collision hulls
    pad = ('friction="%g 0.005 0.0001" solref="0.004 1" condim="4"' % fric)
    s = re.sub(r'(<geom type="mesh" mesh="(?:f_prox|f_mid|f_dist|t_base|t_prox|t_dist)") group="0"',
               r'\1 group="0" ' + pad, s)
    palm = ('  <geom name="palm_base" type="mesh" mesh="palmbase" group="0" %s/>\n'
            '  <geom name="palm_servos" type="mesh" mesh="servos" group="0" %s/>\n'
            '  <geom name="palm_plate1" type="mesh" mesh="plate1" group="0" %s/>\n') % (pad, pad, pad)
    s = s.replace('  <body name="middle"', palm + '  <body name="middle"', 1)
    obj = ('  <body name="obj" pos="%g %g %g"><freejoint name="obj_free"/>'
           '<geom name="obj_geom" %s mass="%g" %s rgba="1 0.5 0.1 1"/></body>\n'
           % (pos[0], pos[1], pos[2], geom_xml(shape, size), mass, pad))
    s = s.replace(' </worldbody>', obj + ' </worldbody>')
    # sensors: fingertip touch is derived from contacts in python, nothing needed here
    ex = ''.join('  <exclude body1="world" body2="%s_prox"/>\n' % d for d in ('index','middle','ring','pinky'))
    s = s.replace(' <actuator>', ' <contact>\n' + ex + ' </contact>\n <actuator>')
    p = os.path.join(HERE, out)
    open(p, "w").write(s)
    return p
