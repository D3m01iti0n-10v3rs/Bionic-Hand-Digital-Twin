import opensim as osim, math
p = r"D:\doan2_realz\shit_for_git\simu\ms_arm_and_hand-main\AAH Model\RightArmAndHand_NoMuscles.osim"
m = osim.Model(p)
m.initSystem()
cs = m.getCoordinateSet()
for i in range(cs.getSize()):
    c = cs.get(i)
    rot = c.getMotionType() == 1
    lo, hi = c.getRangeMin(), c.getRangeMax()
    if rot:
        lo, hi = math.degrees(lo), math.degrees(hi)
    print(f"{c.getName():<20} {'rot' if rot else 'trans'}  {lo:8.1f} {hi:8.1f}")