"""Generate known-feasible position and full-pose example curves."""
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from uvms.kinematics import ArmKinematics
from uvms.spatial import matrix_quaternion


def main():
    arm = ArmKinematics("air")
    home = arm.home_q.copy()
    origin = arm.position().copy()
    pose,position,joint = [],[],[]
    for time in np.linspace(0,6,301):
        u = time/6
        smooth = 10*u**3-15*u**4+6*u**5
        angle = 2*np.pi*smooth
        offset = np.array([.012*(np.cos(angle)-1),.012*np.sin(angle),.003*np.sin(2*angle)])
        position.append({"t":float(time),"position":(origin+offset).tolist()})
        q = home+np.array([.06,.04,-.03,.08])*np.sin(np.pi*u)**2
        state = arm.forward(q,unit="rad")
        pose.append({"t":float(time),"position":state["position_m"],
                     "quaternion":matrix_quaternion(state["rotation_matrix"]).tolist()})
        joint.append({"t":float(time),"q":q.tolist()})
    directory = ROOT/"trajectories"
    directory.mkdir(exist_ok=True)
    for name,mode,points in (("position_loop","position",position),("feasible_pose","pose",pose),("joint_curve","joint",joint)):
        payload = {"mode":mode,"frame":"world","angle_unit":"rad","points":points}
        (directory/f"{name}.json").write_text(json.dumps(payload,indent=2),encoding="utf-8")
        print(directory/f"{name}.json")


if __name__ == "__main__":
    main()
