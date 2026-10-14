"""Preview model or perform a short environment smoke check; no controller."""
import argparse
import json
import math
from pathlib import Path
import struct
import time
import zlib

import mujoco
import numpy as np
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from uvms.kinematics import ArmKinematics

ROOT = Path(__file__).resolve().parents[1]


def write_png(path, pixels):
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff)
    h, w, _ = pixels.shape
    raw = b"".join(b"\x00" + row.tobytes() for row in pixels)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w,h,8,2,0,0,0))
                     + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--medium", choices=["air", "water"], default="air")
    parser.add_argument("--seconds", type=float, default=2)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / f"arm_{args.medium}.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    robot = ArmKinematics(model=model)
    robot.data = data
    assert model.nu == 5
    assert model.neq == 2, "Missing gripper coupling"
    scalar_joints = [i for i in range(model.njnt) if model.jnt_type[i] in (mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE)]
    for i in scalar_joints:
        value = data.qpos[model.jnt_qposadr[i]]
        assert model.jnt_range[i,0] <= value <= model.jnt_range[i,1]
    assert np.all(model.body_mass[1:] > 0)
    assert np.all(model.body_inertia[1:] > 0)
    # Check import/conversion preserves source body poses and physical parameters.
    source = mujoco.MjModel.from_xml_path(str(ROOT / "models/alpha5.urdf"))
    reference = mujoco.MjData(source)
    for i in range(source.njnt):
        name = source.joint(i).name
        reference.qpos[source.jnt_qposadr[i]] = data.qpos[model.jnt_qposadr[model.joint(name).id]]
    mujoco.mj_forward(source, reference)
    for i in range(1, source.nbody):
        name = mujoco.mj_id2name(source, mujoco.mjtObj.mjOBJ_BODY, i)
        target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert target >= 0, f"Missing body {name}"
        np.testing.assert_allclose(data.xpos[target], reference.xpos[i], atol=2e-6)
        np.testing.assert_allclose(data.xmat[target], reference.xmat[i], atol=2e-6)
        np.testing.assert_allclose(model.body_mass[target], source.body_mass[i], rtol=1e-5)
        np.testing.assert_allclose(model.body_inertia[target], source.body_inertia[i], rtol=1e-5)
    home_contacts = sum(robot.robot_contact(c) for c in data.contact)
    assert home_contacts == 0, "Home configuration has contacts"
    initial_tip = data.site_xpos[model.site("tool_tip").id].copy()
    out = ROOT / "outputs"
    out.mkdir(exist_ok=True)
    if args.render:
        with mujoco.Renderer(model, height=960, width=1280) as renderer:
            renderer.update_scene(data, camera="overview")
            write_png(out / f"arm_{args.medium}.png", renderer.render())
    if args.viewer:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model, data) as viewer:
            # Paused physical state for model inspection; GUI camera remains interactive.
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)
        return
    peak_contacts = 0
    for _ in range(math.ceil(args.seconds / model.opt.timestep)):
        data.ctrl[:] = 0
        mujoco.mj_step(model, data)
        peak_contacts = max(peak_contacts, data.ncon)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("Nonfinite simulation state")
    warnings = {str(i): int(w.number) for i, w in enumerate(data.warning) if w.number}
    if warnings:
        raise RuntimeError(f"MuJoCo warnings: {warnings}")
    report = {"mujoco": mujoco.__version__, "medium": args.medium, "nq": model.nq,
              "nv": model.nv, "nu": model.nu, "home_contacts": home_contacts,
              "initial_tool_position_m": initial_tip.tolist(), "duration_s": data.time,
              "peak_contacts": peak_contacts,
              "joint_states_final": {model.joint(i).name: {"position": float(data.qpos[model.jnt_qposadr[i]]),
                  "unit": "m" if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_SLIDE else "rad"}
                  for i in scalar_joints},
              "block_position_m": data.xpos[model.body("grasp_block").id].tolist(),
              "source_conversion_check": "passed",
              "warnings": warnings, "status": "passed", "scope": "uncontrolled environment smoke check"}
    (out / f"check_{args.medium}.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
