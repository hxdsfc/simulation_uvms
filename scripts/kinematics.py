"""Joint-angle and TCP position inputs, with optional static visual preview."""
import argparse
import json
from pathlib import Path
import queue
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from uvms.kinematics import ArmKinematics
from simulate import write_png
import mujoco


def execute(arm, args, q=None, xyz=None):
    if xyz is not None:
        return arm.inverse(xyz, seed=q, unit=args.unit, frame=args.frame, gripper=args.gripper,
                           tolerance=args.tolerance, reject_collisions=args.reject_collisions)
    state = arm.forward(q if q is not None else arm.home_q,
                        unit=args.unit if q is not None else "rad", frame=args.frame, gripper=args.gripper)
    return {"success": True, "status": "forward_kinematics", "state": state}


def emit(result, as_json=False):
    out = ROOT / "outputs"
    out.mkdir(exist_ok=True)
    payload = json.dumps(result, indent=2, allow_nan=False)
    (out / "kinematics_result.json").write_text(payload, encoding="utf-8")
    if as_json:
        print(payload, flush=True)
    elif result["success"]:
        state = result["state"]
        print(f"{result['status']}: TCP ({state['frame']}, m) = "
              + " ".join(f"{x:.7f}" for x in state["position_m"]), flush=True)
        print("q (E D C B, deg) = " + " ".join(f"{x:.5f}" for x in state["q_deg"]), flush=True)
        if "error_m" in result:
            print(f"Position error = {result['error_m'] * 1000:.6f} mm", flush=True)
        print(f"Collision proxy check: {'clear' if state['collision_free'] else 'penetration detected'}", flush=True)
    else:
        print(f"IK failed: {result['status']}; best position error = {result['error_m'] * 1000:.6f} mm."
              " Current pose retained.", flush=True)


def interactive(arm, args):
    commands = queue.Queue()
    print(f"Input: q E D C B ({args.unit}); xyz X Y Z (m, {args.frame}); home; quit", flush=True)
    def read_input():
        while True:
            try:
                command = input("kinematics> ")
            except EOFError:
                command = "quit"
            commands.put(command)
            if command.strip().lower() in ("quit", "exit"):
                return
    threading.Thread(target=read_input, daemon=True).start()
    def handle(command):
        parts = command.split()
        if not parts:
            return True
        if parts[0].lower() in ("quit", "exit"):
            return False
        try:
            if parts == ["home"]:
                result = execute(arm, args)
            elif parts[0] == "q" and len(parts) == 5:
                result = execute(arm, args, q=[float(x) for x in parts[1:]])
            elif parts[0] == "xyz" and len(parts) == 4:
                result = execute(arm, args, xyz=[float(x) for x in parts[1:]])
            else:
                raise ValueError("Use q E D C B, xyz X Y Z, home, or quit")
            emit(result, args.json)
        except ValueError as exc:
            print(f"Invalid input: {exc}", flush=True)
        return True
    if args.viewer:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(arm.model, arm.data) as viewer:
            while viewer.is_running():
                try:
                    command = commands.get_nowait()
                except queue.Empty:
                    command = None
                if command is not None:
                    with viewer.lock():
                        keep_running = handle(command)
                    if not keep_running:
                        break
                viewer.sync()
                time.sleep(0.02)
    else:
        while handle(commands.get()):
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--q", nargs=4, type=float, metavar=("E", "D", "C", "B"), help="FK angles, or IK seed")
    parser.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"), help="Position IK target in metres")
    parser.add_argument("--unit", choices=("deg", "rad"), default="deg")
    parser.add_argument("--frame", choices=("world", "base"), default="world")
    parser.add_argument("--medium", choices=("air", "water"), default="water")
    parser.add_argument("--gripper", type=float, help="Opening fraction 0..1; default home opening")
    parser.add_argument("--tolerance", type=float, default=1e-5, help="Position tolerance in metres")
    parser.add_argument("--reject-collisions", action="store_true", help="Reject IK poses with proxy penetration")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--interactive", action="store_true", help="Read repeated q/xyz inputs from the terminal")
    parser.add_argument("--json", action="store_true", help="Print full machine-readable result")
    args = parser.parse_args()
    arm = ArmKinematics(args.medium)
    try:
        result = execute(arm, args, args.q, args.xyz)
    except ValueError as exc:
        parser.error(str(exc))
    emit(result, args.json)
    if not args.json:
        print(f"Full result: {ROOT / 'outputs/kinematics_result.json'}", flush=True)
    if not result["success"]:
        return 2
    if args.render:
        with mujoco.Renderer(arm.model, height=960, width=1280) as renderer:
            renderer.update_scene(arm.data, camera="overview")
            write_png(ROOT / "outputs/kinematics_pose.png", renderer.render())
    if args.interactive:
        interactive(arm, args)
    elif args.viewer:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(arm.model, arm.data) as viewer:
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)
    return 0


if __name__ == "__main__":
    sys.exit(main())
