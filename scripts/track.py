"""Run a timed trajectory through torque-controlled physical simulation."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from uvms.kinematics import ArmKinematics
from uvms.tracking import TrackingSession
from uvms.pick_place import load_reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory",type=Path)
    parser.add_argument("--medium",choices=("air","water"),default="air")
    parser.add_argument("--time-scale",type=float,default=1)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--no-plots",action="store_true")
    args = parser.parse_args()
    arm = ArmKinematics(args.medium)
    try:
        reference = load_reference(arm,args.trajectory,args.time_scale)
    except (ValueError,KeyError) as exc:
        parser.error(str(exc))
    output = args.output or ROOT/"outputs"/f"tracking_{datetime.now():%Y%m%d_%H%M%S_%f}"
    session = TrackingSession(arm,reference)
    try:
        session.run()
    except KeyboardInterrupt:
        session.stop()
    except RuntimeError as exc:
        print(f"Physical tracking failed: {exc}",file=sys.stderr)
    finally:
        session.export(output,plots=not args.no_plots)
    print(json.dumps(session.summary(),indent=2,ensure_ascii=False))
    print(f"Output: {output}")
    return 0 if session.status == "completed" else 2


if __name__ == "__main__":
    sys.exit(main())
