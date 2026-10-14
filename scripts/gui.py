"""Launch the native Alpha 5 kinematics workbench."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uvms.gui import main

if __name__ == "__main__":
    sys.exit(main())
