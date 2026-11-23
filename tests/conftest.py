"""Mount the plugin package without running its __init__.

`octoprint_laserheightprobe/__init__.py` imports flask and octoprint, which are not here.
Every other module in the package is plain Python, so the tests mount the directory under a
second name whose __init__ is empty. Relative imports inside the package still work.
"""
import pathlib
import sys
import types

ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT / "octoprint_laserheightprobe"

if "lhp_pkg" not in sys.modules:
    package = types.ModuleType("lhp_pkg")
    package.__path__ = [str(PACKAGE_DIR)]
    sys.modules["lhp_pkg"] = package

sys.path.insert(0, str(ROOT / "tests"))
