"""numpy and OpenCV are only for the close-up camera, so the plugin must load without them.

The test machine's OctoPrint Python has both (numpy 2.5, OpenCV 5), but the plugin installs with
pip --no-deps and a machine without them must still probe and run the LightBurn bridge. So no
module in the package may import them at the top: only inside the functions that need them.
"""
import ast
import os

import pytest

PACKAGE = os.path.join(os.path.dirname(__file__), os.pardir, "octoprint_laserheightprobe")
HEAVY = ("numpy", "cv2", "PIL")


def _modules():
    for name in sorted(os.listdir(PACKAGE)):
        if name.endswith(".py"):
            yield name


def _top_level_imports(path):
    with open(path, encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), path)
    found = []
    for node in tree.body:                       # module level only, not inside functions
        if isinstance(node, ast.Import):
            found += [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            found.append(node.module)
        elif isinstance(node, (ast.If, ast.Try)):    # a guarded import at the top still runs
            for inner in ast.walk(node):
                if isinstance(inner, ast.Import):
                    found += [alias.name for alias in inner.names]
                elif isinstance(inner, ast.ImportFrom) and inner.module and inner.level == 0:
                    found.append(inner.module)
    return found


@pytest.mark.parametrize("name", list(_modules()))
def test_no_module_imports_numpy_or_opencv_at_the_top(name):
    heavy = [m for m in _top_level_imports(os.path.join(PACKAGE, name))
             if m.split(".")[0] in HEAVY]
    assert heavy == [], "%s imports %s at module level: import it inside the function" % (name, heavy)


def test_the_guard_sees_a_top_level_import(tmp_path):
    bad = tmp_path / "bad.py"
    bad.write_text("import numpy as np\ntry:\n    import cv2\nexcept ImportError:\n    cv2 = None\n"
                   "def f():\n    import PIL\n")
    assert _top_level_imports(str(bad)) == ["numpy", "cv2"]
