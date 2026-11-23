"""Loading the height map into the printer's mesh, and putting the print mesh back, against the
fake A350 and the real bridge."""
import copy

import pytest

from helpers import Rig
from lhp_pkg import heightmap, meshload

ALLOWED = {"M1006", "M420", "M421", "G1029", "M400", "M114", "M118"}


def a_map():
    pts = [dict(x=120.0 + 20 * i, y=100.0 + 20 * j, z=10.0 + 0.05 * (20 * i))
           for i in range(4) for j in range(4)]
    return heightmap.build(pts, (0.0, 0.0), focus_gap=40.0)


def test_load_writes_the_map_and_turns_levelling_on():
    rig = Rig()
    old = copy.deepcopy(rig.machine.mesh["z"])
    m = a_map()
    backup, loaded = meshload.load(rig.bridge, m)
    assert loaded is m
    assert backup["grid"] == old and backup["levelling"] is False and backup["fade"] == 10.0
    assert backup["toolhead"] == "10W LASER"
    assert rig.machine.mesh["z"] == m["nodes"]
    assert rig.machine.levelling is True and rig.machine.fade == 0.0
    for command in rig.machine.sent:
        assert command.split()[0] in ALLOWED, command


def test_restore_puts_the_print_mesh_levelling_and_fade_back():
    rig = Rig()
    old = copy.deepcopy(rig.machine.mesh["z"])
    backup, _ = meshload.load(rig.bridge, a_map())
    meshload.restore(rig.bridge, backup)
    assert rig.machine.mesh["z"] == old
    assert rig.machine.levelling is False and rig.machine.fade == 10.0
    assert meshload.matches(rig.bridge, backup)
    assert not any(c.startswith("M500") or c.startswith("M501") for c in rig.machine.sent)


def test_a_3d_printing_head_is_refused_before_anything_is_written():
    rig = Rig()
    rig.machine.toolhead = "3DP"
    with pytest.raises(meshload.MeshError, match="3DP"):
        meshload.load(rig.bridge, a_map())
    assert not any(c.split()[0] in ("G1029", "M421", "M420") for c in rig.machine.sent)


def test_a_refused_node_puts_the_old_mesh_back():
    rig = Rig()
    old = copy.deepcopy(rig.machine.mesh["z"])
    rig.machine.fail_m421_at = (5, 5)
    with pytest.raises(meshload.MeshError, match="old mesh is back"):
        meshload.load(rig.bridge, a_map())
    assert rig.machine.mesh["z"] == old and rig.machine.levelling is False
    assert rig.machine.fade == 10.0


def test_a_grid_elsewhere_is_refitted_to_where_the_printer_put_it():
    """A work origin in force at G1029 moves the firmware's grid: refit, do not refuse."""
    rig = Rig()
    rig.machine.g1029_start = (40.0, 43.0)
    m = a_map()
    backup, loaded = meshload.load(rig.bridge, m)
    assert loaded["start"] == [40.0, 43.0] and rig.machine.mesh["z"] == loaded["nodes"]
    assert loaded["nodes"] != m["nodes"] and abs(loaded["worst"]["miss"]) < 0.01
    assert loaded["base"] == m["base"] and loaded["samples"][0]["x"] == m["samples"][0]["x"]
    assert rig.machine.levelling is True


def test_the_copy_is_handed_over_before_the_first_write():
    rig = Rig()
    seen = []
    rig.machine.fail_m421_at = (0, 0)
    rig.machine.hooks.append(lambda machine, command: seen.append(command) if command.startswith("G1029") else None)
    kept = []
    with pytest.raises(meshload.MeshError) as error:
        meshload.load(rig.bridge, a_map(), on_backup=lambda copy: kept.append(len(seen)))
    assert kept == [0] and error.value.restored is True


def test_restore_refuses_on_another_toolhead():
    rig = Rig()
    backup, _ = meshload.load(rig.bridge, a_map())
    rig.machine.toolhead = "3DP"
    with pytest.raises(meshload.MeshError, match="3DP toolhead"):
        meshload.restore(rig.bridge, backup, check_toolhead=True)


def test_fade_height_parser():
    assert meshload.fade_height(["echo:Fade Height 10.00"]) == 10.0
    assert meshload.fade_height(["echo:Fade Height Off"]) == 0.0
    assert meshload.fade_height(["ok"]) is None
