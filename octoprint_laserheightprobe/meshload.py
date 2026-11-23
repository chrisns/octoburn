# coding=utf-8
"""Put a height map into the Snapmaker's bed mesh, and put the 3D printing mesh back.

The firmware has one mesh. It normally holds the 3D printing mesh (absolute heights, about 42
mm on a quick swap kit printer), saved in flash. Nothing here saves to flash: no M500, ever.
The laser mesh lives in RAM only, and a power cycle (which every toolhead swap needs) brings the
print mesh back from flash. The plugin still keeps a copy of what was in RAM and writes it back
on "Restore", so the machine is left as it was found without a power cycle.

load():
1. M1006 must name a laser toolhead. A height map has no business on a 3D printing head.
2. M420 V: the current mesh, whether levelling is on, and the fade height. That is the copy
   restore() writes back.
3. G1029 P<grid>: the firmware's grid of that size. It refills every node with a default and
   turns levelling on, but nothing moves until the next move command, and the next command is
   M420 S0. Its reply gives the grid's start and spacing, which must be the ones the map was
   fitted to.
4. One M421 per node, then M420 V again: every node must read back as written.
5. M420 Z0 (no fade: the laser works well above any fade height, where the correction would
   fade to nothing) and M420 S1. The reply must say "Bed Leveling ON".

If any step after 2 fails, restore() runs at once, and the error says so.

restore(): G1029 P<n> for the copy's size, M420 S0, one M421 per node, M420 V must match the
copy, then the fade height and the levelling state as they were.

G-code sent here: M1006, M420 (S0, S1, V, Z), M421, G1029 P. Plus the bridge's M400, M114, M118.
"""
from __future__ import absolute_import

import re

from . import gcode, heightmap

LASER_HEADS = ("LASER", "10W LASER", "20W LASER", "40W LASER", "2W RED LASER")
_FADE_RE = re.compile(r"Fade Height\s*:?\s*(?P<v>Off|-?\d+(?:\.\d+)?)", re.IGNORECASE)
BATCH = 11
TIMEOUT = 60.0


class MeshError(RuntimeError):
    """The mesh could not be loaded or restored safely. `restored`: the copy is back in the
    printer (True), could not be put back (False), or nothing had been written (None)."""

    def __init__(self, message, restored=None):
        RuntimeError.__init__(self, message)
        self.restored = restored


class GridMoved(MeshError):
    """G1029 placed the grid somewhere other than where the map was fitted."""

    def __init__(self, message, start, spacing):
        MeshError.__init__(self, message)
        self.start, self.spacing = start, spacing


def fade_height(lines):
    """The fade height from an M420 reply: a number, 0.0 for "Off", or None if not reported."""
    value = None
    for line in lines:
        match = _FADE_RE.search(line)
        if match is not None:
            v = match.group("v")
            value = 0.0 if v.lower() == "off" else float(v)
    return value


def read_state(bridge):
    """dict(grid=z[i][j], levelling=bool, fade=float or None) from M420 V. Raises MeshError."""
    lines = bridge.run(["M420 V"], TIMEOUT)
    grid = gcode.bilinear_grid(lines)
    if grid is None:
        raise MeshError("the printer did not report its bed mesh (M420 V)")
    return dict(grid=grid, levelling=bool(gcode.levelling(lines)), fade=fade_height(lines))


def read_toolhead(bridge):
    head = gcode.toolhead(bridge.run(["M1006"], TIMEOUT))
    if head is None:
        raise MeshError("the printer did not say which toolhead is fitted (M1006)")
    return head


def _write(bridge, grid, geometry=None):
    """G1029 P<n>, M420 S0, the nodes, and the read-back. Returns the G1029 geometry."""
    n = len(grid)
    if any(len(col) != n for col in grid):
        raise MeshError("the firmware only takes square meshes, this one is %d x %d"
                        % (n, len(grid[0])))
    lines = bridge.run(["G1029 P%d" % n, "M420 S0"], TIMEOUT)
    found = gcode.grid_geometry(lines)
    if gcode.levelling(lines) is not False:
        raise MeshError("M420 S0 did not turn levelling off")
    if found is None or found[0] != n:
        raise MeshError("G1029 P%d did not confirm a %d point grid" % (n, n))
    if geometry is not None:
        start, spacing = geometry
        if (abs(found[1][0] - start[0]) > 0.01 or abs(found[1][1] - start[1]) > 0.01
                or abs(found[2][0] - spacing[0]) > 0.01 or abs(found[2][1] - spacing[1]) > 0.01):
            raise GridMoved("the firmware's grid starts at X%g Y%g every %g x %g mm, not where the map "
                            "was fitted (X%g Y%g every %g x %g mm)"
                            % (found[1][0], found[1][1], found[2][0], found[2][1],
                               start[0], start[1], spacing[0], spacing[1]), found[1], found[2])
    lines_all = heightmap.commands(grid)
    for k in range(0, len(lines_all), BATCH):
        reply = bridge.run(lines_all[k:k + BATCH], TIMEOUT)
        bad = gcode.errors(reply)
        if bad:
            raise MeshError("the printer refused a mesh value: %s" % bad[0].strip())
    state = read_state(bridge)
    if not heightmap.same(state["grid"], grid):
        raise MeshError("the mesh did not read back as written")
    if state["levelling"]:
        raise MeshError("levelling came back on while the mesh was written")
    return found


def load(bridge, height_map, backup=None, on_backup=None):
    """Load a heightmap.build() result. Returns (backup, the map as loaded). Raises MeshError.

    backup: a copy kept from an earlier load (then the print mesh), used instead of reading the
    printer's present mesh, which is the earlier map. on_backup(copy) is called before the first
    write, so the copy is on disk even if this fails half way. If the printer places its grid
    elsewhere than the map was fitted (a work origin in force), the map is refitted to that grid
    once and written again.
    """
    head = read_toolhead(bridge)
    if head.upper() not in LASER_HEADS:
        raise MeshError("the printer reports a %s toolhead. A height map is only for a laser" % head)
    if backup is None:
        backup = read_state(bridge)
        backup["toolhead"] = head
    if on_backup is not None:
        on_backup(backup)
    try:
        try:
            _write(bridge, height_map["nodes"], (height_map["start"], height_map["spacing"]))
        except GridMoved as moved:
            height_map = heightmap.refit(height_map, list(moved.start), list(moved.spacing))
            _write(bridge, height_map["nodes"], (height_map["start"], height_map["spacing"]))
        lines = bridge.run(["M420 Z0", "M420 S1"], TIMEOUT)
        if gcode.levelling(lines) is not True or gcode.errors(lines):
            raise MeshError("M420 S1 did not turn levelling on")
    except Exception as exception:
        try:
            restore(bridge, backup)
        except Exception as again:                        # noqa: BLE001
            raise MeshError("%s, and putting the old mesh back failed too (%s). Power-cycle the "
                            "printer before printing: that reloads the print mesh from flash"
                            % (exception, again), restored=False)
        raise MeshError("%s. The old mesh is back" % exception, restored=True)
    return backup, height_map


def restore(bridge, backup, check_toolhead=False):
    """Write the copy back and set levelling and fade as they were. Raises MeshError.

    check_toolhead: refuse unless the fitted toolhead is the one the copy was taken with. The
    copy's levelling state belongs to that head: forcing a laser-mode "levelling OFF" onto a 3D
    printing head would drive the nozzle into the plate. After a swap (which needs a power
    cycle) the printer already has the print mesh from flash, so there is nothing to restore.
    """
    if check_toolhead and backup.get("toolhead"):
        head = read_toolhead(bridge)
        if head != backup["toolhead"]:
            raise MeshError("the printer now has a %s toolhead, not the %s the copy was taken with, "
                            "so the copy is not written back" % (head, backup["toolhead"]))
    _write(bridge, backup["grid"])
    commands = []
    if backup.get("fade") is not None:
        commands.append("M420 Z%g" % backup["fade"])
    commands.append("M420 S%d" % (1 if backup.get("levelling") else 0))
    lines = bridge.run(commands, TIMEOUT)
    if gcode.levelling(lines) is not bool(backup.get("levelling")):
        raise MeshError("levelling did not go back to %s" % ("ON" if backup.get("levelling") else "OFF"))
    return True


def matches(bridge, backup):
    """True when the printer's mesh is the backed-up one (after a power cycle, say)."""
    return heightmap.same(read_state(bridge)["grid"], backup["grid"])
