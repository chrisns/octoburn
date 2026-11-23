"""Build the real bridge and ESP client over the fakes."""
import logging

from lhp_pkg.esp import EspProbe
from lhp_pkg.gcode import GcodeBridge
from lhp_pkg.settings import routine_config
from fakes import FakeClock, FakeEsp, FakeSnapmaker, MachinePrinter

LOG = logging.getLogger("tests")
BASE = "http://laserprobe.test"


def is_step(command, feed=300):
    """A probing step: a G0 Z move at the probe feed (repositions use the Z feed)."""
    words = command.split()
    return (len(words) == 3 and words[0] == "G0" and words[1].startswith("Z")
            and words[2] == "F%d" % feed)


class Rig(object):
    def __init__(self, surface=None, settings=None, trigger_travel=2.5, shift=0.0, **esp_kwargs):
        self.machine = FakeSnapmaker(shift=shift)
        self.machine.z = 60.0           # parked at safe_z; tests that start higher set it
        self.printer = MachinePrinter(self.machine)
        self.bridge = GcodeBridge(self.printer, LOG)
        self.printer.bridge = self.bridge
        self.device = FakeEsp(self.machine, surface=surface, trigger_travel=trigger_travel,
                              **esp_kwargs)
        self.esp = EspProbe(BASE, 2.0, transport=self.device.transport, logger=LOG, sleep=lambda s: None)
        self.clock = FakeClock()
        # The tests were written against these stages; the shipped default now starts higher.
        raw = dict(esp_url=BASE, safe_z=60.0, z_floor=0.5, stages="0.5; 0.1, 0.6; 0.02, 0.15")
        raw.update(settings or {})
        self.config = routine_config(raw)

    def prober(self, **kwargs):
        from lhp_pkg.probe import Prober
        said = []
        if "watch" not in kwargs:
            from lhp_pkg.esp import Watchdog
            kwargs["watch"] = Watchdog(self.esp, clock=self.clock).start()
        kwargs.setdefault("sleep", self.clock.sleep)
        kwargs.setdefault("say", said.append)
        p = Prober(self.bridge, self.esp, self.config, 400.0, logger=LOG, **kwargs)
        p.said = said
        return p

    def gcode_z_commands(self):
        """Every Z the head was told to go to, in order."""
        out = []
        for command in self.machine.sent:
            for word in command.split()[1:]:
                if command.split()[0] in ("G0", "G1") and word.startswith("Z"):
                    out.append(float(word[1:]))
        return out
