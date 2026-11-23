"""Recent burns: what each LightBurn job burned, and a close-up of it afterwards.

BurnTracker follows the moves the GRBL bridge sends for a real burn (beam on, power above 0;
never the pointer or a dry run, whose moves reach it with the beam off) and keeps the job's
burned bounds in LightBurn mm, its times, power, feed and laser dose. Once the bridge has been
idle for idle_s after a burn, step() hands the finished job over.

BurnHistory keeps the last KEEP jobs in burns.json in the plugin's data folder, with a crop of
each job's close-up in burns/<id>.jpg.

Plain Python (numpy and OpenCV only inside crop_closeup), so it tests without OctoPrint.
"""
import json
import math
import os
import re
import threading
import time

KEEP = 50                      # entries kept in burns.json
SHOW = 8                       # entries the tab shows
IDLE_S = 10.0                  # the bridge idle this long after a burn: the job has finished
PAD_MM = 5.0                   # round the burned bounds for the close-up
S_MAX = 255.0                  # the Snapmaker's laser power scale (S0..S255)
HISTORY_FILE = "burns.json"
IMAGE_DIR = "burns"
CROP_MAX_PX = 1200             # the longest side of a saved crop
CROP_QUALITY = 85
_ID = re.compile(r"^[0-9]{8}-[0-9]{6}(?:-[0-9]{1,3})?$")


def valid_id(burn_id):
    return isinstance(burn_id, str) and bool(_ID.match(burn_id))


class BurnTracker(object):
    """The burn in progress, from the bridge's moves. Thread safe: move() runs on LightBurn's
    connection thread (inside the bridge's lock), step() on the plugin's watcher thread."""

    def __init__(self, clock=time.monotonic, wall=time.time, idle_s=IDLE_S):
        self.clock, self.wall, self.idle_s = clock, wall, idle_s
        self.lock = threading.Lock()
        self.job = None
        self.idle_since = None

    def move(self, mv, start, context):
        """One move the bridge sent. start: the LightBurn (u, v) the move began at, or None.
        Only moves with the beam on at a power above 0 count."""
        power = float(mv.get("power") or 0.0)
        if not mv.get("fire") or power <= 0:
            return
        u, v = float(mv["u"]), float(mv["v"])
        points = [(u, v)]
        if start is not None and start[0] is not None and start[1] is not None:
            points.append((float(start[0]), float(start[1])))
        minutes = float(mv.get("minutes") or 0.0)
        length = math.hypot(points[-1][0] - u, points[-1][1] - v)
        with self.lock:
            job = self.job
            if job is None:
                mode = getattr(context, "mode", None)
                area = getattr(context, "bounds", None) if mode == "flat" else None
                job = self.job = dict(
                    start=self.wall(), end=None, mode=mode, module=getattr(context, "module", None),
                    focus_z=getattr(context, "focus_z", None), area=list(area) if area else None,
                    box=None, max_power=0.0, burn_mm=0.0, burn_minutes=0.0, duty_s=0.0, moves=0)
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            box = job["box"]
            if box is None:
                job["box"] = [min(xs), min(ys), max(xs), max(ys)]
            else:
                job["box"] = [min(box[0], min(xs)), min(box[1], min(ys)), max(box[2], max(xs)), max(box[3], max(ys))]
            job["max_power"] = max(job["max_power"], power)
            job["burn_mm"] += length
            job["burn_minutes"] += minutes
            job["duty_s"] += minutes * 60.0 * min(power, S_MAX) / S_MAX
            job["moves"] += 1
            job["end"] = self.wall()
            self.idle_since = None

    def busy(self):
        with self.lock:
            return self.job is not None

    def step(self, state):
        """The bridge's state now ("Idle", "Run", ...). Returns the finished job once the bridge
        has been idle (or in an alarm) for idle_s after a burn, else None."""
        with self.lock:
            if self.job is None or state not in ("Idle", "Alarm"):
                self.idle_since = None
                return None
            now = self.clock()
            if self.idle_since is None:
                self.idle_since = now
                self.job["idle_at"] = self.wall()    # the moves done: the job's end
                return None
            if now - self.idle_since < self.idle_s:
                return None
            job, self.job, self.idle_since = self.job, None, None
            job["alarm"] = state == "Alarm"
            return job

    def drop(self):
        with self.lock:
            self.job, self.idle_since = None, None


def entry_for(job, watts):
    """A history entry (without id or close-up) from a finished job. watts: the laser module's
    power, for the dose (None if not known)."""
    start = job.get("start")
    end = max(job.get("end") or start or 0.0, job.get("idle_at") or 0.0) or start
    feed = job["burn_mm"] / job["burn_minutes"] if job.get("burn_minutes") else None
    box = job.get("box")
    return dict(
        time=start, end=end, duration_s=round(max(0.0, (end or 0.0) - (start or 0.0)), 1),
        mode=job.get("mode"), module=job.get("module"),
        bounds=[round(v, 2) for v in box] if box else None,
        energy_j=round(job["duty_s"] * watts, 1) if watts else None,
        max_power=round(job.get("max_power") or 0.0, 1),
        max_power_pct=round(100.0 * (job.get("max_power") or 0.0) / S_MAX, 1),
        feed=round(feed, 1) if feed else None,
        burn_mm=round(job.get("burn_mm") or 0.0, 1),
        focus_z=job.get("focus_z"), alarm=bool(job.get("alarm")))


def closeup_box(box, area=None, pad=PAD_MM):
    """The burned bounds padded by pad, clipped to the work area (LightBurn mm)."""
    x0, y0, x1, y1 = box
    out = [x0 - pad, y0 - pad, x1 + pad, y1 + pad]
    if area:
        ax0, ay0, ax1, ay1 = area
        out = [max(ax0, out[0]), max(ay0, out[1]), min(ax1, out[2]), min(ay1, out[3])]
    return [round(v, 2) for v in out]


def surface_height(focus_z, bed_focus):
    """The burned surface above the bed: the job's focus Z less the bare bed's, never below 0."""
    return round(max(0.0, float(focus_z) - float(bed_focus)), 2)


def crop_closeup(jpeg, dpm, box, max_px=CROP_MAX_PX, quality=CROP_QUALITY):
    """The part of the close-up image (whole bed, LightBurn coordinates, dpm px/mm) over box,
    as a JPEG no bigger than max_px on its longest side."""
    from . import compositor, cvio
    img = cvio.decode(jpeg)
    h, w = img.shape[:2]
    geom = dict(dpm=float(dpm), W=w, H=h, bed_w=compositor.BED_W, bed_h=compositor.BED_H)
    i0, i1, j0, j1 = compositor.px_rect(geom, box)
    if i1 - i0 < 2 or j1 - j0 < 2:
        raise ValueError("the burned area is outside the close-up image")
    crop = img[j0:j1, i0:i1]
    longest = max(crop.shape[0], crop.shape[1])
    if longest > max_px:
        crop = cvio.shrink_to(crop, max(2, int(crop.shape[1] * max_px / float(longest))))
    return cvio.encode(crop, quality)


class BurnHistory(object):
    """burns.json and burns/<id>.jpg in folder; the last keep entries, oldest first."""

    def __init__(self, folder, keep=KEEP, logger=None):
        self.folder = folder
        self.keep = keep
        self.log = logger
        self.lock = threading.RLock()
        self._entries = None

    @property
    def path(self):
        return os.path.join(self.folder, HISTORY_FILE)

    def _image(self, burn_id):
        return os.path.join(self.folder, IMAGE_DIR, burn_id + ".jpg")

    def _load(self):
        if self._entries is None:
            try:
                with open(self.path) as handle:
                    data = json.load(handle)
                self._entries = [e for e in data if isinstance(e, dict) and valid_id(e.get("id"))] \
                    if isinstance(data, list) else []
            except (OSError, ValueError):
                self._entries = []
        return self._entries

    def _save(self):
        try:
            if not os.path.isdir(self.folder):
                os.makedirs(self.folder)
            with open(self.path + ".part", "w") as handle:
                json.dump(self._entries, handle, indent=1)
            os.replace(self.path + ".part", self.path)
        except OSError as exception:
            if self.log:
                self.log.warning("could not write %s: %s", self.path, exception)

    def _remove_image(self, burn_id):
        try:
            os.remove(self._image(burn_id))
        except OSError:
            pass

    def entries(self):
        """A copy, oldest first."""
        with self.lock:
            return [dict(e) for e in self._load()]

    def get(self, burn_id):
        with self.lock:
            for e in self._load():
                if e["id"] == burn_id:
                    return dict(e)
        return None

    def add(self, entry):
        """Store a new entry (an id is made from its time); returns it. Trims to keep."""
        with self.lock:
            entries = self._load()
            base = time.strftime("%Y%m%d-%H%M%S", time.localtime(entry.get("time") or time.time()))
            taken = set(e["id"] for e in entries)
            burn_id, n = base, 1
            while burn_id in taken:
                n += 1
                burn_id = "%s-%d" % (base, n)
            entry = dict(entry, id=burn_id)
            entries.append(entry)
            while len(entries) > self.keep:
                self._remove_image(entries.pop(0)["id"])
            self._save()
            return dict(entry)

    def update(self, burn_id, **fields):
        """Change an entry's fields; None if it has gone (cleared or trimmed)."""
        with self.lock:
            for e in self._load():
                if e["id"] == burn_id:
                    e.update(fields)
                    self._save()
                    return dict(e)
        return None

    def save_image(self, burn_id, jpeg):
        if not valid_id(burn_id):
            raise ValueError("bad burn id")
        path = self._image(burn_id)
        folder = os.path.dirname(path)
        if not os.path.isdir(folder):
            os.makedirs(folder)
        with open(path + ".part", "wb") as handle:
            handle.write(jpeg)
        os.replace(path + ".part", path)

    def image_path(self, burn_id):
        """The crop's file for a known entry, or None (never a path built from a bad id)."""
        if not valid_id(burn_id) or self.get(burn_id) is None:
            return None
        path = self._image(burn_id)
        return path if os.path.isfile(path) else None

    def clear(self):
        """Every entry and image gone; returns how many entries there were."""
        with self.lock:
            entries = self._load()
            n = len(entries)
            for e in entries:
                self._remove_image(e["id"])
            folder = os.path.join(self.folder, IMAGE_DIR)
            if os.path.isdir(folder):
                for name in os.listdir(folder):
                    if name.endswith(".jpg") or name.endswith(".part"):
                        try:
                            os.remove(os.path.join(folder, name))
                        except OSError:
                            pass
            self._entries = []
            self._save()
            return n
