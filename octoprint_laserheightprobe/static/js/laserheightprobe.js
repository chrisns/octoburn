$(function () {
    var PLUGIN = "laserheightprobe";

    function errorText(response) {
        if (response && response.responseJSON && response.responseJSON.error) {
            return response.responseJSON.error;
        }
        return (response && response.statusText) || "failed";
    }

    function stamp(text) {
        var now = new Date();
        function two(v) { return (v < 10 ? "0" : "") + v; }
        return two(now.getHours()) + ":" + two(now.getMinutes()) + ":" +
               two(now.getSeconds()) + "  " + text;
    }

    function number(value) {
        if (value === null || value === undefined || value === "") return null;
        var n = parseFloat(value);
        return isNaN(n) ? null : n;
    }

    function LaserHeightProbeViewModel(parameters) {
        var self = this;

        self.loginState = parameters[0];
        self.access = parameters[1];
        self.printerState = parameters[2];

        self.running = ko.observable(false);
        self.mode = ko.observable("");
        self.message = ko.observable("");
        self.done = ko.observable(0);
        self.total = ko.observable(0);
        self.log = ko.observable("");
        self.error = ko.observable("");
        self.settingsError = ko.observable("");
        self.espUrl = ko.observable("");
        self.setupConfirmed = ko.observable(false);
        self.heightsMissing = ko.observableArray([]);
        self.safeZInput = ko.observable("");
        self.zFloorInput = ko.observable("");
        self.tapTestEnabled = ko.observable(true);
        self.waitingForTap = ko.observable(false);
        self.dxInput = ko.observable("");
        self.dyInput = ko.observable("");

        // Probe here (the panel in the Control tab)
        self.sessionOk = ko.observable(false);
        self.sessionReason = ko.observable("");
        self.panelError = ko.observable("");
        self.hereRows = ko.observableArray([]);
        self.hereText = ko.observable("");
        self.hereLast = ko.observable(null);
        self.hereCopyMessage = ko.observable("");
        self.hereMaxDescent = ko.observable(10);
        self.hereArea = ko.observable("point");
        self.hereRepeatN = ko.observable(5);
        // Flat bed: start Probe here from an object's height above the bed (like the rotary's
        // known diameter). Empty: probe from where the head was jogged.
        self.hereHeight = ko.observable("");
        self.flatBedZ = ko.observable(null);
        self.hereSquareSize = ko.observable(3);
        self.hereSquareN = ko.observable(3);
        var hereSquareLoaded = false;
        self.panelPlace = "";
        var controlTabActive = false;

        self.probeStatus = ko.observable({});
        self.checking = ko.observable(false);
        self.checkResult = ko.observable("");

        self.pointsText = ko.observable("");
        self.gridX0 = ko.observable(50);
        self.gridY0 = ko.observable(50);
        self.gridX1 = ko.observable(150);
        self.gridY1 = ko.observable(150);
        self.gridNx = ko.observable(3);
        self.gridNy = ko.observable(3);

        self.modules = ko.observableArray([]);
        self.selectedModule = ko.observable();
        self.calibration = ko.observable({});
        self.calibX = ko.observable(160);
        self.calibY = ko.observable(160);
        self.laserFocusZ = ko.observable("");
        self.laserMachineZ = null;
        self.touchZ = ko.observable("");
        self.focalDistance = ko.observable("");
        self.touchMachineZ = null;
        self.cardThickness = ko.observable(loadCard());
        self.calibrationReport = ko.observable(null);
        self.typedOffset = ko.observable("");
        self.offsetMessage = ko.observable("");
        self.offsetWarning = ko.observable(false);

        self.last = ko.observable(null);
        self.rows = ko.observableArray([]);
        self.tableText = ko.observable("");
        self.tableModule = ko.observable(null);
        self.datumText = ko.observable("");
        self.copyMessage = ko.observable("");
        self.heightMap = ko.observable(null);
        self.meshState = ko.observable({});

        // What the printer says is fitted (M1006, M114 B, M2002 S1), the bridge and the rotary.
        self.detected = ko.observable(null);
        self.bridge = ko.observable({});
        // the close-up camera
        self.closeupInfo = ko.observable({});
        self.purifierInfo = ko.observable({});
        self.dotInfo = ko.observable({});
        self.headSwapInfo = ko.observable({});
        self.obstacles = ko.observableArray([]);
        self.obstaclesText = ko.pureComputed(function () {
            var o = self.obstacles();
            if (!o.length) return "No obstacles marked.";
            return o.length + " obstacle" + (o.length === 1 ? "" : "s") + ": " + o.map(function (b) {
                return (b.name || "object") + " " + b.top + " mm at X" + Math.round(b.box[0]) + "-" + Math.round(b.box[2]) +
                    " Y" + Math.round(b.box[1]) + "-" + Math.round(b.box[3]);
            }).join("; ");
        });
        self.headSwapMessage = ko.observable("");
        self.headSwapReady = ko.pureComputed(function () {
            var w = self.headSwapInfo() || {};
            return !(w.mesh_loaded || w.thermal === "disabled");
        });
        self.headSwapText = ko.pureComputed(function () {
            var w = self.headSwapInfo() || {}, parts = [];
            if (w.mesh_loaded) parts.push("the laser height map is still loaded instead of the print mesh");
            if (w.thermal === "disabled") parts.push("ThermalRunaway is off (it was turned off for the laser)");
            var text = parts.join("; ") + ". Heating is refused until both are fixed.";
            return text.charAt(0).toUpperCase() + text.slice(1);
        });
        self.doorInfo = ko.observable(null);
        self.doorOpen = ko.pureComputed(function () { var d = self.doorInfo(); return !!(d && d.state === "open"); });
        self.dotMessage = ko.observable("");
        self.dotBusy = ko.observable(false);
        self.purifierMessage = ko.observable("");
        self.closeupBox = ko.observable(null);          // {x0, y0, x1, y1} in LightBurn mm
        self.closeupHeight = ko.observable("0");
        self.closeupStandoff = ko.observable("");
        self.closeupPlanData = ko.observable(null);
        self.closeupT = ko.observable(Date.now());
        self.closeupMessage = ko.observable("");
        self.closeupCalibClear = ko.observable(false);
        self.closeupCalibMessage = ko.observable("");
        var closeupDrag = null;
        self.rotary = ko.observable(null);
        self.handPointer = ko.observable(null);
        self.offsetPin = ko.observable(null);
        self.offsetResult = ko.observable(null);
        self.laserMessage = ko.observable("");
        var rotaryGuessed = false;                       // prefilled once from the last measurement
        self.rotaryFrom = ko.observable(30);          // guesses: the walks find the real ends
        self.rotaryTo = ko.observable(215);
        self.rotaryChuckLimit = ko.observable("");     // blank: the rotary body (rotary_head_y_limit)
        self.rotaryKnown = ko.observable("");

        var refreshing = false;      // set while the server's state is copied in
        var positionZ = null;        // the focus Z text that came from "Use current Z"

        // The buttons that move the machine need the PROBE permission. The server refuses
        // without it either way; hiding them keeps the page honest about what this user may do.
        self.canProbe = ko.pureComputed(function () {
            return self.loginState.hasPermission(
                self.access.permissions.PLUGIN_LASERHEIGHTPROBE_PROBE);
        });

        self.readyText = ko.pureComputed(function () {
            if (!self.printerState.isOperational()) return "The printer is not connected.";
            if (self.printerState.isPrinting() || self.printerState.isPaused()) {
                return "The printer is busy with a job.";
            }
            if (!self.canProbe()) return "You do not have permission to run this.";
            if (self.settingsError()) return "Fix the settings first.";
            if (self.heightsMissing().length) return "Set safe Z and the Z floor first (Heights, above).";
            return "";
        });

        self.printerReady = ko.pureComputed(function () {
            return self.printerState.isOperational() && !self.printerState.isPrinting() &&
                   !self.printerState.isPaused();
        });

        // square, diamond, a line along X or Y: the head moves between points
        self.patternArea = ko.pureComputed(function () {
            return ["square", "diamond", "x", "y"].indexOf(self.hereArea()) >= 0;
        });
        self.patternCount = ko.pureComputed(function () {
            var n = parseInt(self.hereSquareN(), 10) || 0;
            return (self.hereArea() === "x" || self.hereArea() === "y") ? n : n * n;
        });
        self.probeButtonText = ko.pureComputed(function () {
            var area = self.hereArea();
            return {square: "Probe square", diamond: "Probe diamond", x: "Probe X line", y: "Probe Y line",
                    repeat: "Probe " + self.hereRepeatN() + " times"}[area] || "Probe here";
        });

        self.canProbeHere = ko.pureComputed(function () {
            return self.canProbe() && !self.running() && self.printerReady() && self.sessionOk();
        });

        self.canStart = ko.pureComputed(function () {
            return !self.running() && self.setupConfirmed() && self.readyText() === "";
        });

        self.percent = ko.pureComputed(function () {
            return self.total() ? Math.round(100 * self.done() / self.total()) : 0;
        });

        self.yesNo = function (value) {
            return value === true ? "yes" : (value === false ? "no" : "?");
        };

        self.num = function (value, signed) {
            if (value === null || value === undefined) return "-";
            return (signed && value >= 0 ? "+" : "") + value.toFixed(3);
        };

        self.moduleLabel = function (m) {
            return m.name + (m.focus_offset === null || m.focus_offset === undefined
                ? " (not calibrated)" : " (" + self.num(m.focus_offset, true) + " mm)");
        };

        self.selectedOffsetText = ko.pureComputed(function () {
            var name = self.selectedModule(), found = null;
            self.modules().forEach(function (m) { if (m.name === name) found = m; });
            if (!found) return "";
            return found.focus_offset === null || found.focus_offset === undefined
                ? "Not calibrated yet: focus Z stays blank, LightBurn offsets still work."
                : "Focus offset " + self.num(found.focus_offset, true) + " mm.";
        });

        self.heightsText = ko.pureComputed(function () {
            var missing = self.heightsMissing();
            return missing.length ? "Not set yet: " + missing.join(", ") + ". No job runs until both are set." : "";
        });

        function selected() {
            var name = self.selectedModule(), found = null;
            self.modules().forEach(function (m) { if (m.name === name) found = m; });
            return found;
        }

        self.xyMissing = ko.pureComputed(function () {
            var m = selected();
            return !!m && (m.dx === null || m.dx === undefined);
        });

        self.hasReference = ko.pureComputed(function () {
            var c = self.calibration();
            return c && c.z_ref !== null && c.z_ref !== undefined;
        });

        self.calibrationText = ko.pureComputed(function () {
            var c = self.calibration();
            if (!self.hasReference()) return "No reference touch yet.";
            return "Reference: X" + c.x.toFixed(2) + " Y" + c.y.toFixed(2) + ", probe trigger Z" +
                   c.z_ref.toFixed(3) + (c.time ? ", " + new Date(c.time * 1000).toLocaleString() : "") + ".";
        });

        self.resultCaption = ko.pureComputed(function () {
            var r = self.last();
            if (!r) return "";
            var text = new Date(r.finished * 1000).toLocaleString() + ". ";
            if (!r.complete) text += "Incomplete: the job stopped before the last point. ";
            text += "Trigger Z is the commanded Z of the step that tripped, true trip within one " +
                    "fine step (" + r.fine_step + " mm) above it. ";
            text += self.tableModule() ? "Focus Z for " + self.tableModule() + ". " : "";
            text += "LightBurn Z offset: focus here minus focus at the datum; positive = higher.";
            return text;
        });

        // -- helpers --------------------------------------------------------

        function command(name, data) {
            return OctoPrint.simpleApiCommand(PLUGIN, name, data || {});
        }

        function fail(target) {
            return function (response) { (target || self.error)(errorText(response)); };
        }

        self.append = function (line) {
            self.log(self.log() + line + "\n");
            var pane = document.getElementById("laserheightprobe_log");
            if (pane) {
                window.setTimeout(function () { pane.scrollTop = pane.scrollHeight; }, 0);
            }
        };

        function showTable(table) {
            self.rows((table && table.rows) || []);
            self.tableText((table && table.text) || "");
            self.tableModule((table && table.module) || null);
            var datum = "";
            ((table && table.rows) || []).forEach(function (r) { if (r.is_datum) datum = String(r.index); });
            refreshing = true;
            self.datumText(datum);
            refreshing = false;
        }

        // -- the probe by hand ------------------------------------------------

        self.readProbe = function () {
            command("probe_status").done(function (status) { self.probeStatus(status || {}); })
                .fail(function (response) { self.probeStatus({error: errorText(response)}); });
        };

        self.probeButton = function (name) {
            command(name).done(self.readProbe).fail(fail());
        };

        self.checkWiring = function () {
            self.checking(true);
            self.checkResult("Testing...");
            command("check").done(function (data) {
                self.checkResult(data.ok ? "Wiring test passed." : "Wiring test failed.");
            }).fail(function (response) {
                self.checkResult("Wiring test could not run: " + errorText(response));
            }).always(function () { self.checking(false); self.readProbe(); });
        };

        // -- points -----------------------------------------------------------

        self.savePoints = function () {
            command("save_points", {text: self.pointsText()}).done(function (data) {
                self.error("");
                self.append(stamp("Saved " + data.count + " points."));
            }).fail(fail());
        };

        self.addPosition = function () {
            command("position").done(function (pos) {
                var line = pos.pin_x.toFixed(2) + " " + pos.pin_y.toFixed(2);
                var text = self.pointsText().replace(/\s+$/, "");
                self.pointsText((text ? text + "\n" : "") + line);
            }).fail(fail());
        };

        self.captureHeight = function (which) {
            command("set_height", {which: which}).done(function () { self.error(""); self.refresh(); })
                .fail(fail());
        };

        self.saveHeight = function (which, value) {
            if (number(value) === null) { self.error("Type a number, or use the current Z."); return; }
            command("set_height", {which: which, value: number(value)})
                .done(function () { self.error(""); self.refresh(); }).fail(fail());
        };

        self.saveXy = function () {
            command("set_xy_offset", {module: self.selectedModule(), dx: number(self.dxInput()),
                                      dy: number(self.dyInput())})
                .done(function () { self.error(""); self.refresh(); }).fail(fail());
        };

        self.clearXy = function () {
            command("set_xy_offset", {module: self.selectedModule(), clear: true})
                .done(function () { self.refresh(); }).fail(fail());
        };

        self.setOdMode = function () {
            if (!window.confirm("Set the probe output to open drain? This is stored in the probe. " +
                                "Only needed if the white wire measured 5 V.")) return;
            command("set_od_mode").done(function () {
                self.checkResult("Open-drain mode sent.");
            }).fail(fail());
        };

        function showModuleXy() {
            var m = selected();
            self.dxInput(m && m.dx !== null && m.dx !== undefined ? m.dx : "");
            self.dyInput(m && m.dy !== null && m.dy !== undefined ? m.dy : "");
        }

        self.makeGrid = function () {
            command("grid", {x0: self.gridX0(), y0: self.gridY0(), x1: self.gridX1(),
                             y1: self.gridY1(), nx: self.gridNx(), ny: self.gridNy()})
                .done(function (data) { self.pointsText(data.text); self.error(""); })
                .fail(fail());
        };

        self.selectedModule.subscribe(function (name) {
            showModuleXy();
            if (refreshing || !name) return;
            command("select_module", {name: name}).done(function () { self.loadTable(); })
                .fail(fail());
        });

        // -- height map -----------------------------------------------------------

        // The mesh as rows for the table: back row (highest Y index) first, shaded by height.
        self.meshRows = ko.pureComputed(function () {
            var m = self.heightMap();
            if (!m || !m.nodes) return [];
            var n = m.nodes.length, top = m.range || 1, rows = [];
            for (var j = n - 1; j >= 0; j--) {
                var row = [];
                for (var i = 0; i < n; i++) {
                    var v = m.nodes[i][j], t = Math.max(0, Math.min(1, v / top));
                    row.push({
                        text: v.toFixed(1),
                        colour: "hsl(" + Math.round(220 - 200 * t) + ", 70%, " + (v < 0 ? 92 : 80) + "%)",
                        title: "machine X" + (m.start[0] + i * m.spacing[0]) + " Y" +
                               (m.start[1] + j * m.spacing[1]) + ": " + v.toFixed(3) + " mm"
                    });
                }
                rows.push(row);
            }
            return rows;
        });

        self.meshLoadedText = ko.pureComputed(function () {
            var at = self.meshState().at;
            return at ? "since " + new Date(at * 1000).toLocaleTimeString() : "";
        });

        self.buildMap = function () {
            command("map_build").done(function (data) {
                self.error("");
                self.heightMap(data.height_map || null);
                self.refresh();
            }).fail(fail());
        };

        self.loadMap = function () {
            if (!window.confirm("Load the height map into the printer's bed mesh and turn levelling " +
                                "on?\n\nNothing moves. The print mesh is kept by the plugin and put back " +
                                "with Restore print mesh (a power cycle also brings it back). Focus " +
                                "LightBurn on the lowest point.")) return;
            command("map_load").done(function () { self.error(""); self.refresh(); }).fail(fail());
        };

        self.restoreMap = function () {
            command("map_restore").done(function () { self.error(""); self.refresh(); }).fail(fail());
        };

        // -- run ----------------------------------------------------------------

        self.start = function () {
            self.error("");
            command("start", {points: self.pointsText()}).done(function () {
                self.running(true);
                self.setupConfirmed(false);
            }).fail(fail());
        };

        self.abort = function () { command("abort"); };

        // -- results --------------------------------------------------------------

        self.loadTable = function () {
            var data = {module: self.selectedModule()};
            if (self.datumText() !== "") data.datum = parseInt(self.datumText(), 10);
            command("table", data).done(showTable).fail(fail());
        };

        self.datumText.subscribe(function () {
            if (!refreshing) self.loadTable();
        });

        function copy(text, areaId, message) {
            function fallback() {
                var area = document.getElementById(areaId);
                if (!area) return;
                area.focus();
                area.select();
                var ok = false;
                try { ok = document.execCommand("copy"); } catch (e) { ok = false; }
                message(ok ? "Copied." : "Selected: press Ctrl+C (Cmd+C) to copy.");
            }
            // navigator.clipboard exists only on HTTPS or localhost; OctoPrint is often plain HTTP.
            if (navigator.clipboard && window.isSecureContext) {
                navigator.clipboard.writeText(text).then(function () { message("Copied."); }, fallback);
            } else {
                fallback();
            }
        }

        self.copyText = function () { copy(self.tableText(), "laserheightprobe_text", self.copyMessage); };
        self.copyHereText = function () {
            copy(self.hereText(), "laserheightprobe_here_text", self.hereCopyMessage);
        };

        // -- Probe here -----------------------------------------------------------

        function showHere(here) {
            self.hereRows((here && here.rows) || []);
            self.hereText((here && here.text) || "");
            self.hereLast((here && here.last) || null);
        }

        self.probeHere = function () {
            self.panelError("");
            var area = self.hereArea(), args = {};
            if (self.patternArea()) {
                args = {square: true, shape: area, size: parseFloat(self.hereSquareSize()),
                        n: parseInt(self.hereSquareN(), 10)};
            } else if (area === "repeat") {
                args = {repeat: parseInt(self.hereRepeatN(), 10)};
            }
            var height = String(self.hereHeight() === null || self.hereHeight() === undefined ? "" : self.hereHeight()).trim();
            if (area !== "repeat" && height !== "") {
                args.height = parseFloat(height);
            }
            command("here", args).done(function () { self.running(true); })
                .fail(fail(self.panelError));
        };

        self.tapTest = function () {
            self.panelError("");
            command("tap_test").done(function () { self.running(true); })
                .fail(fail(self.panelError));
        };

        self.setHereDatum = function (index) {
            command("here_datum", {index: index}).done(self.refresh).fail(fail(self.panelError));
        };

        self.clearHere = function () {
            if (!window.confirm("Clear all Probe here results?")) return;
            command("here_clear").done(self.refresh).fail(fail(self.panelError));
        };

        self.useHereAsBed = function () {
            self.panelError("");
            command("here_as_bed", {}).done(self.refresh).fail(fail(self.panelError));
        };

        self.useHereReference = function () {
            command("here_as_reference", {}).done(function (out) {
                self.offsetMessage("Reference set from the last Probe here result: trigger Z" +
                                   out.z_ref.toFixed(3) + ".");
                self.offsetWarning(false);
                self.refresh();
            }).fail(function (response) {
                self.offsetWarning(true);
                self.offsetMessage(errorText(response));
            });
        };

        // OctoPrint has no Control-tab template type: the panel is a generic template, moved
        // into the Control tab after binding (moving a bound node keeps its bindings; moving it
        // before binding would let the control view model try to bind it). The places are
        // OctoPrint 1.11's: after #control-jog-general, before #control-jog-custom. If neither
        // is there, the plugin's sidebar slot takes it instead.
        self.placePanel = function () {
            var panel = document.getElementById("laserheightprobe_control");
            if (!panel) return;
            var control = document.getElementById("control");
            var general = document.getElementById("control-jog-general");
            var custom = document.getElementById("control-jog-custom");
            var wrapper = document.getElementById("sidebar_plugin_laserheightprobe_wrapper");
            if (control && custom && custom.parentNode) {
                custom.parentNode.insertBefore(panel, custom);
                self.panelPlace = "control";
            } else if (control && general && general.parentNode) {
                general.parentNode.insertBefore(panel, general.nextSibling);
                self.panelPlace = "control";
            } else {
                var slot = document.getElementById("laserheightprobe_sidebar_slot");
                if (slot) {
                    slot.appendChild(panel);
                    self.panelPlace = "sidebar";
                }
            }
            if (wrapper) wrapper.style.display = self.panelPlace === "sidebar" ? "" : "none";
            panel.style.display = "";
        };

        // Light polling of the probe state while the panel can be seen.
        function poll() {
            var visible = !document.hidden &&
                (self.panelPlace === "sidebar" || (self.panelPlace === "control" && controlTabActive));
            if (visible && !self.running() && self.loginState.isUser()) self.readProbe();
        }

        // -- calibration ------------------------------------------------------------

        self.calibrateProbe = function () {
            self.error("");
            command("calibrate_probe", {x: number(self.calibX()), y: number(self.calibY())})
                .done(function () { self.running(true); self.setupConfirmed(false); })
                .fail(fail());
        };

        self.useCurrentZ = function () {
            command("position").done(function (pos) {
                positionZ = pos.z.toFixed(3);
                self.laserFocusZ(positionZ);
                self.laserMachineZ = pos.machine_z;
            }).fail(fail(self.offsetMessage));
        };

        function saveOffset(data) {
            data.module = self.selectedModule();
            self.offsetWarning(false);
            command("set_offset", data).done(function (out) {
                var check = offsetCheck(out.module, out.offset);
                self.offsetWarning(!!out.warning || !!check);
                self.offsetMessage(out.warning || (out.offset === null
                    ? "Cleared the offset for " + out.module + "."
                    : "Saved: " + out.module + " focus offset " + self.num(out.offset, true) + " mm." +
                      (check ? " " + check : "")));
                self.refresh();
            }).fail(function (response) {
                self.offsetWarning(true);
                self.offsetMessage(errorText(response));
            });
        }

        self.saveFromFocus = function () {
            saveOffset({laser_focus_z: number(self.laserFocusZ()), laser_machine_z: self.laserMachineZ});
        };
        // A Z typed by hand has no machine Z to compare against: drop the one M114 gave.
        self.laserFocusZ.subscribe(function (value) {
            if (value !== positionZ) self.laserMachineZ = null;
        });
        self.saveFromTouch = function () {
            var touch = cardTouchZ();
            if (touch === null) {
                self.offsetWarning(true);
                self.offsetMessage("Enter the Z where the card drags and the card thickness (0 if the hood touches the bare surface).");
                return;
            }
            var data = {touch_z: touch, focal_distance: number(self.focalDistance())};
            // machine Z of the same focus height, so the server can spot an origin change
            if (self.touchMachineZ !== null && self.touchMachineZ !== undefined) {
                data.laser_machine_z = self.touchMachineZ - parseFloat(self.cardThickness()) + data.focal_distance;
            }
            saveOffset(data);
        };
        self.saveTyped = function () { saveOffset({offset: number(self.typedOffset())}); };
        self.clearOffset = function () { saveOffset({clear: true}); };

        // -- calibration checks: card thickness, CAD comparison, re-touch -------------------
        // Kept apart from the rest so the LightBurn bridge UI can grow without touching it.
        // The same numbers and limits as calibration.py (CAD_OFFSET_10W, RETOUCH_*).

        var CAD_OFFSET_10W = 5.75, PRETRAVEL_LOW = -0.5, PRETRAVEL_HIGH = 2.5;
        var RETOUCH_WARN = 0.05, RETOUCH_REHOME = 0.5, RETOUCH_XY = 1.0;

        function loadCard() {
            try { return window.localStorage.getItem("laserheightprobe.card") || ""; }
            catch (e) { return ""; }
        }
        self.cardThickness.subscribe(function (value) {
            try { window.localStorage.setItem("laserheightprobe.card", value); } catch (e) { /* private window */ }
        });

        function given(value) {
            return value !== null && value !== undefined && String(value).trim() !== "" && !isNaN(parseFloat(value));
        }

        // The Z where the hood would touch the bare surface: card-drag Z minus the card.
        function cardTouchZ() {
            if (!given(self.touchZ()) || !given(self.cardThickness())) return null;
            var t = parseFloat(self.cardThickness());
            if (t < 0 || t > 2) return null;
            return Math.round((parseFloat(self.touchZ()) - t) * 10000) / 10000;
        }

        var touchPositionZ = null;   // the card Z text that came from "Use current Z"
        self.useCurrentTouchZ = function () {
            command("position").done(function (pos) {
                touchPositionZ = pos.z.toFixed(3);
                self.touchZ(touchPositionZ);
                self.touchMachineZ = pos.machine_z;
            }).fail(fail(self.offsetMessage));
        };
        // A Z typed by hand has no machine Z to compare against.
        self.touchZ.subscribe(function (value) {
            if (value !== touchPositionZ) self.touchMachineZ = null;
        });

        // The focal distance: 10 for a 10W (hood to focus), blank for anything else, so the
        // 10W constant is never applied to a module without one. A typed value is kept.
        var autoFocal = true;
        function defaultFocal() {
            var m = selected();
            if (!autoFocal) return;
            refreshingFocal = true;
            self.focalDistance(m && /10W/i.test(m.name) ? "10" : "");
            refreshingFocal = false;
        }
        var refreshingFocal = false;
        self.focalDistance.subscribe(function () { if (!refreshingFocal) autoFocal = false; });
        self.selectedModule.subscribe(defaultFocal);
        self.modules.subscribe(defaultFocal);

        // Any saved offset, however it was found: a negative one is the sign slip (the hood in
        // the work), and a 10W one far from the CAD is suspect. Mirrors calibration.cad_check.
        function offsetCheck(name, offset) {
            if (offset === null || offset === undefined) return "";
            if (offset <= 0) return "NEGATIVE: focus Z must be ABOVE the trigger Z, or the hood goes into the work. Check the sign.";
            if (/10W/i.test(name || "")) {
                var p = offset - CAD_OFFSET_10W;
                if (p < PRETRAVEL_LOW || p > PRETRAVEL_HIGH) {
                    return "That implies " + self.num(p, true) + " mm of pin pre-travel against the CAD +5.75 (expected " +
                        PRETRAVEL_LOW + " to " + PRETRAVEL_HIGH + "): re-check it, then confirm with a focus ladder.";
                }
            }
            return "";
        }

        self.touchPreview = ko.pureComputed(function () {
            var c = self.calibration(), touch = cardTouchZ();
            if (!self.hasReference() || touch === null || !given(self.focalDistance())) return "";
            var offset = touch + parseFloat(self.focalDistance()) - c.z_ref;
            var text = "Offset " + self.num(offset, true) + " mm (hood on the surface at Z" + touch.toFixed(3) +
                       ", focus Z" + (touch + parseFloat(self.focalDistance())).toFixed(3) + ").";
            var m = selected();
            if (m && /10W/i.test(m.name)) {
                var p = offset - CAD_OFFSET_10W;
                text += " CAD says +5.75 plus the pin pre-travel, so this implies " + self.num(p, true) + " mm of pre-travel";
                text += offset <= 0 ? ". NEGATIVE: focus is above the trigger, check the sign."
                    : (p < PRETRAVEL_LOW || p > PRETRAVEL_HIGH ? ": outside " + PRETRAVEL_LOW + " to " + PRETRAVEL_HIGH +
                       ", re-check the card and the reference, then confirm with a focus ladder." : " (plausible).");
            }
            return text;
        });

        self.touchPreviewWarning = ko.pureComputed(function () {
            return /NEGATIVE|outside/.test(self.touchPreview());
        });

        // Step 3: the probe back in the bay, Probe here over the reference spot again.
        function localRetouch() {
            var c = self.calibration(), last = self.hereLast();
            if (!self.hasReference() || !last || last.failed) return null;
            // only a result taken after the reference was set (not the reference itself)
            if (!last.time || !c.time || last.time <= c.time) return null;
            if (last.x !== undefined && c.x !== null && c.x !== undefined &&
                Math.sqrt(Math.pow(last.x - c.x, 2) + Math.pow(last.y - c.y, 2)) > RETOUCH_XY) return null;
            var d = last.z - c.z_ref;
            if (last.machine_z !== null && last.machine_z !== undefined && c.machine_z !== null &&
                c.machine_z !== undefined && Math.abs((last.machine_z - c.machine_z) - d) > 0.05) {
                return {level: "origin", message: "G-code and machine Z disagree: the work origin changed between the touches. Home (G28) without a G92 and redo the calibration."};
            }
            if (Math.abs(d) > RETOUCH_REHOME) {
                return {level: "rehome", message: "The re-touch is " + self.num(d, true) + " mm from the reference: Z steps were probably lost. Re-home with G28 X Y Z and redo the calibration from step 1."};
            }
            if (Math.abs(d) > RETOUCH_WARN) {
                return {level: "warn", message: "The re-touch is " + self.num(d, true) + " mm from the reference: the head seats differently after a swap, or the surface moved. Repeat the swap to see which."};
            }
            return {level: "ok", message: "The re-touch agrees with the reference within " + Math.abs(d).toFixed(3) + " mm."};
        }

        self.retouch = ko.pureComputed(function () {
            var r = self.calibrationReport();
            return (r && r.retouch) || localRetouch();
        });

        self.retouchText = ko.pureComputed(function () {
            var r = self.retouch();
            return r ? r.message : "No Probe here result over the reference spot since it was set.";
        });

        self.retouchBad = ko.pureComputed(function () {
            var r = self.retouch();
            return !!r && r.level !== "ok";
        });

        // The last Probe here result's own touches (a Repeat run keeps them all).
        self.repeatText = ko.pureComputed(function () {
            var r = self.calibrationReport(), s = r && r.last;
            if (!s || s.n < 3) return "";
            return "Last result: " + s.n + " touches, SD " + s.sd.toFixed(3) + " mm, range " + s.range.toFixed(3) +
                " mm, trend " + (s.slope >= 0 ? "+" : "") + s.slope.toFixed(4) + " mm per touch.";
        });

        self.swapText = ko.pureComputed(function () {
            var r = self.calibrationReport(), s = r && r.swaps;
            if (!s) return "";
            return s.seatings + " Probe here results over the reference spot (" + s.touches + " touches; swap the head " +
                "between them for a swap figure): between results SD " +
                (s.between_sd === null ? "-" : s.between_sd.toFixed(3)) + " mm, range " + s.between_range.toFixed(3) +
                " mm; within a result SD " + (s.within_sd === null ? "-" : s.within_sd.toFixed(3)) + " mm.";
        });

        // -- state from the server ------------------------------------------------------

        self.refresh = function () {
            OctoPrint.simpleApiGet(PLUGIN).done(function (data) {
                refreshing = true;
                try {
                    self.running(!!data.running);
                    self.mode(data.mode || "");
                    self.log((data.log || []).join("\n") + (data.log && data.log.length ? "\n" : ""));
                    self.error(data.error ? "Failed: " + data.error : "");
                    self.settingsError(data.settings_error || "");
                    self.espUrl(data.esp_url || "");
                    if (!self.running()) self.pointsText(data.points || "");
                    self.modules(data.modules || []);
                    self.selectedModule(data.selected_module);
                    self.calibration(data.calibration || {});
                    self.calibrationReport(data.calibration_report || null);
                    var h = data.heights || {};
                    self.heightsMissing(h.missing || []);
                    self.safeZInput(h.safe_z === null || h.safe_z === undefined ? "" : h.safe_z);
                    self.zFloorInput(h.z_floor === null || h.z_floor === undefined ? "" : h.z_floor);
                    self.tapTestEnabled(data.tap_test !== false);
                    self.waitingForTap(!!(data.progress && data.progress.waiting_for_tap) && !!data.running);
                    showModuleXy();
                    var session = data.session || {};
                    self.sessionOk(!!session.ok);
                    self.sessionReason(session.reason || "");
                    self.flatBedZ(data.flat_bed_z === undefined ? null : data.flat_bed_z);
                    if (data.here_max_descent !== undefined && data.here_max_descent !== null) {
                        self.hereMaxDescent(data.here_max_descent);
                    }
                    // the saved defaults, once; after that the panel keeps what the user typed
                    if (!hereSquareLoaded && data.here_square_size !== undefined && data.here_square_size !== null) {
                        self.hereSquareSize(data.here_square_size);
                        self.hereSquareN(data.here_square_n);
                        hereSquareLoaded = true;
                    }
                    if (data.calibration && data.calibration.x !== null && data.calibration.x !== undefined) {
                        self.calibX(data.calibration.x);
                        self.calibY(data.calibration.y);
                    }
                    self.last(data.last || null);
                    self.heightMap(data.height_map || null);
                    self.meshState(data.mesh || {});
                    self.detected(data.detected || null);
                    self.bridge(data.bridge || {});
                    var before = (self.closeupInfo().last || {}).time;
                    self.closeupInfo(data.closeup || {});
                    self.purifierInfo(data.purifier || {});
                    self.dotInfo(data.dot || {});
                    self.headSwapInfo(data.swap || {});
                    self.obstacles(data.obstacles || []);
                    self.flatFocus(data.flat_focus || {});
                    self.uiReady(data.ready || []);
                    self.uiAutoFrame(!!data.focus_after_frame);
                    self.uiAutoMode(data.auto_mode || {});
                    self.uiHomedHead(data.homed_head || null);
                    if (!autoHeightLoaded && data.auto_mode && data.auto_mode.flat_height != null) {
                        self.uiFocusHeight(String(data.auto_mode.flat_height));
                        autoHeightLoaded = true;
                    }
                    self.burnHistory(data.burns || {});
                    self.doorInfo(data.door || null);
                    if (data.closeup && data.closeup.last && data.closeup.last.time !== before) self.closeupT(Date.now());
                    self.rotary(data.rotary || null);
                    var rs = data.rotary || {};
                    if (!rotaryGuessed && rs.body_from_y != null && rs.body_to_y != null) {
                        self.rotaryFrom(Math.round(rs.body_from_y));
                        self.rotaryTo(Math.round(rs.body_to_y));
                        rotaryGuessed = true;
                    }
                    var p = data.progress;
                    self.message(p ? p.message : "");
                    self.done(p && p.done ? p.done : 0);
                    self.total(p && p.total ? p.total : 0);
                } finally {
                    refreshing = false;
                }
                showTable(data.table);
                showHere(data.here);
            });
        };

        // -- what is fitted -----------------------------------------------------------

        // Unknown (not read yet) counts as a laser, so nothing hides before the first read.
        self.isLaser = ko.pureComputed(function () {
            var d = self.detected();
            return !d || !!d.laser;
        });
        self.rotaryOn = ko.pureComputed(function () {
            var d = self.detected();
            return !!(d && d.rotary);
        });
        self.pickModule = ko.pureComputed(function () {
            var d = self.detected();
            return !d || !d.module_known;
        });
        self.detectedText = ko.pureComputed(function () {
            var d = self.detected();
            if (!d) return "Not read yet.";
            var parts = [d.toolhead || "toolhead unknown"];
            if (d.laser) parts.push(d.module ? "the " + d.module : "a laser M1006 cannot name: pick it below");
            parts.push(d.rotary ? (d.rotary_unusable ? "rotary attached but unusable" : "rotary on") : "no rotary");
            if (d.security) {
                parts.push(d.security.state === 0 ? "laser on the head (safety 0)"
                    : "laser will not fire (safety 0x" + d.security.state.toString(16) + ", roll " + d.security.roll + ")");
            }
            return parts.join(" · ");
        });
        self.detectedWarning = ko.pureComputed(function () {
            var d = self.detected();
            return !!(d && d.security && d.security.state !== 0);
        });

        self.detect = function () {
            self.laserMessage("");
            command("detect").done(function (out) { self.detected(out.detected); self.refresh(); })
                .fail(fail(self.laserMessage));
        };

        // -- pointer and beam offset (by hand, from the Control tab) ---------------------

        self.togglePointer = function () {
            self.laserMessage("");
            command("pointer", {on: !self.handPointer()}).done(function (out) { self.handPointer(out.pointer); })
                .fail(fail(self.laserMessage));
        };
        self.markPin = function () {
            self.laserMessage("");
            command("beam_offset", {step: "pin"}).done(function (out) {
                self.offsetPin(out.pin);
                self.offsetResult(null);
            }).fail(fail(self.laserMessage));
        };
        self.markBeam = function () {
            self.laserMessage("");
            command("beam_offset", {step: "beam"}).done(function (out) { self.offsetResult(out); })
                .fail(fail(self.laserMessage));
        };
        self.saveBeamOffset = function () {
            var r = self.offsetResult();
            if (!r) return;
            if (!window.confirm("Save dx " + r.dx.toFixed(3) + ", dy " + r.dy.toFixed(3) + " mm as the " +
                                r.module + "'s beam offset?")) return;
            command("set_xy_offset", {module: r.module, dx: r.dx, dy: r.dy}).done(function () {
                self.laserMessage("Saved the " + r.module + " beam offset.");
                self.refresh();
            }).fail(fail(self.laserMessage));
        };

        // -- rotary and the LightBurn bridge (tab) ------------------------------------------

        self.rotaryDiameterText = ko.pureComputed(function () {
            var r = self.rotary();
            if (!r || !r.diameter) return "";
            return r.diameter.toFixed(2) + " mm" + (r.diameter_measured ? " (measured)" : " (typed in)");
        });
        self.rotaryRunoutText = ko.pureComputed(function () {
            var r = self.rotary();
            if (!r || r.runout_swing === null || r.runout_swing === undefined) return "";
            var where = (r.runout_swing_by_y || []).map(function (t) {
                return "Y" + Math.round(t[0]) + " " + t[1].toFixed(1);
            }).join(", ");
            var tol = (r.runout_tolerance || 3).toFixed(1);
            return "Runout: the top swings " + r.runout_swing.toFixed(1) + " mm per turn (" + where + "). " +
                (r.runout_within ? "Within the " + tol + " mm tolerance: accepted, and rotary mode follows it in Z."
                                 : "Over the " + tol + " mm tolerance: reseat the object truer in the chuck, or focus drifts by up to " +
                                   (r.runout_swing / 2).toFixed(1) + " mm either way.");
        });
        self.measureRotary = function () {
            var last = self.hereLast();
            if (!last) { self.laserMessage("Probe here on the top of the object first (Control tab)."); return; }
            var from = number(self.rotaryFrom()), to = number(self.rotaryTo());
            if (from === null || to === null || to - from < 20) {
                self.laserMessage("Guess where the body runs (free end and chuck end), at least 20 mm apart.");
                return;
            }
            var params = {axis_x: last.x, crown: last.z, free_end_guess: from, chuck_end_guess: to,
                          y_chuck_limit: number(self.rotaryChuckLimit())};
            if (number(self.rotaryKnown()) !== null) params.known_diameter = number(self.rotaryKnown());
            if (!window.confirm("Measure the object in the chuck? The head touches the rotary housing past " +
                                "the soft Y limit first, then the object inside Y" + from + " to Y" + to + ", walking out to find its ends.")) return;
            command("rotary", {params: params}).done(function () { self.running(true); })
                .fail(fail(self.laserMessage));
        };
        self.downloadTemplate = function () {
            command("rotary_template").done(function (out) {
                var blob = new Blob([out.content], {type: "application/xml"});
                var link = document.createElement("a");
                link.href = URL.createObjectURL(blob);
                link.download = out.filename;
                document.body.appendChild(link);
                link.click();
                window.setTimeout(function () { URL.revokeObjectURL(link.href); link.remove(); }, 1000);
                self.laserMessage("Saved " + out.filename + ": rotary diameter " + out.summary.diameter.toFixed(2) +
                                  ", " + out.summary.touches + " touches.");
            }).fail(fail(self.laserMessage));
        };
        self.rotaryTurnText = ko.observable("");
        self.rotaryTurn = function (how) {
            self.rotaryTurnText("Turning...");
            command("rotary_turn", how).done(function (out) {
                self.rotaryTurnText("B" + out.b.toFixed(1) + ", head at Z" + out.z.toFixed(1) + ".");
            }).fail(fail(self.rotaryTurnText));
        };
        // -- the close-up camera --------------------------------------------
        self.closeupSrc = ko.pureComputed(function () {
            return "plugin/laserheightprobe/closeup_preview.jpg?t=" + self.closeupT();
        });
        self.closeupUrl = ko.pureComputed(function () {
            return window.location.protocol + "//" + window.location.host + "/plugin/laserheightprobe/closeup.mjpg";
        });
        self.closeupText = ko.pureComputed(function () {
            var c = self.closeupInfo() || {};
            if (c.available === false) return "numpy and OpenCV are not installed in OctoPrint's Python.";
            if (!c.model) return "Not calibrated yet.";
            var text = "Calibrated (camera " + c.model.t.map(function (v) { return v.toFixed(1); }).join(", ") + " mm from the beam).";
            var last = c.last;
            if (last && last.tiles) {
                text += " Last close-up: " + last.tiles + " tile" + (last.tiles === 1 ? "" : "s") + " at height " + last.height + " mm";
                text += last.seam_mm !== null && last.seam_mm !== undefined ? ", tiles agree within " + last.seam_mm.toFixed(2) + " mm." : ".";
                if (last.photos_time) {
                    var mins = Math.round((Date.now() / 1000 - last.photos_time) / 60);
                    text += " Photos taken " + (mins < 1 ? "just now" : mins + " min ago") + ".";
                    if (last.overview_time && last.overview_time > last.photos_time + 5)
                        text += " The overview is newer than the close-up photos: take the close-up again if anything under the box moved.";
                }
            }
            return text;
        });
        self.closeupStartText = ko.pureComputed(function () {
            var h = parseFloat(self.closeupHeight());
            return "Take close-up at height " + (isNaN(h) ? "?" : h.toFixed(1)) + " mm" + (h === 0 ? " (flat)" : "");
        });
        self.closeupRectAttr = function (r) {
            if (!r) return {};
            var x0 = Math.min(r[0], r[2]), x1 = Math.max(r[0], r[2]);
            var y0 = Math.min(r[1], r[3]), y1 = Math.max(r[1], r[3]);
            return {x: x0, y: 350 - y1, width: x1 - x0, height: y1 - y0};
        };
        function closeupPoint(event) {
            var box = event.currentTarget.getBoundingClientRect();
            return {x: Math.max(0, Math.min(320, (event.clientX - box.left) / box.width * 320)),
                    y: Math.max(0, Math.min(350, 350 - (event.clientY - box.top) / box.height * 350))};
        }
        self.closeupDragStart = function (data, event) {
            var p = closeupPoint(event);
            closeupDrag = p;
            self.closeupPlanData(null);
            self.closeupBox({x0: p.x, y0: p.y, x1: p.x, y1: p.y});
            return false;
        };
        self.closeupDragMove = function (data, event) {
            if (!closeupDrag) return true;
            var p = closeupPoint(event);
            self.closeupBox({x0: closeupDrag.x, y0: closeupDrag.y, x1: p.x, y1: p.y});
            return false;
        };
        self.closeupDragEnd = function () {
            if (!closeupDrag) return true;
            closeupDrag = null;
            var b = self.closeupBox();
            if (b && (Math.abs(b.x1 - b.x0) < 2 || Math.abs(b.y1 - b.y0) < 2)) self.closeupBox(null);
            return true;
        };
        function closeupArgs() {
            var b = self.closeupBox();
            var args = {box: [Math.min(b.x0, b.x1), Math.min(b.y0, b.y1), Math.max(b.x0, b.x1), Math.max(b.y0, b.y1)],
                        height: self.closeupHeight()};
            if (self.closeupStandoff() !== "") args.standoff = self.closeupStandoff();
            return args;
        }
        self.closeupOverview = function () {
            self.closeupMessage("Taking the overview photo...");
            command("head_photo").done(function () {
                command("closeup_clear").done(function () {
                    self.closeupMessage("Overview taken.");
                    self.closeupT(Date.now());
                }).fail(function (r) {
                    self.closeupMessage("Overview taken (" + errorText(r) + ").");
                });
            }).fail(fail(self.closeupMessage));
        };
        self.closeupPlan = function () {
            command("closeup_plan", closeupArgs()).done(function (plan) {
                self.closeupPlanData(plan);
                self.closeupMessage(plan.tiles.length + " tile" + (plan.tiles.length === 1 ? "" : "s") + " at Z" +
                                    plan.head_z.toFixed(1) + ", about " + Math.round(plan.seconds) + " s. The hood stays " +
                                    plan.clearance.toFixed(0) + " mm above the height you gave.");
            }).fail(fail(self.closeupMessage));
        };
        self.closeupStart = function () {
            command("closeup", closeupArgs()).done(function (out) {
                self.closeupMessage("Close-up started: " + out.tiles + " tile" + (out.tiles === 1 ? "" : "s") +
                                    ", about " + Math.round(out.seconds) + " s.");
            }).fail(fail(self.closeupMessage));
        };
        self.closeupClear = function () {
            command("closeup_clear").done(function () {
                self.closeupPlanData(null);
                self.closeupT(Date.now());
            }).fail(fail(self.closeupMessage));
        };

        self.purifierText = ko.pureComputed(function () {
            var p = self.purifierInfo() || {};
            if (p.enabled === undefined) return "";
            if (!p.enabled) return "Off in the settings.";
            var air = "";
            if (p.pm25 !== null && p.pm25 !== undefined) {
                air = " Air: PM2.5 " + Math.round(p.pm25) + " \u00b5g/m\u00b3" +
                    (p.hcho !== null && p.hcho !== undefined ? ", formaldehyde " + Math.round(p.hcho) : "") +
                    (p.air_age_s !== null && p.air_age_s !== undefined ? " (" + Math.round(p.air_age_s) + " s ago)." : ".");
            }
            var text = p.power ? "Running at gear " + p.gear + ": " + p.reason + "." :
                (p.phase === "idle" ? "Idle (" + p.reason + ")." : p.reason + ".");
            if (p.dose_j) text += " This job: " + Math.round(p.dose_j) + " J.";
            if (!p.ha) text += " No Home Assistant token: no socket or air readings, so the fan runs on a timer.";
            var f = p.filter || {};
            if (f.lifetime) text += " Filter: " + f.lifetime + (f.lifetime === "low" ? " (replace it soon)" : "") + ".";
            var r = p.run || {};
            if (r.run_s) {
                var mins = Math.round(r.run_s / 60), g = r.by_gear || {};
                text += " Fan time " + (mins >= 60 ? Math.floor(mins / 60) + " h " + (mins % 60) + " min" : mins + " min") +
                    " (gear 1/2/3: " + [1, 2, 3].map(function (k) { return Math.round((g[k] || 0) / 60); }).join("/") + " min)" +
                    (r.since ? " since " + new Date(r.since * 1000).toLocaleDateString() : "") + ".";
            }
            if (p.error) text += " Problem: " + p.error + ".";
            return text + air;
        });
        self.dotText = ko.pureComputed(function () {
            var d = self.dotInfo() || {};
            if (!d.calibrated) return "Calibrate the red dot first (in Calibrate the close-up camera).";
            if (!d.last) return "";
            return "Last: " + d.last.height.toFixed(2) + " mm at LightBurn (" + d.last.lb[0] + ", " + d.last.lb[1] +
                "), good to about " + (d.last.accuracy || 0.3).toFixed(1) + " mm" +
                (d.last.mapped ? "." : " (outside the probed bed map).");
        });
        self.dotCentre = function () {
            var b = self.closeupBox();
            return {x: (b.x0 + b.x1) / 2, y: (b.y0 + b.y1) / 2};
        };
        self.dotMeasure = function () {
            var c = self.dotCentre();
            self.dotBusy(true);
            self.dotMessage("Measuring with the red dot (about a minute)...");
            command("dot_measure", c).done(function (r) {
                // one point only: the Height field means the tallest thing under the dashed area
                self.closeupHeight(Math.max(0, Math.ceil(r.height * 10) / 10).toFixed(1));
                self.dotMessage("Red dot: " + r.height.toFixed(2) + " mm at the box centre, put in Height. " +
                    "It is one point: if anything under the dashed area is taller, use that instead.");
            }).fail(fail(self.dotMessage)).always(function () { self.dotBusy(false); });
        };
        self.dotMapStep = ko.observable("40");
        self.dotFocus = function () {
            var c = self.dotCentre();
            self.dotBusy(true);
            self.laserMessage("");
            self.dotMessage("Focusing from the red dot (about a minute)...");
            command("dot_focus", c).done(function (r) {
                self.dotMessage("Flat mode, focus Z" + r.focus_z.toFixed(2) + ": the work is " + r.height.toFixed(2) +
                    " mm above the bed at the box centre. Arm the laser to burn.");
            }).fail(fail(self.dotMessage)).always(function () { self.dotBusy(false); });
        };
        self.dotMap = function () {
            var b = self.closeupBox();
            self.dotBusy(true);
            self.dotMessage("Mapping with the red dot (about 12 s a point)...");
            command("dot_map", {x0: b.x0, y0: b.y0, x1: b.x1, y1: b.y1, step: parseFloat(self.dotMapStep())}).done(function (r) {
                var c = r.compare || [];
                self.dotMessage("Red-dot map: " + r.points + " points" + (r.failed.length ? " (" + r.failed.length + " failed)" : "") +
                    (c.length ? ", against the old map " + Math.min.apply(null, c).toFixed(2) + " to +" + Math.max.apply(null, c).toFixed(2) + " mm" : "") +
                    " (loaded in the printer if a map was loaded; otherwise press Load map).");
            }).fail(fail(self.dotMessage)).always(function () { self.dotBusy(false); });
        };
        self.mapBed = function () {
            command("map_bed").done(function (r) {
                self.heightMap(r.height_map || null);
                self.dotMessage("Back to the bed map.");
            }).fail(fail(self.dotMessage));
        };
        self.dotCalibrate = function () {
            var c = self.dotCentre();
            self.dotBusy(true);
            self.dotMessage("Calibrating the red dot (about 3 minutes)...");
            command("dot_calibrate", {x: c.x, y: c.y, clear: true}).done(function (r) {
                self.dotMessage("Red dot calibrated: fit within " + r.model.residual_px.toFixed(2) + " px, " +
                    r.mm_per_px_z100.toFixed(3) + " mm per px at Z100.");
            }).fail(fail(self.dotMessage)).always(function () { self.dotBusy(false); });
        };
        self.obstacleMark = function () {
            var b = self.closeupBox();
            command("obstacle_add", {box: [b.x0, b.y0, b.x1, b.y1], height: parseFloat(self.closeupHeight())})
                .done(function (r) { self.obstacles(r.obstacles || []); })
                .fail(fail(self.closeupMessage || self.headSwapMessage));
        };
        self.obstacleClear = function () {
            command("obstacle_clear").done(function (r) { self.obstacles(r.obstacles || []); })
                .fail(fail(self.headSwapMessage));
        };
        self.readyToPrint = function () {
            command("ready_to_print").done(function (r) {
                self.headSwapMessage((r.done.length ? r.done.join("; ") : "Nothing to change") + "." +
                    (r.restart ? " Restart OctoPrint to apply." : ""));
            }).fail(fail(self.headSwapMessage));
        };
        self.readyForLaser = function () {
            command("ready_for_laser").done(function (r) {
                self.headSwapMessage((r.done.length ? r.done.join("; ") : "Nothing to change") + "." +
                    (r.restart ? " Restart OctoPrint to apply." : ""));
            }).fail(fail(self.headSwapMessage));
        };
        self.purifierStop = function () {
            command("purifier_stop").done(function () { self.purifierMessage(""); })
                .fail(fail(self.purifierMessage));
        };
        self.purifierTest = function () {
            self.purifierMessage("Testing for 5 s...");
            command("purifier_test", {seconds: 5}).done(function (r) {
                self.purifierMessage((r.report || []).join(" | ") || "No report from the purifier.");
            }).fail(fail(self.purifierMessage));
        };

        // -- recent burns (Burn pane) ---------------------------------------------------
        self.burnHistory = ko.observable({});
        self.burnMessage = ko.observable("");
        function burnDuration(s) {
            s = Math.round(s || 0);
            if (s < 60) return s + " s";
            var m = Math.floor(s / 60);
            return m < 60 ? m + " min " + (s % 60) + " s" : Math.floor(m / 60) + " h " + (m % 60) + " min";
        }
        function burnEnergy(j) {
            return j >= 1000 ? (j / 1000).toFixed(1) + " kJ" : Math.round(j) + " J";
        }
        function burnWhen(t) {
            if (!t) return "";
            var d = new Date(t * 1000), now = new Date();
            var time = d.toLocaleTimeString([], {hour: "2-digit", minute: "2-digit"});
            return d.toDateString() === now.toDateString() ? time
                : d.toLocaleDateString([], {day: "numeric", month: "short"}) + " " + time;
        }
        self.burnRows = ko.pureComputed(function () {
            return ((self.burnHistory() || {}).entries || []).map(function (e) {
                var c = e.closeup || {}, b = e.bounds;
                var parts = [burnDuration(e.duration_s)];
                if (e.energy_j !== null && e.energy_j !== undefined) parts.push(burnEnergy(e.energy_j));
                if (e.max_power_pct) parts.push("up to " + Math.round(e.max_power_pct) + " % power");
                if (e.mode === "rotary") parts.push("rotary");
                return {
                    id: e.id,
                    rotary: e.mode === "rotary",
                    src: e.image_url || "",
                    when: burnWhen(e.time) || e.id,
                    detail: parts.join(", "),
                    status: c.message || c.status || "",
                    failed: c.status === "refused" || c.status === "failed",
                    title: b ? "LightBurn X" + b[0].toFixed(1) + " to " + b[2].toFixed(1) + ", Y" + b[1].toFixed(1) +
                        " to " + b[3].toFixed(1) + (e.focus_z !== null && e.focus_z !== undefined
                        ? ", focus Z" + Number(e.focus_z).toFixed(2) : "") : ""
                };
            });
        });
        self.burnClear = function () {
            if (!window.confirm("Clear the list of recent burns and their close-up photos?")) return;
            command("burns_clear").done(function () { self.burnMessage(""); self.refresh(); })
                .fail(fail(self.burnMessage));
        };

        self.closeupCalibInfo = ko.pureComputed(function () {
            return (self.closeupInfo() || {}).calib || {};
        });
        self.closeupCalibReady = ko.pureComputed(function () {
            var c = self.closeupCalibInfo().candidate;
            return !!(c && c.board && c.cross);
        });
        self.closeupCalibText = ko.pureComputed(function () {
            var c = self.closeupCalibInfo().candidate;
            if (!c) return "";
            var t = "";
            if (c.board) t += "Lens: fits the board photos within " + c.board.fit_rms_px.toFixed(2) + " px; held-back photos within " +
                (c.board.holdout_rms_mm === null ? "?" : c.board.holdout_rms_mm.toFixed(2)) + " mm. ";
            if (c.cross) t += "Beam: crosses within " + c.cross.rms_mm.toFixed(2) + " mm (each left out in turn: " +
                (c.cross.loo_max_mm === null ? "?" : c.cross.loo_max_mm.toFixed(2)) + " mm at worst). ";
            if (c.board && c.cross) t += "Good if both are under about 0.3 mm.";
            return t;
        });
        self.closeupCalibCapture = function (stage) {
            self.closeupCalibMessage("");
            command("closeup_calib_capture", {stage: stage, clear: true}).done(function (out) {
                self.closeupCalibClear(false);
                self.closeupCalibMessage(out.photos + " photos started; the head comes back up at the end.");
            }).fail(fail(self.closeupCalibMessage));
        };
        self.closeupCalibSolve = function () {
            self.closeupCalibMessage("Solving...");
            command("closeup_calib_solve", {}).done(function () {
                self.closeupCalibMessage("Lens solved: now the crosses.");
                self.refresh();
            }).fail(fail(self.closeupCalibMessage));
        };
        self.closeupCalibTie = function () {
            self.closeupCalibMessage("Tying...");
            command("closeup_calib_tie", {}).done(function () {
                self.closeupCalibMessage("Tied to the beam: check the numbers, then use it.");
                self.refresh();
            }).fail(fail(self.closeupCalibMessage));
        };
        self.closeupCalibAccept = function () {
            command("closeup_calib_accept", {}).done(function () {
                self.closeupCalibMessage("The new calibration is in use.");
                self.refresh();
            }).fail(fail(self.closeupCalibMessage));
        };

        self.bridgeMode = function (mode, extra) {
            self.laserMessage("");
            command("bridge_mode", $.extend({mode: mode}, extra || {})).done(function (out) {
                self.laserMessage(out.describe || (mode === "off" ? "Bridge off." : ""));
                self.refresh();
            }).fail(fail(self.laserMessage));
        };
        self.bridgeLaser = function (on, confirmed) {
            if (on && !confirmed && !window.confirm("ARM THE LASER for LightBurn? Burns from LightBurn will fire until it " +
                                                    "expires or you disarm it.")) return;
            command("bridge_laser", {laser: on}).done(self.refresh).fail(fail(self.laserMessage));
        };
        self.bridgePointer = function (on) {
            command("bridge_pointer", {on: on}).done(self.refresh).fail(fail(self.laserMessage));
        };
        self.bridgeText = ko.pureComputed(function () {
            var b = self.bridge() || {};
            if (!b.enabled) return "The LightBurn bridge is off in the settings.";
            if (!b.mode) return "No mode: LightBurn can connect, but nothing moves.";
            var t = b.mode + " · " + (b.client ? "LightBurn connected" : "LightBurn not connected");
            if (b.laser_seconds_left) t += " · LASER ARMED " + Math.ceil(b.laser_seconds_left / 60) + " min";
            if (b.pointer) t += " · pointer " + (b.pointer.kind === "cross" ? "crosshair" : "beam S" + b.pointer.s_max);
            return t;
        });

        // -- the tab's layout: panes, the status strip, the safety bar ---------------------

        var UI_TAB_KEY = "laserheightprobe.tab";
        function uiStored() {
            try { return window.localStorage.getItem(UI_TAB_KEY) || ""; } catch (e) { return ""; }
        }
        self.uiTab = ko.observable(uiStored());
        self.uiLogOpen = ko.observable(false);
        self.uiMoreOpen = ko.observable(false);
        self.flatFocus = ko.observable({});
        var autoHeightLoaded = false;                   // the material thickness the automatic mode uses, once
        self.uiFocusMode = ko.observable("bed");        // bed (+ height), here, dot
        self.uiFocusHeight = ko.observable("0");
        self.uiArmPending = ko.observable(false);
        self.uiCopied = ko.observable("");
        self.uiRotaryCheck = ko.observable("");
        self.uiReady = ko.observableArray([]);
        self.uiAutoFrame = ko.observable(false);
        self.uiReadyBad = ko.pureComputed(function () {
            return self.uiReady().filter(function (c) { return c.level === "bad"; }).length;
        });
        self.uiReadySummary = ko.pureComputed(function () {
            var bad = self.uiReadyBad(), warn = self.uiReady().filter(function (c) { return c.level === "warn"; }).length;
            if (!self.uiReady().length) return "";
            return bad ? bad + " to fix" : (warn ? "ready, " + warn + " to check" : "ready");
        });
        self.uiCheckIcon = function (level) {
            return {ok: "fas fa-check-circle text-success", warn: "fas fa-exclamation-circle text-warning",
                    bad: "fas fa-times-circle text-error", info: "fas fa-info-circle text-info"}[level] || "far fa-circle";
        };
        self.uiFixLabel = function (fix) {
            return {detect: "Detect", ready_for_laser: "Set up for the laser", map_bed: "Back to the bed map",
                    "tab:rotary": "Open Rotary"}[fix] || "";
        };
        self.uiFix = function (fix) {
            if (fix === "detect") self.detect();
            else if (fix === "ready_for_laser") self.readyForLaser();
            else if (fix === "map_bed") self.mapBed();
            else if (fix && fix.indexOf("tab:") === 0) self.uiSelect(fix.slice(4));
        };
        // X homes ~12 mm differently with the probe on: rotary measuring needs a home made with the 10W seated
        self.uiHomedHead = ko.observable(null);
        self.uiHomedHeadText = ko.pureComputed(function () {
            var h = self.uiHomedHead();
            if (h === "laser") return "X was homed with the 10W seated: swap to the probe now without homing, measure, then swap back.";
            if (h === "probe") return "X was last homed with the probe on, so measuring is refused: fit the 10W, home, then swap to the probe without homing.";
            return "Before measuring: home with the 10W seated, then swap to the probe without homing (X homes about 12 mm differently with the probe on).";
        });
        self.uiAutoMode = ko.observable({});
        self.uiAutoNote = ko.pureComputed(function () {
            var a = self.uiAutoMode() || {}, b = self.bridge() || {};
            if (!a.enabled) return "";
            if (a.paused) return "(automatic: paused by Off)";
            return self.rotaryOn() ? "(set automatically: the rotary is plugged in)" : "(set automatically: no rotary)";
        });
        self.uiSetAutoFrame = function () {
            var value = !self.uiAutoFrame();
            command("set_option", {key: "focus_after_frame", value: value}).done(function () { self.uiAutoFrame(value); })
                .fail(fail(self.laserMessage));
            return true;
        };
        var UI_TABS = {
            burn: {label: "Burn", icon: "fas fa-fire"},
            camera: {label: "Camera", icon: "fas fa-camera"},
            surface: {label: "Surface", icon: "fas fa-mountain"},
            rotary: {label: "Rotary", icon: "fas fa-sync-alt"},
            probe: {label: "Probe", icon: "fas fa-crosshairs"},
            setup: {label: "Calibrate", icon: "fas fa-sliders-h"}
        };
        self.uiTabs = ko.pureComputed(function () {
            if (!self.isLaser()) return [];
            var ids = self.rotaryOn() ? ["burn", "rotary", "setup"] : ["burn", "camera", "surface", "probe", "setup"];
            if (!self.uiHasCamera()) ids = ids.filter(function (id) { return id !== "camera"; });
            return ids.map(function (id) { return {id: id, label: UI_TABS[id].label, icon: UI_TABS[id].icon}; });
        });
        self.uiActive = ko.pureComputed(function () {
            var ids = self.uiTabs().map(function (t) { return t.id; });
            return ids.indexOf(self.uiTab()) >= 0 ? self.uiTab() : (ids[0] || "");
        });
        self.uiShow = function (id) { return self.uiActive() === id; };
        self.uiSelect = function (id) {
            self.uiTab(id);
            try { window.localStorage.setItem(UI_TAB_KEY, id); } catch (e) { /* private window */ }
        };

        function uiClock(seconds) {
            var s = Math.max(0, Math.round(seconds || 0));
            return Math.floor(s / 60) + ":" + (s % 60 < 10 ? "0" : "") + (s % 60);
        }
        self.uiRunTitle = ko.pureComputed(function () {
            var names = {calibrate: "Calibration touch", closeup: "Close-up", rotary: "Measuring the rotary object",
                         probe: "Measuring the points", here: "Probe here", tap: "Tap test", dot: "Red dot"};
            return names[self.mode()] || "Working";
        });
        self.uiArmed = ko.pureComputed(function () { return !!(self.bridge() || {}).laser_seconds_left; });
        self.uiDryRun = ko.pureComputed(function () { return /DRY RUN/.test((self.bridge() || {}).describe || ""); });
        self.uiModeName = ko.pureComputed(function () {
            var m = (self.bridge() || {}).mode;
            return m === "flat" ? "Flat" : (m === "rotary" ? "Rotary" : "");
        });
        self.uiArmedText = ko.pureComputed(function () {
            var b = self.bridge() || {};
            return "Laser armed: " + (self.uiModeName() || "?") + " mode, " + uiClock(b.laser_seconds_left) + " left";
        });
        self.uiChips = ko.pureComputed(function () {
            var chips = [];
            if (self.uiReady().length) {
                chips.push({label: "Ready", icon: "fas fa-clipboard-check", value: self.uiReadyBad() ? self.uiReadyBad() + " to fix" : "Yes",
                            level: self.uiReadyBad() ? "bad" : "ok", title: "See Ready to burn on the Burn tab"});
            }
            var d = self.detected();
            var head = !d ? {value: "Reading", level: "off"} :
                (d.laser ? (d.security && d.security.state !== 0 ?
                    {value: (d.module || "Laser") + ", will not fire", level: "warn",
                     title: "Safety 0x" + d.security.state.toString(16) + ": the laser is not seated on the head"} :
                    {value: d.module || d.toolhead || "Laser", level: "ok", title: d.toolhead})
                 : {value: d.toolhead || "3D print head", level: "info"});
            chips.push($.extend({label: "Head", icon: "fas fa-microchip"}, head));
            var b = self.bridge() || {};
            var laser = !b.enabled ? {value: "Bridge off", level: "off", title: "Turn the LightBurn bridge on in the settings"} :
                (b.laser_seconds_left ? {value: "ARMED " + uiClock(b.laser_seconds_left), level: "bad"} :
                 (b.mode ? {value: self.uiModeName() + (self.uiDryRun() ? ", dry run" : ", safe"), level: "info"} :
                  {value: "Safe", level: "ok", title: "No mode picked: nothing LightBurn sends moves the head"}));
            chips.push($.extend({label: "Laser", icon: "fas fa-bolt"}, laser));
            chips.push({label: "LightBurn", icon: "fas fa-link", value: b.client ? "Connected" : "Not connected",
                        level: b.client ? "ok" : "off", title: b.client ? "From " + b.client : "No LightBurn connection"});
            var door = self.doorInfo();
            chips.push({label: "Door", icon: "fas fa-door-closed",
                        value: !door ? "Unknown" : (door.state === "open" ? "Open" : "Closed"),
                        level: !door ? "off" : (door.state === "open" ? (b.laser_seconds_left ? "bad" : "warn") : "ok")});
            var m = self.meshState() || {}, hm = self.heightMap() || {};
            chips.push({label: "Surface", icon: "fas fa-mountain",
                        value: m.loaded ? (hm.source === "red dot" ? "Object map" : "Bed map") : "Print mesh",
                        level: m.loaded ? (hm.source === "red dot" ? "info" : "ok") : "off",
                        title: m.loaded ? "The printer follows the loaded height map" : "No laser height map in the printer"});
            if (self.rotaryOn()) {
                var r = self.rotary();
                chips.push({label: "Rotary", icon: "fas fa-sync-alt",
                            value: r && r.stale ? "Stale" : (r && r.complete ? "\u00d8 " + (r.diameter || 0).toFixed(1) + " mm" : "Not measured"),
                            level: r && r.complete && !r.stale ? "ok" : (r && r.stale ? "bad" : "warn"),
                            title: r && r.stale ? r.stale.reason : ""});
            }
            var p = self.purifierInfo() || {};
            chips.push({label: "Air", icon: "fas fa-wind",
                        value: p.enabled === false ? "Off" : (p.power ? "Gear " + p.gear : "Idle"),
                        level: p.enabled === false ? "off" : (p.power ? "info" : "ok"),
                        title: self.purifierText()});
            return chips.map(function (c) { return $.extend({title: ""}, c); });
        });
        // the probe board answers: the probe head is (probably) in the bay
        self.uiProbeOnline = ko.pureComputed(function () {
            var p = self.probeStatus() || {};
            return p.latched !== undefined && p.latched !== null && !p.error;
        });
        self.uiChipCss = function (level) {
            return "lhp-chip lhp-chip-" + level;
        };

        // Arming: what stops it, shown on the button instead of a refusal after the click
        self.uiArmBlock = ko.pureComputed(function () {
            var b = self.bridge() || {}, d = self.detected() || {};
            if (!self.canProbe()) return "You do not have permission to arm the laser.";
            if (!b.enabled) return "The LightBurn bridge is off in the settings.";
            if (!self.printerReady()) return "The printer is not connected, or busy.";
            if (!b.mode) return "Pick a mode first.";
            if (self.doorOpen()) return "The enclosure door is open.";
            if (d.laser === false) return "No laser is fitted.";
            if (d.security && d.security.state !== 0) return "The laser is not seated on the head (safety 0x" + d.security.state.toString(16) + ").";
            if (self.running()) return "A job is running.";
            return "";
        });
        var armTimer = null;
        self.uiArm = function () {
            if (self.uiArmBlock()) return;
            if (!self.uiArmPending()) {
                self.uiArmPending(true);
                window.clearTimeout(armTimer);
                armTimer = window.setTimeout(function () { self.uiArmPending(false); }, 5000);
                return;
            }
            self.uiArmPending(false);
            self.bridgeLaser(true, true);
        };
        self.uiDisarm = function () { self.uiArmPending(false); self.bridgeLaser(false); };

        // Flat mode: where the focus comes from
        // the red dot and the camera are the 10W's: the 2W IR has neither
        self.uiHasCamera = ko.pureComputed(function () {
            var d = self.detected() || {};
            return !d.module || /10W/i.test(d.module);
        });
        self.uiFocusPreview = ko.pureComputed(function () {
            var mode = self.uiFocusMode();
            if ((mode === "frame" || mode === "dot") && !self.uiHasCamera()) {
                return {text: "The red dot is on the 10W only: with the " + ((self.detected() || {}).module || "this laser") +
                        " pick the bed height or Probe here.", bad: true};
            }
            if (mode === "bed") {
                var f = self.flatFocus() || {};
                if (f.error) return {text: f.error, bad: true};
                if (f.bed_z === undefined || f.bed_z === null) return {text: "Reading the bed's focus...", bad: false};
                var h = number(self.uiFocusHeight()) || 0;
                return {text: "Focus Z " + (f.bed_z + h).toFixed(2) + " = " + f.source + (h ? " + " + h.toFixed(1) + " mm" : ""), bad: false};
            }
            if (mode === "here") {
                var last = (self.hereRows() || []).slice(-1)[0];
                return last ? {text: "Focus from the last Probe here (it must be under an hour old).", bad: false}
                            : {text: "No Probe here result yet: use the OctoBurn panel in the Control tab.", bad: true};
            }
            if (mode === "frame") {
                var fr = (self.bridge() || {}).frame;
                if (!fr) return {text: "Press Frame in LightBurn: its outline shows up here, then OctoBurn measures its centre.", bad: true};
                if (!(self.dotInfo() || {}).calibrated) return {text: "Calibrate the red dot first (Calibrate tab).", bad: true};
                var fb = fr.bounds;
                return {text: "Frame X" + Math.round(fb[0]) + "-" + Math.round(fb[2]) + " Y" + Math.round(fb[1]) + "-" + Math.round(fb[3]) +
                        " (" + Math.round(fr.age_s / 60) + " min ago): measures its centre with the red dot (about a minute, laser off), then focuses there.", bad: false};
            }
            if (!self.closeupBox()) return {text: "Draw a box round the work on the Camera tab first.", bad: true};
            if (!(self.dotInfo() || {}).calibrated) return {text: "Calibrate the red dot first (Calibrate tab).", bad: true};
            return {text: "Measures the work at the box's centre with the red dot (about a minute, laser off), then focuses there.", bad: false};
        });
        self.uiFlatStart = function (dry) {
            var mode = self.uiFocusMode();
            if (mode === "dot") { self.dotFocus(); return; }
            if (mode === "frame") {
                self.dotBusy(true);
                self.dotMessage("Measuring the Frame's centre with the red dot (about a minute)...");
                command("focus_frame").done(function (r) {
                    self.dotMessage("Flat mode, focus Z" + r.focus_z.toFixed(2) + " from the Frame's centre. Arm the laser to burn.");
                    self.refresh();
                }).fail(fail(self.dotMessage)).always(function () { self.dotBusy(false); });
                return;
            }
            var extra = {dry_run: !!dry};
            if (mode === "bed") extra.height = number(self.uiFocusHeight()) || 0;
            self.bridgeMode("flat", extra);
        };

        self.uiOpenTab = function () {
            var link = $('#tabs a[href="#tab_plugin_laserheightprobe"]');
            if (link.length && link.tab) link.tab("show");
        };
        self.uiCopyUrl = function () {
            var text = self.closeupUrl();
            var done = function () { self.uiCopied("Copied."); window.setTimeout(function () { self.uiCopied(""); }, 2500); };
            if (navigator.clipboard && navigator.clipboard.writeText) {
                navigator.clipboard.writeText(text).then(done, function () { self.uiCopied("Select and copy it by hand."); });
            } else {
                self.uiCopied("Select and copy it by hand.");
            }
        };

        function uiAgo(t) {
            if (!t) return "";
            var m = Math.max(0, Math.round((Date.now() / 1000 - t) / 60));
            return m < 1 ? "just now" : (m < 90 ? m + " min ago" : (m < 2880 ? Math.round(m / 60) + " h ago" : Math.round(m / 1440) + " days ago"));
        }
        self.uiRotaryState = ko.pureComputed(function () {
            var r = self.rotary() || {}, lines = [];
            if (r.stale) lines.push({text: "Stale: " + r.stale.reason + ". Measure it again before a rotary burn.", tone: "text-error"});
            else if (r.complete) lines.push({text: "Measured " + uiAgo(r.finished) + ".", tone: ""});
            if (r.partial) lines.push({text: "A run stopped part way: Resume picks up from it.", tone: "muted"});
            var c = r.last_check;
            if (c && c.time) lines.push({text: "Housing checked " + uiAgo(c.time) + ": " + (c.moved ? "the rotary had moved X" + (c.dx >= 0 ? "+" : "") + c.dx.toFixed(2) + " mm." : "not moved (X" + (c.dx >= 0 ? "+" : "") + c.dx.toFixed(2) + " mm)."), tone: c.moved ? "text-error" : "text-success"});
            var v = r.last_verify;
            if (v && v.time && !r.stale) lines.push({text: "Verified " + uiAgo(v.time) + ": the top within " + v.rms.toFixed(2) + " mm rms (worst " + (v.worst.miss >= 0 ? "+" : "") + v.worst.miss.toFixed(2) + ")" + (v.sideways_worst != null ? ", crest " + (v.sideways_worst >= 0 ? "+" : "") + v.sideways_worst.toFixed(2) + " mm sideways" : "") + ".", tone: Math.abs(v.worst.miss) > 0.5 || Math.abs(v.sideways_worst || 0) > 1.0 ? "text-warning" : "text-success"});
            return lines;
        });
        function rotaryJob(name, data, what) {
            self.uiRotaryCheck("");
            command(name, data || {}).done(function () { self.running(true); self.uiRotaryCheck(what); })
                .fail(fail(self.uiRotaryCheck));
        }
        self.uiRotaryHousing = function () {
            rotaryJob("rotary_check", {}, "Checking the housing (about a minute)...");
        };
        self.uiRotaryVerify = function () {
            rotaryJob("rotary_verify", {}, "Touching the top where the measurement says it is (about 10 minutes)...");
        };
        self.uiRotaryRemeasure = function () {
            var from = number(self.rotaryFrom()), to = number(self.rotaryTo());
            var params = {seed: "last", y_chuck_limit: number(self.rotaryChuckLimit())};
            if (from !== null) params.free_end_guess = from;
            if (to !== null) params.chuck_end_guess = to;
            if (!window.confirm("Measure the object again, starting from the last measurement? The head touches the rotary " +
                                "housing past the soft Y limit first and follows it if the rotary has moved.")) return;
            rotaryJob("rotary", {params: params}, "Measuring from the last measurement (about 20 minutes)...");
        };
        self.uiRotaryResume = function () {
            rotaryJob("rotary", {params: {resume: true, seed: "last"}}, "Resuming the measurement...");
        };

        self.uiRotaryDotCheck = function () {
            self.dotBusy(true);
            self.uiRotaryCheck("Reading the object's top with the red dot at six points (about 3 minutes, laser off)...");
            command("dot_rotary_check", {points: 6}).done(function (r) {
                self.uiRotaryCheck("Red dot minus the measurement: " + (r.mean_diff >= 0 ? "+" : "") + r.mean_diff.toFixed(2) +
                    " mm on average (spread " + r.spread.toFixed(2) + " mm). Height only: it cannot see the object shifted sideways.");
            }).fail(fail(self.uiRotaryCheck)).always(function () { self.dotBusy(false); });
        };

        self.onDataUpdaterPluginMessage = function (plugin, data) {
            if (plugin !== PLUGIN || !data) return;
            if (data.type === "detected") {
                self.detected(data.detected);
                return;
            }
            if (data.type === "burns") {                       // Recent burns changed
                self.refresh();
                return;
            }
            if (data.type === "refused") {
                new PNotify({title: "Laser fitted", text: data.message, type: "error", hide: false});
                self.append(stamp(data.message));
                return;
            }
            if (data.type === "started") {
                self.running(true);
                self.mode(data.mode || "job");
                self.error("");
                self.log("");
                self.done(0);
                self.total(0);
                self.append(stamp(data.message));
            } else if (data.type === "progress") {
                self.running(true);
                self.message(data.message);
                if (data.waiting_for_tap !== undefined) self.waitingForTap(!!data.waiting_for_tap);
                if (data.waiting_for_tap) {
                    new PNotify({title: "OctoBurn", text: "Tap the probe pin with a finger now.",
                                 type: "info", hide: true});
                }
                if (data.total) {
                    self.done(data.done);
                    self.total(data.total);
                }
                self.append(stamp(data.message));
            } else if (data.type === "done" || data.type === "failed" || data.type === "aborted") {
                self.running(false);
                self.waitingForTap(false);
                if (data.mode === "here" || data.mode === "tap") {
                    self.panelError(data.type === "done" ? "" : data.message);
                    if (data.here) showHere(data.here);
                }
                if (data.type === "failed") self.error("Failed: " + data.message);
                self.append(stamp(data.type === "done" ? "Done." : (data.type === "failed"
                    ? "Failed: " + data.message : data.message)));
                new PNotify({title: "OctoBurn",
                             text: data.type === "done" ? "Done." : data.message,
                             type: data.type === "done" ? "success" : (data.type === "failed" ? "error" : "notice"),
                             hide: data.type !== "failed"});
                self.refresh();
            }
        };

        self.onBeforeBinding = function () { self.refresh(); };
        self.onUserLoggedIn = function () { self.refresh(); };
        self.onEventSettingsUpdated = function () { if (!self.running()) self.refresh(); };
        self.onTabChange = function (current) {
            controlTabActive = current === "#control";
            if (current === "#tab_plugin_laserheightprobe" || controlTabActive) self.readProbe();
        };

        self.onAllBound = function () {
            self.placePanel();
            controlTabActive = !!document.querySelector("#control.active");
            window.setInterval(poll, 3000);
        };
    }

    OCTOPRINT_VIEWMODELS.push({
        construct: LaserHeightProbeViewModel,
        dependencies: ["loginStateViewModel", "accessViewModel", "printerStateViewModel"],
        elements: ["#laserheightprobe_tab", "#laserheightprobe_control"]
    });
});
