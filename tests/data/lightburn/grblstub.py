# grblstub.py: a do-nothing GRBL 1.1 server on 127.0.0.1:23 that logs every byte LightBurn sends.
# It never touches a machine. Replies: banner on connect, "ok" per line, fixed answers to $ queries,
# a status report to "?" that tracks the last absolute X/Y/Z/A it was sent.
import re, socket, sys, threading, time

LOG = open(sys.argv[1] if len(sys.argv) > 1 else r"C:\OctoPrint\lbtest\grbl_capture.log", "a", buffering=1)
pos = dict(X=0.0, Y=0.0, Z=0.0, A=0.0)
state = ["Idle"]
import os
OK_DELAY = float(os.environ.get("OK_DELAY", "0"))
relative = [False]

SETTINGS = """$0=10
$1=25
$2=0
$3=0
$4=0
$5=0
$6=0
$10=1
$11=0.010
$12=0.002
$13=0
$20=0
$21=0
$22=1
$23=0
$24=25.000
$25=500.000
$26=250
$27=1.000
$30=255
$31=0
$32=1
$100=400.000
$101=400.000
$102=400.000
$103=888.890
$110=7200.000
$111=7200.000
$112=2400.000
$113=2700.000
$120=3000.000
$121=3000.000
$122=100.000
$123=3000.000
$130=320.000
$131=350.000
$132=330.000
$133=360.000"""


def log(direction, data):
    LOG.write("%.3f %s %r\n" % (time.time(), direction, data))


def status():
    return "<%s|MPos:%.3f,%.3f,%.3f,%.3f|FS:0,0>" % (state[0], pos["X"], pos["Y"], pos["Z"], pos["A"])


def handle_line(line):
    u = line.strip().upper()
    if u.startswith("$$"):
        return SETTINGS + "\nok"
    if u.startswith("$I"):
        return "[VER:1.1h.20190825:]\n[OPT:VZ,15,128]\nok"
    if u.startswith("$G"):
        return "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]\nok"
    if u.startswith("$#"):
        return "[G54:0.000,0.000,0.000,0.000]\n[G92:0.000,0.000,0.000,0.000]\n[TLO:0.000]\nok"
    if u.startswith("$J="):
        body = u[3:]
        rel = "G91" in body
        for k, v in re.findall(r"([XYZA])(-?[\d.]+)", body):
            pos[k] = pos[k] + float(v) if rel else float(v)
        return "ok"
    code = u.split(";")[0]
    if "G91" in code:
        relative[0] = True
    if "G90" in code:
        relative[0] = False
    for k, v in re.findall(r"([XYZA])(-?[\d.]+)", code):
        pos[k] = pos[k] + float(v) if relative[0] else float(v)
    return "ok"


def serve(conn):
    conn.sendall(b"\r\nGrbl 1.1h ['$' for help]\r\n")
    log("TX", "banner")
    buf = b""
    while True:
        data = conn.recv(4096)
        if not data:
            log("--", "disconnect")
            return
        log("RX", data)
        for ch in data:
            c = bytes([ch])
            if c == b"?":
                conn.sendall((status() + "\r\n").encode())
                continue
            if c in (b"!", b"~", b"\x18", b"\x85") or ch >= 0x80:
                log("RT", c)
                if c == b"\x18":
                    conn.sendall(b"\r\nGrbl 1.1h ['$' for help]\r\n")
                continue
            if c == b"\r":
                continue
            if c == b"\n":
                if not buf.strip():
                    conn.sendall(b"ok\r\n")          # GRBL answers a blank line with ok
                if buf.strip():
                    if re.match(rb"\s*G[01]", buf.strip().upper()) and OK_DELAY:
                        time.sleep(OK_DELAY)
                    reply = handle_line(buf.decode(errors="replace"))
                    conn.sendall((reply.replace("\n", "\r\n") + "\r\n").encode())
                buf = b""
                continue
            buf += c


srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", int(sys.argv[2]) if len(sys.argv) > 2 else 23))
srv.listen(1)
log("--", "listening")
while True:
    c, a = srv.accept()
    log("--", "connect %r" % (a,))
    threading.Thread(target=serve, args=(c,), daemon=True).start()
