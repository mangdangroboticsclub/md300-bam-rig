#!/usr/bin/env python3
"""Host driver for the BAM bridge firmware (ESP32-S3 + AT32F413 driver boards).

A standalone tester for the bridge. BAM's own driver is bam/mangdang/spi.py;
this one has the same frame format, unit conversions and BAM-shaped API
(set_goal_position / set_torque_enable / set_p_gain / read_present_*).

Wiring: servo ids 1..12, board = (id-1)//3. Connectors:
CN4 -> servos 1-3, CN5 -> 4-6, CN3 -> 7-9, CN6 -> 10-12.

Examples:
    python3 bam_bridge.py ping --servo 1                 # --port auto-detected
    python3 bam_bridge.py --port /dev/ttyUSB0 ping --servo 1
    python3 bam_bridge.py dump --servo 1
    python3 bam_bridge.py kp --servo 1 --value 30
    python3 bam_bridge.py kd --servo 1 --value 0
    python3 bam_bridge.py scale --servo 1 --from-board
    python3 bam_bridge.py sweep --servo 1 \
        --low 60 --high 240 --hold 800 --cycles 3 --hz 200 --out sweep.csv
    python3 bam_bridge.py idle --servo 1

If --port is omitted the only USB-serial port found is used; if several are
present they are listed so you can pick one.
"""
import argparse
import math
import struct
import sys
import time

import serial

MAGIC_REQ = 0x42
MAGIC_RSP = 0x62

CMD_PING = 0
CMD_SET_POS = 1
CMD_IDLE = 2
CMD_TORQUE = 3
CMD_SET_KP = 4
CMD_SET_KD = 5
CMD_GET_PARAM = 6
CMD_GET_LIVE = 7
CMD_SET_SCALE = 8
CMD_RAW_REPLY = 9
CMD_SET_PARAM = 10   # pad byte = param id, arg1|arg2 = float
CMD_GET_STATUS = 11  # val = board feedback status word

BAM_OK = 0
STATUS = {0: "OK", 1: "ERR_SERVO", 2: "ERR_ARG", 3: "ERR_BOARD",
          4: "ERR_CRC", 5: "ERR_CMD"}

# AT32 sms_config parameter ids (driver_board.h)
PARAM = {
    "reverse_position_sensor": 0, "min_position_adc": 1, "max_position_adc": 2,
    "range_position_deg": 3, "reverse_motor": 4, "kp_position": 5,
    "kd_position": 6, "kp_current": 7, "kff_current": 8,
    "max_pwm_duty_cycle": 9,
    "ki_position": 10,
    "control_mode": 11,  # 0 current loop, 1 PID->PWM
    "pwm_scale": 12,
}
PARAM_BY_ID = {v: k for k, v in PARAM.items()}

# AT32 live control-loop ids (driver_board.h)
LIVE = {
    "pos_adc": 0, "cur_adc": 1, "setpoint_pos_deg": 2, "present_pos_deg": 3,
    "error_pos_deg": 4, "max_current_mA": 5, "setpoint_cur_mA": 6,
    "present_cur_mA": 7, "error_cur_mA": 8, "pwm_duty": 9, "mode": 10,
    "loop_counter": 11,
    "temperature_c": 12,   # motor NTC, degC
}
LIVE_BY_ID = {v: k for k, v in LIVE.items()}

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

DEFAULT_SCALE_DD = 3100.0   # 310.0 deg, must equal range_position_deg * 10
DEFAULT_CUR_MA = 900        # default current cap, mA


def crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE, init 0xFFFF, poly 0x1021, MSB-first."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack_float(f: float) -> tuple:
    return struct.unpack("<HH", struct.pack("<f", f))


class BamBridge:
    """Serial link to the BAM bridge firmware. servo = 1..12."""

    def __init__(self, port: str, baud: int = 921600, timeout: float = 1.0,
                 scale_dd: float = DEFAULT_SCALE_DD):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.timeout = timeout
        self.scale_dd = scale_dd
        self._goal = {}          # last commanded dd per servo (torque enable)
        self._last_t = 0.0
        self._last_dd = None
        self._vel = 0.0
        time.sleep(0.2)          # let the firmware finish booting
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    # ---- transport -------------------------------------------------------
    def _request(self, cmd: int, servo: int = 0, arg1: int = 0,
                 arg2: int = 0, pad: int = 0) -> dict:
        head = struct.pack("<BBBBHH", MAGIC_REQ, cmd, servo, pad & 0xFF,
                           arg1 & 0xFFFF, arg2 & 0xFFFF)
        self.ser.write(head + struct.pack("<H", crc16(head)))
        # Up to two attempts: a boot-banner byte or a lost frame can desync
        # the stream once, and a resync from a fresh write always recovers.
        last_err = None
        for _ in range(2):
            try:
                return self._read_response()
            except (TimeoutError, IOError) as e:
                last_err = e
                self.ser.reset_input_buffer()
                self.ser.write(head + struct.pack("<H", crc16(head)))
        raise last_err

    def _read_response(self) -> dict:
        buf = bytearray()
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] == MAGIC_RSP:
                buf += b
                break
            if b[0] == ord("#"):        # firmware text line -> skip to newline
                while time.time() < deadline:
                    c = self.ser.read(1)
                    if c in (b"", b"\n"):
                        break
        if len(buf) == 0:
            raise TimeoutError("no response (wrong port? wrong firmware?)")
        while len(buf) < 14 and time.time() < deadline:
            buf += self.ser.read(14 - len(buf))
        if len(buf) < 14:
            raise TimeoutError("short response: %d bytes" % len(buf))
        magic, cmd, servo, status, pos_dd, cur_ma, val, crc = \
            struct.unpack("<BBBBHhfH", bytes(buf))
        if crc16(bytes(buf[:12])) != crc:
            raise IOError("response CRC mismatch")
        return {"cmd": cmd, "servo": servo, "status": status,
                "pos_dd": pos_dd, "cur_mA": cur_ma, "val": val}

    def _check(self, r: dict, what: str) -> dict:
        if r["status"] != BAM_OK:
            raise IOError("%s: %s" % (what, STATUS.get(r["status"], r["status"])))
        return r

    # ---- raw firmware operations (mirror the C command set) --------------
    def ping(self, servo: int = 1) -> dict:
        """Refresh feedback; returns fresh pos_dd + cur_mA."""
        return self._check(self._request(CMD_PING, servo), "ping")

    def set_pos_dd(self, servo: int, pos_dd: int, cur_ma: int = DEFAULT_CUR_MA) -> dict:
        pos_dd = max(0, min(int(self.scale_dd), int(pos_dd)))
        return self._check(self._request(CMD_SET_POS, servo, pos_dd, cur_ma),
                           "set_pos")

    def idle(self, servo: int) -> dict:
        """Motor off (torque disable)."""
        return self._check(self._request(CMD_IDLE, servo), "idle")

    def torque(self, servo: int, cur_ma: int) -> dict:
        """Raw current mode, signed mA (spin tests)."""
        return self._check(self._request(CMD_TORQUE, servo, cur_ma & 0xFFFF),
                           "torque")

    def set_p_gain(self, servo: int, kp: float) -> dict:
        lo, hi = pack_float(float(kp))
        return self._check(self._request(CMD_SET_KP, servo, lo, hi), "set_kp")

    def set_d_gain(self, servo: int, kd: float) -> dict:
        lo, hi = pack_float(float(kd))
        return self._check(self._request(CMD_SET_KD, servo, lo, hi), "set_kd")

    def get_param(self, servo: int, param_id: int) -> dict:
        return self._check(self._request(CMD_GET_PARAM, servo, param_id),
                           "get_param")

    def get_live(self, servo: int, live_id: int) -> dict:
        return self._check(self._request(CMD_GET_LIVE, servo, live_id),
                           "get_live")
    def set_param(self, servo: int, param_id: int, value: float) -> dict:
        """Write any sms_config parameter (RAM only); val = readback."""
        lo, hi = pack_float(float(value))
        return self._check(self._request(CMD_SET_PARAM, servo, lo, hi, pad=param_id),
                           "set_param")
    def get_status(self, servo: int) -> dict:
        """Raw 16-bit status word of the servo's board feedback frame."""
        return self._check(self._request(CMD_GET_STATUS, servo), "get_status")
    @staticmethod
    def temperature_c(response: dict) -> float:
        """NTC temperature carried in the val bits of PING/SET_POS/IDLE/TORQUE replies."""
        res = struct.unpack("<I", struct.pack("<f", response["val"]))[0] & 0xFFFF
        return res / 10.0 if 0 < res < 3000 else 0.0

    def set_scale_dd(self, scale_dd: float) -> dict:
        r = self._check(self._request(CMD_SET_SCALE, 0, int(scale_dd)),
                        "set_scale")
        self.scale_dd = r["val"]
        return r

    def raw_reply(self, servo: int) -> dict:
        """Diagnostic: byte statistics of the 36 raw MISO bytes.

        Returns {'ff_count', 'b0', 'b1', 'b2'} - 36 ff_count means no board is
        driving MISO (the line is pulled up); a varying count means the line is
        floating / noisy.
        """
        r = self._check(self._request(CMD_RAW_REPLY, servo), "raw_reply")
        packed = struct.unpack("<I", struct.pack("<f", r["val"]))[0]
        return {"ff_count": packed & 0xFF,
                "b0": (packed >> 8) & 0xFF,
                "b1": (packed >> 16) & 0xFF,
                "b2": (packed >> 24) & 0xFF}

    def sync_scale_from_board(self, servo: int) -> dict:
        """Read range_position_deg off the AT32 and adopt it as host scale."""
        r = self.get_param(servo, PARAM["range_position_deg"])
        return self.set_scale_dd(r["val"] * 10.0)

    # ---- BAM-driver-shaped API (what bam/<vendor>/record.py calls) -------
    def set_goal_position(self, servo: int, rad: float,
                          cur_ma: int = DEFAULT_CUR_MA) -> dict:
        """rad, 0.0 == centre of pot travel -> firmware deci-degrees."""
        dd = int(round(rad * RAD2DEG * 10.0 + self.scale_dd / 2.0))
        self._goal[servo] = dd
        return self.set_pos_dd(servo, dd, cur_ma)

    def set_torque_enable(self, servo: int, enable: bool,
                          cur_ma: int = DEFAULT_CUR_MA) -> dict:
        if enable:
            dd = self._goal.get(servo, int(self.scale_dd / 2))
            return self.set_pos_dd(servo, dd, cur_ma)
        return self.idle(servo)

    def read_present_position(self, servo: int) -> float:
        """rad, 0.0 == centre of pot travel. 0.1 deg resolution."""
        r = self.ping(servo)
        return (r["pos_dd"] - self.scale_dd / 2.0) * 0.1 * DEG2RAD

    def read_present_velocity(self, servo: int, alpha: float = 0.2) -> float:
        """rad/s via host-side differentiation, exponential smoothing.

        No velocity field exists on the board; alpha is the per-sample EMA
        gain (0.2 default is a mild low-pass)."""
        r = self.ping(servo)
        t = time.time()
        dd = r["pos_dd"]
        if self._last_dd is not None and t > self._last_t:
            v = (dd - self._last_dd) * 0.1 * DEG2RAD / (t - self._last_t)
            self._vel += alpha * (v - self._vel)
        self._last_t, self._last_dd = t, dd
        return self._vel

    def read_present_current(self, servo: int) -> int:
        """Signed motor current, mA."""
        return self.ping(servo)["cur_mA"]

    def read_pwm_duty(self, servo: int) -> float:
        """PWM duty 0..1 (slow: 2 SPI transactions + 500 us settle)."""
        return self.get_live(servo, LIVE["pwm_duty"])["val"]

    def read_input_voltage(self) -> float:
        """No ADC on the board; rail = 12 V adapter / 3S pack."""
        return 12.0


# ---- demo: PID step sweep -> CSV ----------------------------------------
def run_sweep(b: BamBridge, servo: int, low_deg: float, high_deg: float,
              hold_ms: int, cycles: int, hz: int, cur_ma: int,
              kp: float, out):
    dt = 1.0 / hz
    prev_dd = None
    prev_t = 0.0
    if kp is not None:
        b.set_p_gain(servo, kp)
    w = out if hasattr(out, "write") else open(out, "w")
    w.write("# bam sweep servo=%d low=%.1f high=%.1f hold_ms=%d cycles=%d hz=%d cur=%d kp=%s\n"
            % (servo, low_deg, high_deg, hold_ms, cycles, hz, cur_ma, kp))
    w.write("t_ms,goal_dd,pos_dd,cur_mA,vel_rad_s\n")
    t0 = time.time()
    try:
        for _ in range(cycles):
            for tgt in (high_deg, low_deg):
                goal_dd = int(round(tgt * 10.0))
                t_end = time.time() + hold_ms / 1000.0
                while time.time() < t_end:
                    ts0 = time.time()
                    r = b.set_pos_dd(servo, goal_dd, cur_ma)
                    ts1 = time.time()
                    if prev_dd is not None and ts1 > prev_t:
                        vel = (r["pos_dd"] - prev_dd) * 0.1 * DEG2RAD / (ts1 - prev_t)
                    else:
                        vel = 0.0
                    prev_dd, prev_t = r["pos_dd"], ts1
                    w.write("%.2f,%d,%d,%d,%.4f\n"
                            % ((ts0 + ts1) / 2.0 * 1e3 - t0 * 1e3, goal_dd,
                               r["pos_dd"], r["cur_mA"], vel))
                    time.sleep(max(0.0, dt - (ts1 - ts0)))
    finally:
        b.idle(servo)
        if hasattr(w, "close") and w is not out:
            w.close()


def pick_port(port):
    """Return `port` if given, else auto-detect a single USB-serial port.

    USB-serial adapters (CH340K, CP210x, FTDI) are preferred over the many
    virtual ports that sometimes show up on Linux.
    """
    if port:
        return port
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit("pyserial not installed. Run:  pip install pyserial")

    ports = list(list_ports.comports())
    # prefer real USB bridges
    usb = [p for p in ports
           if (p.vid is not None) or "USB" in (p.device or "").upper()
           or "USB" in (p.description or "").upper()]
    cands = usb or ports

    if not cands:
        sys.exit("No serial port found. Is the board plugged in? "
                 "Pass --port explicitly, e.g. --port /dev/ttyUSB0")
    if len(cands) > 1:
        print("Several serial ports found:")
        for p in cands:
            print(f"  {p.device}  ({p.description})")
        sys.exit("Re-run with --port <one of the above>, "
                 f"e.g. --port {cands[0].device}")
    return cands[0].device


# ---- CLI ------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(description="Host driver for the BAM bridge firmware")
    p.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0 "
                                   "(auto-detected if omitted)")
    p.add_argument("--baud", type=int, default=921600)
    sub = p.add_subparsers(dest="cmd", required=True)

    # Every subcommand takes an optional servo id, so `ping --servo 1` works
    # (a top-level --servo is rejected when placed after the subcommand).
    def with_servo(sp):
        sp.add_argument("--servo", type=int, default=1, help="servo id 1..12")
        return sp

    with_servo(sub.add_parser("ping", help="refresh + print feedback"))

    sp = with_servo(sub.add_parser("pos", help="position mode"))
    sp.add_argument("--deg", type=float, required=True)
    sp.add_argument("--ma", type=int, default=DEFAULT_CUR_MA, help="current cap")

    with_servo(sub.add_parser("idle", help="motor off"))

    tp = with_servo(sub.add_parser("tor", help="raw current mode (spin test)"))
    tp.add_argument("--ma", type=int, required=True, help="signed mA")

    kp = with_servo(sub.add_parser("kp", help="set kp_position"))
    kp.add_argument("--value", type=float, required=True)
    kd = with_servo(sub.add_parser("kd", help="set kd_position"))
    kd.add_argument("--value", type=float, required=True)

    g = with_servo(sub.add_parser("get", help="read one sms_config param"))
    g.add_argument("--name", choices=sorted(PARAM))
    g.add_argument("--id", type=int)

    l = with_servo(sub.add_parser("live", help="read one live value"))
    l.add_argument("--name", choices=sorted(LIVE))
    l.add_argument("--id", type=int)

    with_servo(sub.add_parser("dump", help="read all params + all live values"))

    sub.add_parser("scan", help="probe all 12 servos, report which answer")

    rd = with_servo(sub.add_parser(
        "raw", help="diagnostic: raw MISO byte stats for one servo's board"))

    sc = with_servo(sub.add_parser("scale", help="set host scale, or sync from board"))
    sc.add_argument("--value", type=float)
    sc.add_argument("--from-board", action="store_true")

    sw = with_servo(sub.add_parser("sweep", help="PID step sweep -> CSV"))
    sw.add_argument("--low", type=float, default=100.0)
    sw.add_argument("--high", type=float, default=200.0)
    sw.add_argument("--hold", type=int, default=800, help="ms per step")
    sw.add_argument("--cycles", type=int, default=3)
    sw.add_argument("--hz", type=int, default=50)
    sw.add_argument("--cur", type=int, default=DEFAULT_CUR_MA)
    sw.add_argument("--kp", type=float, default=None, help="set before sweeping")
    sw.add_argument("--out", required=True, help="CSV output file")
    return p


def main():
    args = build_parser().parse_args()
    port = pick_port(args.port)
    try:
        b = BamBridge(port, args.baud)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\n"
                 "Is 'idf.py monitor' still holding the port? Close it first.")
    try:
        if args.cmd == "ping":
            r = b.ping(args.servo)
            print("servo %d: pos_dd=%d (%.2f deg) cur=%d mA" %
                  (args.servo, r["pos_dd"], r["pos_dd"] / 10.0, r["cur_mA"]))
        elif args.cmd == "pos":
            r = b.set_pos_dd(args.servo, int(round(args.deg * 10.0)), args.ma)
            print("goal %.1f deg -> pos %.2f deg, %d mA" %
                  (args.deg, r["pos_dd"] / 10.0, r["cur_mA"]))
        elif args.cmd == "idle":
            b.idle(args.servo)
            print("servo %d idle (motor off)" % args.servo)
        elif args.cmd == "tor":
            r = b.torque(args.servo, args.ma)
            print("torque %d mA -> pos %.2f deg, %d mA" %
                  (args.ma, r["pos_dd"] / 10.0, r["cur_mA"]))
        elif args.cmd == "kp":
            r = b.set_p_gain(args.servo, args.value)
            print("servo %d kp_position = %g" % (args.servo, r["val"]))
        elif args.cmd == "kd":
            r = b.set_d_gain(args.servo, args.value)
            print("servo %d kd_position = %g" % (args.servo, r["val"]))
        elif args.cmd == "get":
            pid = args.id if args.id is not None else PARAM[args.name]
            r = b.get_param(args.servo, pid)
            print("servo %d %s = %g" % (args.servo, PARAM_BY_ID[pid], r["val"]))
        elif args.cmd == "live":
            lid = args.id if args.id is not None else LIVE[args.name]
            r = b.get_live(args.servo, lid)
            print("servo %d %s = %g" % (args.servo, LIVE_BY_ID[lid], r["val"]))
        elif args.cmd == "dump":
            print("servo %d sms_config:" % args.servo)
            for name, pid in PARAM.items():
                print("  %-24s = %g" % (name, b.get_param(args.servo, pid)["val"]))
            print("servo %d live:" % args.servo)
            for name, lid in LIVE.items():
                print("  %-24s = %g" % (name, b.get_live(args.servo, lid)["val"]))
        elif args.cmd == "scale":
            if args.from_board:
                r = b.sync_scale_from_board(args.servo)
                print("host scale set to %.1f dd (%.1f deg) from board" %
                      (b.scale_dd, b.scale_dd / 10.0))
            else:
                r = b.set_scale_dd(args.value)
                print("host scale = %.1f dd (%.1f deg)" % (b.scale_dd, b.scale_dd / 10.0))
        elif args.cmd == "sweep":
            run_sweep(b, args.servo, args.low, args.high, args.hold,
                      args.cycles, args.hz, args.cur, args.kp, args.out)
            print("sweep done -> %s" % args.out)
        elif args.cmd == "scan":
            found = []
            for sid in range(1, 13):
                try:
                    r = b.ping(sid)
                    print("  servo %2d  OK   pos_dd=%5d (%.2f deg)  cur=%d mA"
                          % (sid, r["pos_dd"], r["pos_dd"] / 10.0, r["cur_mA"]))
                    found.append(sid)
                except (IOError, TimeoutError):
                    print("  servo %2d  --   no board on this connector (board %d)"
                          % (sid, (sid - 1) // 3))
            if not found:
                print("\nNo servo boards answered. Check the SPI harness and that the\n"
                      "servo rail (12 V) is powered.")
            else:
                print("\nfound %d servo(s): %s"
                      % (len(found), ", ".join(map(str, found))))
        elif args.cmd == "raw":
            r = b.raw_reply(args.servo)
            print("servo %d (board %d) raw MISO: %d/36 bytes are 0xFF"
                  % (args.servo, (args.servo - 1) // 3, r["ff_count"]))
            print("  first bytes: %02X %02X %02X" % (r["b0"], r["b1"], r["b2"]))
            if r["ff_count"] == 36:
                print("  -> no board is driving MISO (line pulled up): "
                      "board absent or unpowered")
            elif r["ff_count"] == 0:
                print("  -> MISO is being driven low / all zeros: check wiring")
            else:
                print("  -> MISO is FLOATING or noisy (partial 0xFF): "
                      "bad connection or contention")
    except (IOError, TimeoutError) as e:
        # expected conditions (board absent, board busy) - no traceback
        sys.exit("error: %s" % e)
    finally:
        b.close()


if __name__ == "__main__":
    main()
