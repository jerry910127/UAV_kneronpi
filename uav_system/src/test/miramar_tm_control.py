#!/usr/bin/env python3
import os, time, struct, select, argparse
import termios, fcntl

READMEM_CMD  = 0x0800
READMEM_ACK  = 0x0801
WRITEMEM_CMD = 0x0802
WRITEMEM_ACK = 0x0803
PENDING_ACK  = 0x0805

TM_CONTROL = 0x2C000000  # 4 bytes

BAUD_MAP = {
    9600: termios.B9600,
    115200: termios.B115200,
}

# best-effort DTR/RTS (some CDC ACM firmware requires DTR=1 to respond)
TIOCMGET = getattr(termios, "TIOCMGET", 0x5415)
TIOCMSET = getattr(termios, "TIOCMSET", 0x5418)
TIOCM_DTR = getattr(termios, "TIOCM_DTR", 0x002)
TIOCM_RTS = getattr(termios, "TIOCM_RTS", 0x004)

def set_dtr_rts(fd, dtr=True, rts=True):
    try:
        buf = fcntl.ioctl(fd, TIOCMGET, struct.pack("I", 0))
        bits = struct.unpack("I", buf)[0]
        bits = (bits | TIOCM_DTR) if dtr else (bits & ~TIOCM_DTR)
        bits = (bits | TIOCM_RTS) if rts else (bits & ~TIOCM_RTS)
        fcntl.ioctl(fd, TIOCMSET, struct.pack("I", bits))
    except Exception:
        pass

def configure_raw(fd, baud=115200):
    attrs = termios.tcgetattr(fd)

    # iflag, oflag, cflag, lflag
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] |= (termios.CLOCAL | termios.CREAD)
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CRTSCTS if hasattr(termios, "CRTSCTS") else 0)
    attrs[3] = 0

    # cc: non-canonical read
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1  # 0.1s

    spd = BAUD_MAP.get(baud, termios.B115200)
    attrs[4] = spd  # ispeed
    attrs[5] = spd  # ospeed

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def drain(fd, seconds=0.2):
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if not r:
            continue
        try:
            _ = os.read(fd, 4096)
        except BlockingIOError:
            pass

def wait_for_ack(fd, expect_cmdid, expect_reqid, timeout_s=3.0):
    """
    Scan incoming bytes for a GenCP ACK header that matches (cmdid, reqid).
    ACK layout: status(2) cmdid(2) length(2) reqid(2) + payload(length):contentReference[oaicite:6]{index=6}
    """
    buf = bytearray()
    deadline = time.time() + timeout_s

    def try_parse():
        cmd_le = expect_cmdid.to_bytes(2, "little")
        start = 0
        while True:
            i = buf.find(cmd_le, start)
            if i < 0:
                return None
            s = i - 2
            if s >= 0 and s + 8 <= len(buf):
                status = int.from_bytes(buf[s:s+2], "little")
                cmdid  = int.from_bytes(buf[s+2:s+4], "little")
                length = int.from_bytes(buf[s+4:s+6], "little")
                reqid  = int.from_bytes(buf[s+6:s+8], "little")
                if cmdid == expect_cmdid and reqid == expect_reqid:
                    total = 8 + length
                    if s + total <= len(buf):
                        payload = bytes(buf[s+8:s+total])
                        return status, payload
            start = i + 1

    while time.time() < deadline:
        r, _, _ = select.select([fd], [], [], max(0.0, deadline - time.time()))
        if not r:
            break
        chunk = os.read(fd, 4096)
        if chunk:
            buf += chunk

            # handle PendingAck: extend deadline if present:contentReference[oaicite:7]{index=7}
            pending_le = PENDING_ACK.to_bytes(2, "little")
            ip = buf.find(pending_le)
            if ip >= 2 and ip + 8 <= len(buf):
                sp = ip - 2
                plen = int.from_bytes(buf[sp+4:sp+6], "little")
                preq = int.from_bytes(buf[sp+6:sp+8], "little")
                if plen >= 4 and sp + 8 + plen <= len(buf) and preq == expect_reqid:
                    # payload: reserved(2) + temp_timeout_ms(2)
                    temp_ms = int.from_bytes(buf[sp+10:sp+12], "little")
                    deadline = max(deadline, time.time() + temp_ms/1000.0)

            parsed = try_parse()
            if parsed is not None:
                return parsed

    raise TimeoutError("no matching ACK")

class GenICamPort:
    def __init__(self, dev, baud=115200):
        self.dev = dev
        self.baud = baud
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY)
        configure_raw(self.fd, baud=baud)
        set_dtr_rts(self.fd, True, True)
        self.reqid = 1

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass

    def read_reg(self, addr, length):
        # vendor sample uses little-endian fields & 20-byte readCMD:contentReference[oaicite:8]{index=8}
        cmd = bytearray(20)
        cmd[0:2] = (0x4000).to_bytes(2, "little")  # flags: RequestAck
        cmd[2:4] = READMEM_CMD.to_bytes(2, "little")
        cmd[4:6] = (12).to_bytes(2, "little")      # SCD length
        cmd[6:8] = (self.reqid).to_bytes(2, "little")
        cmd[8:16] = int(addr).to_bytes(8, "little")
        cmd[16:18] = (0).to_bytes(2, "little")
        cmd[18:20] = int(length).to_bytes(2, "little")

        drain(self.fd, 0.2)
        os.write(self.fd, cmd)
        status, payload = wait_for_ack(self.fd, READMEM_ACK, self.reqid, timeout_s=5.0)
        self.reqid = (self.reqid + 1) & 0xFFFF
        if status != 0:
            raise RuntimeError(f"READMEM_ACK status=0x{status:04x}")
        return payload

    def write_reg(self, addr, data_bytes: bytes):
        data_bytes = bytes(data_bytes)
        scd_len = 8 + len(data_bytes)
        cmd = bytearray(8 + scd_len)
        cmd[0:2] = (0x4000).to_bytes(2, "little")
        cmd[2:4] = WRITEMEM_CMD.to_bytes(2, "little")
        cmd[4:6] = int(scd_len).to_bytes(2, "little")
        cmd[6:8] = (self.reqid).to_bytes(2, "little")
        cmd[8:16] = int(addr).to_bytes(8, "little")
        cmd[16:] = data_bytes

        drain(self.fd, 0.2)
        os.write(self.fd, cmd)
        status, payload = wait_for_ack(self.fd, WRITEMEM_ACK, self.reqid, timeout_s=5.0)
        self.reqid = (self.reqid + 1) & 0xFFFF
        if status != 0:
            raise RuntimeError(f"WRITEMEM_ACK status=0x{status:04x}")

        # payload length may be 0 or 4 depending on capability:contentReference[oaicite:9]{index=9}
        if len(payload) == 4:
            length_written = int.from_bytes(payload[2:4], "little")
            return length_written
        return None

def try_identify(dev, baud):
    gc = GenICamPort(dev, baud)
    try:
        # doc says 0x08, 64 bytes -> "OBSIDIAN SENSORS INC.":contentReference[oaicite:10]{index=10}
        for addr in (0x08, 0x04):
            try:
                s = gc.read_reg(addr, 64)
                txt = s.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
                if "OBSIDIAN" in txt:
                    return gc, txt, addr
            except Exception:
                pass
        gc.close()
        return None, None, None
    except Exception:
        gc.close()
        return None, None, None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default=None, help="e.g. /dev/ttyACM1")
    ap.add_argument("--auto", action="store_true", help="try /dev/ttyACM0,/dev/ttyACM1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--get", action="store_true", help="print TM_CONTROL")
    ap.add_argument("--agc", type=int, choices=[0,1], default=None, help="set AGC enable (bit0)")
    ap.add_argument("--clahe", type=int, choices=[0,1], default=None, help="set CLAHE enable (bit1)")
    args = ap.parse_args()

    gc = None
    ident = None

    if args.auto:
        for dev in ("/dev/ttyACM0", "/dev/ttyACM1"):
            if not os.path.exists(dev):
                continue
            gc, ident, used_addr = try_identify(dev, args.baud)
            if gc:
                print(f"[OK] GenICam port = {dev}, ident@0x{used_addr:x} = {ident!r}")
                break
        if not gc:
            raise SystemExit("[FAIL] cannot find GenICam port on ttyACM0/1")
    else:
        if not args.dev:
            raise SystemExit("need --dev or --auto")
        gc = GenICamPort(args.dev, args.baud)

    try:
        if args.get or args.agc is not None or args.clahe is not None:
            cur = int.from_bytes(gc.read_reg(TM_CONTROL, 4), "little")
            print(f"TM_CONTROL current = 0x{cur:08x} (AGC={(cur>>0)&1}, CLAHE={(cur>>1)&1})")

            new = cur
            # TM_CONTROL bits: bit0 AGC, bit1 CLAHE; don't change other bits:contentReference[oaicite:11]{index=11}
            if args.agc is not None:
                new = (new | 0x1) if args.agc else (new & ~0x1)
            if args.clahe is not None:
                new = (new | 0x2) if args.clahe else (new & ~0x2)

            if new != cur:
                gc.write_reg(TM_CONTROL, new.to_bytes(4, "little"))
                verify = int.from_bytes(gc.read_reg(TM_CONTROL, 4), "little")
                print(f"TM_CONTROL updated = 0x{verify:08x} (AGC={(verify>>0)&1}, CLAHE={(verify>>1)&1})")
    finally:
        gc.close()

if __name__ == "__main__":
    main()
