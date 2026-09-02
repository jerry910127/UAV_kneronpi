#!/usr/bin/env python3
import os, sys, time, struct, termios, fcntl, select

PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUDS = [115200, 9600]
PROBE_ADDRS = [0x08, 0x04]   # doc says 0x08; sample probes 0x04
PROBE_LEN = 64

def baud_const(baud: int):
    m = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    return m.get(baud)

def set_dtr_rts(fd: int):
    # Best-effort: some platforms may not expose these constants
    try:
        mask = 0
        if hasattr(termios, "TIOCM_DTR"): mask |= termios.TIOCM_DTR
        if hasattr(termios, "TIOCM_RTS"): mask |= termios.TIOCM_RTS
        if mask and hasattr(termios, "TIOCMBIS"):
            fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", mask))
    except Exception:
        pass

def open_serial(path: str, baud: int):
    b = baud_const(baud)
    if b is None:
        raise RuntimeError(f"unsupported baud={baud} on this platform")

    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    attrs = termios.tcgetattr(fd)
    # iflag, oflag, cflag, lflag, ispeed, ospeed, cc
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag 8N1
    attrs[3] = 0  # lflag
    attrs[4] = b  # ispeed
    attrs[5] = b  # ospeed

    # VMIN/VTIME: non-canonical read with timeout handled by select()
    cc = attrs[6]
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    attrs[6] = cc

    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)

    set_dtr_rts(fd)
    return fd

def read_with_timeout(fd: int, want: int, timeout_s: float):
    buf = bytearray()
    end = time.time() + timeout_s
    while len(buf) < want and time.time() < end:
        r, _, _ = select.select([fd], [], [], max(0, end - time.time()))
        if not r:
            break
        try:
            chunk = os.read(fd, want - len(buf))
        except BlockingIOError:
            chunk = b""
        if chunk:
            buf += chunk
    return bytes(buf)

def build_readmem_cmd(req_id: int, addr: int, length: int) -> bytes:
    # This matches the vendor Python sample (little-endian CCD/SCD, no serial prefix)
    # flags=0x4000 (RequestAck), command_id=0x0800(ReadMem), scd_len=12
    cmd = bytearray(20)
    cmd[0:2] = (0x4000).to_bytes(2, "little")
    cmd[2:4] = (0x0800).to_bytes(2, "little")
    cmd[4:6] = (12).to_bytes(2, "little")
    cmd[6:8] = (req_id & 0xFFFF).to_bytes(2, "little")
    cmd[8:16] = (addr & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "little")
    cmd[16:18] = (0).to_bytes(2, "little")          # reserved
    cmd[18:20] = (length & 0xFFFF).to_bytes(2, "little")
    return bytes(cmd)

def try_probe(port: str, baud: int):
    fd = open_serial(port, baud)
    try:
        # some devices print banners/prompts; read a little first
        _ = read_with_timeout(fd, 256, 0.2)

        req = 1
        for a in PROBE_ADDRS:
            cmd = build_readmem_cmd(req, a, PROBE_LEN)
            req += 1
            os.write(fd, cmd)
            ack = read_with_timeout(fd, 8 + PROBE_LEN, 1.0)
            if len(ack) < 8:
                continue

            status = int.from_bytes(ack[0:2], "little", signed=False)
            cmdid  = int.from_bytes(ack[2:4], "little", signed=False)
            if status == 0 and cmdid == 0x0801 and len(ack) >= 8 + PROBE_LEN:
                payload = ack[8:8+PROBE_LEN]
                if b"OBSIDIAN" in payload:
                    return True, a, payload
        return False, None, None
    finally:
        os.close(fd)

def main():
    for p in PORTS:
        if not os.path.exists(p):
            continue
        for b in BAUDS:
            try:
                ok, addr, payload = try_probe(p, b)
                if ok:
                    print(f"[OK] GenCP port={p} baud={b} addr=0x{addr:02X}")
                    print(payload.decode("utf-8", errors="replace"))
                    return 0
                else:
                    print(f"[FAIL] {p} baud={b} (no GenCP signature)")
            except Exception as e:
                print(f"[ERR]  {p} baud={b} err={e}")
    print("[FAIL] cannot find GenCP port on ttyACM0/1")
    return 1

if __name__ == "__main__":
    sys.exit(main())
