#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

static void hexdump(const std::string& tag, const std::vector<uint8_t>& v, size_t max_bytes = 128) {
    std::fprintf(stderr, "\n[%s] %zu bytes\n", tag.c_str(), v.size());
    size_t n = std::min(v.size(), max_bytes);
    for (size_t i = 0; i < n; i++) {
        if (i % 16 == 0) std::fprintf(stderr, "%04zx : ", i);
        std::fprintf(stderr, "%02X ", v[i]);
        if (i % 16 == 15 || i == n - 1) std::fprintf(stderr, "\n");
    }
    if (v.size() > n) std::fprintf(stderr, "... (truncated)\n");
}

static std::string ascii_preview(const std::vector<uint8_t>& v, size_t max_bytes = 160) {
    std::string out;
    size_t n = std::min(v.size(), max_bytes);
    out.reserve(n);
    for (size_t i = 0; i < n; i++) {
        unsigned char c = v[i];
        if (c == '\r') out += "\\r";
        else if (c == '\n') out += "\\n\n";
        else if (std::isprint(c)) out.push_back((char)c);
        else out.push_back('.');
    }
    return out;
}

static bool read_file_first_line(const std::string& path, std::string& out) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return false;
    char buf[512];
    ssize_t r = ::read(fd, buf, sizeof(buf) - 1);
    ::close(fd);
    if (r <= 0) return false;
    buf[r] = 0;
    // trim
    std::string s(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    out = s;
    return true;
}

static void print_sysfs_info(const std::string& dev) {
    // dev like /dev/ttyACM0 -> sysfs: /sys/class/tty/ttyACM0/device/...
    std::string tty = dev;
    auto pos = tty.find_last_of('/');
    if (pos != std::string::npos) tty = tty.substr(pos + 1);

    std::string base = "/sys/class/tty/" + tty + "/device/";
    std::string v;

    std::fprintf(stderr, "\n=== SYSFS for %s ===\n", dev.c_str());
    if (read_file_first_line(base + "bInterfaceNumber", v)) std::fprintf(stderr, "bInterfaceNumber: %s\n", v.c_str());
    if (read_file_first_line(base + "interface", v)) std::fprintf(stderr, "interface(str): %s\n", v.c_str());

    // walk up a bit for idVendor/idProduct (parent usb device)
    // often at: /sys/class/tty/ttyACM0/device/../idVendor
    std::array<std::string, 6> ups = {"../", "../../", "../../../", "../../../../", "../../../../../", "../../../../../../"};
    for (const auto& up : ups) {
        std::string pvid = base + up + "idVendor";
        std::string ppid = base + up + "idProduct";
        std::string vid, pid;
        if (read_file_first_line(pvid, vid) && read_file_first_line(ppid, pid)) {
            std::fprintf(stderr, "idVendor: %s  idProduct: %s  (from %s)\n", vid.c_str(), pid.c_str(), (base + up).c_str());
            break;
        }
    }
}

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return (speed_t)0;
    }
}

static bool set_serial(int fd, int baud) {
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) return false;

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS; // disable HW flow control
#endif

    speed_t sp = baud_to_speed(baud);
    if (!sp) return false;

    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    // non-blocking read with timeout via poll, so keep VMIN/VTIME minimal
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) return false;

    // raise DTR/RTS
    int mcs = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd, TIOCMBIS, &mcs);

    tcflush(fd, TCIFLUSH);
    return true;
}

static bool write_all(int fd, const uint8_t* data, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t w = ::write(fd, data + off, len - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        off += (size_t)w;
    }
    tcdrain(fd);
    return true;
}

static std::vector<uint8_t> read_for_ms(int fd, int ms_total) {
    std::vector<uint8_t> out;
    auto start = std::chrono::steady_clock::now();

    while (true) {
        int ms_left = ms_total - (int)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (ms_left <= 0) break;

        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int pr = poll(&pfd, 1, ms_left);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (pr == 0) break;
        if (pfd.revents & POLLIN) {
            uint8_t buf[4096];
            ssize_t r = ::read(fd, buf, sizeof(buf));
            if (r > 0) {
                out.insert(out.end(), buf, buf + r);
                // keep reading until timeout
                continue;
            }
        }
        break;
    }
    return out;
}

// 16-bit one's complement checksum over 16-bit big-endian words
static uint16_t checksum16_ones_complement(const uint8_t* data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i += 2) {
        uint16_t w = (uint16_t)data[i] << 8;
        if (i + 1 < len) w |= data[i + 1];
        sum += w;
        sum = (sum & 0xFFFFu) + (sum >> 16);
    }
    return (uint16_t)(~sum & 0xFFFFu);
}

static void append_be16(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back((uint8_t)((x >> 8) & 0xFF));
    v.push_back((uint8_t)(x & 0xFF));
}
static void append_be64(std::vector<uint8_t>& v, uint64_t x) {
    for (int i = 7; i >= 0; --i) v.push_back((uint8_t)((x >> (i * 8)) & 0xFF));
}

enum class CrcMode {
    SpecNoPreamble,      // ccd_crc = csum(channel+CCD), scd_crc = csum(channel+CCD+SCD)
    SpecWithPreamble,    // include preamble bytes in checksum input
    WholePacketNoCrc,    // checksum over (preamble + 0 + 0 + channel + CCD + SCD) but excluding CRC fields
    ZeroCrc              // both CRC fields = 0
};

static std::string crc_mode_name(CrcMode m) {
    switch (m) {
        case CrcMode::SpecNoPreamble:   return "SpecNoPreamble";
        case CrcMode::SpecWithPreamble: return "SpecWithPreamble";
        case CrcMode::WholePacketNoCrc: return "WholePacketNoCrc";
        case CrcMode::ZeroCrc:          return "ZeroCrc";
    }
    return "Unknown";
}

static std::vector<uint8_t> build_readmem(uint64_t addr, uint16_t len, uint16_t req_id,
                                          uint16_t channel_id, CrcMode mode) {
    const uint16_t READMEM_CMD = 0x0800;
    const uint16_t flags = 0x4000; // RequestAck
    const uint16_t scd_len = 12;

    std::vector<uint8_t> ccd;
    append_be16(ccd, flags);
    append_be16(ccd, READMEM_CMD);
    append_be16(ccd, scd_len);
    append_be16(ccd, req_id);

    std::vector<uint8_t> scd;
    append_be64(scd, addr);
    append_be16(scd, 0);     // reserved
    append_be16(scd, len);

    std::vector<uint8_t> prefix;
    prefix.push_back(0x01);
    prefix.push_back(0x00);

    uint16_t ccd_crc = 0;
    uint16_t scd_crc = 0;

    if (mode == CrcMode::ZeroCrc) {
        ccd_crc = 0;
        scd_crc = 0;
    } else if (mode == CrcMode::SpecNoPreamble || mode == CrcMode::SpecWithPreamble) {
        std::vector<uint8_t> buf;
        if (mode == CrcMode::SpecWithPreamble) {
            buf.push_back(0x01); buf.push_back(0x00);
        }
        append_be16(buf, channel_id);
        buf.insert(buf.end(), ccd.begin(), ccd.end());
        ccd_crc = checksum16_ones_complement(buf.data(), buf.size());

        buf.insert(buf.end(), scd.begin(), scd.end());
        scd_crc = checksum16_ones_complement(buf.data(), buf.size());
    } else if (mode == CrcMode::WholePacketNoCrc) {
        // Treat CRC fields as 0 while computing checksum over whole thing
        std::vector<uint8_t> buf;
        buf.push_back(0x01); buf.push_back(0x00);
        append_be16(buf, 0); // placeholder CCD CRC
        append_be16(buf, 0); // placeholder SCD CRC
        append_be16(buf, channel_id);
        buf.insert(buf.end(), ccd.begin(), ccd.end());
        buf.insert(buf.end(), scd.begin(), scd.end());

        // Use same checksum for both fields (some devices do this); also try mirrored by caller
        uint16_t crc = checksum16_ones_complement(buf.data(), buf.size());
        ccd_crc = crc;
        scd_crc = crc;
    }

    append_be16(prefix, ccd_crc);
    append_be16(prefix, scd_crc);
    append_be16(prefix, channel_id);

    std::vector<uint8_t> pkt;
    pkt.reserve(prefix.size() + ccd.size() + scd.size());
    pkt.insert(pkt.end(), prefix.begin(), prefix.end());
    pkt.insert(pkt.end(), ccd.begin(), ccd.end());
    pkt.insert(pkt.end(), scd.begin(), scd.end());
    return pkt;
}

struct Ack {
    bool ok = false;
    uint16_t status = 0;
    uint16_t cmd_id = 0;
    uint16_t scd_length = 0;
    uint16_t req_id = 0;
    std::vector<uint8_t> payload;
    size_t frame_offset = 0;
};

static Ack try_parse_ack(const std::vector<uint8_t>& rx) {
    // Look for preamble 0x01 0x00
    for (size_t i = 0; i + 16 <= rx.size(); i++) {
        if (rx[i] != 0x01 || rx[i + 1] != 0x00) continue;

        // Serial Prefix = 8 bytes: preamble(2) + ccd_crc(2) + scd_crc(2) + channel(2)
        size_t p = i;
        size_t ccd_ack_off = p + 8;
        if (ccd_ack_off + 8 > rx.size()) continue;

        // CCD ACK: status(2) + cmd_id(2) + length(2) + req_id(2)
        uint16_t status = (rx[ccd_ack_off] << 8) | rx[ccd_ack_off + 1];
        uint16_t cmd_id = (rx[ccd_ack_off + 2] << 8) | rx[ccd_ack_off + 3];
        uint16_t len    = (rx[ccd_ack_off + 4] << 8) | rx[ccd_ack_off + 5];
        uint16_t req_id = (rx[ccd_ack_off + 6] << 8) | rx[ccd_ack_off + 7];

        size_t payload_off = ccd_ack_off + 8;
        if (payload_off + len > rx.size()) continue;

        Ack a;
        a.ok = true;
        a.status = status;
        a.cmd_id = cmd_id;
        a.scd_length = len;
        a.req_id = req_id;
        a.payload.assign(rx.begin() + payload_off, rx.begin() + payload_off + len);
        a.frame_offset = p;
        return a;
    }
    return Ack{};
}

static void passive_read_report(int fd, const std::string& port, int ms = 300) {
    tcflush(fd, TCIFLUSH);
    auto rx = read_for_ms(fd, ms);
    std::fprintf(stderr, "\n[PASSIVE] %s read %zu bytes (without TX)\n", port.c_str(), rx.size());
    if (!rx.empty()) {
        hexdump("PASSIVE_RX", rx, 160);
        std::fprintf(stderr, "[PASSIVE_ASCII]\n%s\n", ascii_preview(rx).c_str());
    }
}

int main(int argc, char** argv) {
    std::vector<std::string> ports;
    for (int i = 1; i < argc; i++) ports.push_back(argv[i]);
    if (ports.empty()) {
        ports = {"/dev/ttyACM0", "/dev/ttyACM1"};
    }

    std::vector<int> bauds = {9600, 115200, 19200, 38400, 57600, 230400, 460800, 921600};
    std::vector<CrcMode> modes = {
        CrcMode::SpecNoPreamble,
        CrcMode::SpecWithPreamble,
        CrcMode::WholePacketNoCrc,
        CrcMode::ZeroCrc,
    };

    const uint16_t channel_id = 0x0000;
    uint16_t req_id = 1;

    std::fprintf(stderr, "=== GenCP Probe Tool (ports: ");
    for (auto& p : ports) std::fprintf(stderr, "%s ", p.c_str());
    std::fprintf(stderr, ")\n");

    for (const auto& port : ports) {
        print_sysfs_info(port);

        int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            std::fprintf(stderr, "!!! open(%s) failed: %s\n", port.c_str(), std::strerror(errno));
            continue;
        }

        // Try a passive read first to see if this port is "log spam"
        passive_read_report(fd, port, 300);

        bool found = false;

        for (int baud : bauds) {
            if (!set_serial(fd, baud)) {
                std::fprintf(stderr, "\n[%s] set_serial baud=%d failed\n", port.c_str(), baud);
                continue;
            }

            std::fprintf(stderr, "\n=== TRY %s baud=%d ===\n", port.c_str(), baud);

            for (auto mode : modes) {
                // Build READMEM(0x08, 64) probe
                auto tx = build_readmem(/*addr*/0x08, /*len*/64, req_id++, channel_id, mode);

                std::fprintf(stderr, "\n-- mode=%s  TX_len=%zu\n", crc_mode_name(mode).c_str(), tx.size());
                hexdump("TX", tx, 96);

                // Flush RX before sending
                tcflush(fd, TCIFLUSH);

                if (!write_all(fd, tx.data(), tx.size())) {
                    std::fprintf(stderr, "write_all failed: %s\n", std::strerror(errno));
                    continue;
                }

                // Read response
                auto rx = read_for_ms(fd, 800);
                std::fprintf(stderr, "[RX] %zu bytes\n", rx.size());
                if (!rx.empty()) {
                    hexdump("RX", rx, 192);
                    std::fprintf(stderr, "[RX_ASCII]\n%s\n", ascii_preview(rx).c_str());
                } else {
                    std::fprintf(stderr, "(no data)\n");
                }

                // Try parse ACK
                Ack a = try_parse_ack(rx);
                if (a.ok) {
                    std::fprintf(stderr, "\n*** Parsed GenCP ACK at offset 0x%zx ***\n", a.frame_offset);
                    std::fprintf(stderr, "status=0x%04X  cmd_id=0x%04X  scd_len=%u  req_id=%u\n",
                                 a.status, a.cmd_id, a.scd_length, a.req_id);

                    // READMEM_ACK is 0x0801
                    if (a.cmd_id == 0x0801) {
                        std::fprintf(stderr, ">>> READMEM_ACK received!\n");
                    }

                    std::fprintf(stderr, "[PAYLOAD_HEX]\n");
                    hexdump("PAYLOAD", a.payload, 192);

                    // print payload as ASCII up to first NUL
                    size_t nul = 0;
                    while (nul < a.payload.size() && a.payload[nul] != 0) nul++;
                    std::string s((const char*)a.payload.data(), (const char*)a.payload.data() + nul);
                    for (auto& ch : s) if (!std::isprint((unsigned char)ch)) ch = '.';
                    std::fprintf(stderr, "[PAYLOAD_ASCII] '%s'\n", s.c_str());

                    found = true;
                    break;
                } else {
                    std::fprintf(stderr, "(no GenCP ACK parsed)\n");
                }
            }

            if (found) break;
        }

        if (!found) {
            std::fprintf(stderr, "\n### RESULT for %s: NO GenCP control channel found (with tried bauds/modes)\n", port.c_str());
        } else {
            std::fprintf(stderr, "\n### RESULT for %s: GenCP ACK FOUND (this is likely the control channel)\n", port.c_str());
        }

        ::close(fd);
    }

    std::fprintf(stderr, "\n=== Done ===\n");
    return 0;
}
