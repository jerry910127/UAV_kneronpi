#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

static void hexdump(const uint8_t* p, size_t n, size_t base = 0) {
    for (size_t i = 0; i < n; i += 16) {
        std::printf("%04zx : ", base + i);
        for (size_t j = 0; j < 16; ++j) {
            if (i + j < n) std::printf("%02X ", p[i + j]);
            else std::printf("   ");
        }
        std::printf(" ");
        for (size_t j = 0; j < 16 && i + j < n; ++j) {
            uint8_t c = p[i + j];
            std::printf("%c", (c >= 32 && c <= 126) ? c : '.');
        }
        std::printf("\n");
    }
}

// 16-bit one's complement checksum over big-endian 16-bit words
static uint16_t ones_complement_sum_be(const uint8_t* data, size_t len) {
    uint32_t sum = 0;
    size_t i = 0;
    while (i + 1 < len) {
        uint16_t w = (uint16_t(data[i]) << 8) | uint16_t(data[i + 1]);
        sum += w;
        // fold
        sum = (sum & 0xFFFFu) + (sum >> 16);
        i += 2;
    }
    if (i < len) {
        uint16_t w = (uint16_t(data[i]) << 8); // pad low byte with 0
        sum += w;
        sum = (sum & 0xFFFFu) + (sum >> 16);
    }
    // final fold
    while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
    return uint16_t(~sum) & 0xFFFFu;
}

static bool set_raw_serial(int fd, int baud) {
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) return false;

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS; // no hw flow
    tio.c_cflag &= ~PARENB;  // 8N1
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    speed_t sp = B9600;
    switch (baud) {
        case 9600: sp = B9600; break;
        case 19200: sp = B19200; break;
        case 38400: sp = B38400; break;
        case 57600: sp = B57600; break;
        case 115200: sp = B115200; break;
        case 230400: sp = B230400; break;
        case 460800: sp = B460800; break;
        case 921600: sp = B921600; break;
        default:
            std::cerr << "Unsupported baud in this build: " << baud << "\n";
            return false;
    }

    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    // VTIME/VMIN: we use select(), so keep non-blocking-ish
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN]  = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) return false;

    // Assert DTR/RTS
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0) {
        status |= (TIOCM_DTR | TIOCM_RTS);
        ioctl(fd, TIOCMSET, &status);
    }
    return true;
}

static std::vector<uint8_t> read_for_ms(int fd, int ms) {
    std::vector<uint8_t> out;
    const int chunk = 4096;
    uint8_t buf[chunk];

    int remaining = ms;
    while (remaining > 0) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = std::min(remaining, 50) * 1000; // poll in <=50ms slices
        int r = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        remaining -= std::min(remaining, 50);
        if (r > 0 && FD_ISSET(fd, &rfds)) {
            int n = (int)read(fd, buf, chunk);
            if (n > 0) out.insert(out.end(), buf, buf + n);
        }
    }
    return out;
}

static void write_all(int fd, const uint8_t* p, size_t n) {
    size_t off = 0;
    while (off < n) {
        int w = (int)write(fd, p + off, n - off);
        if (w > 0) off += (size_t)w;
        else break;
    }
    tcdrain(fd);
}

static std::vector<uint8_t> build_readmem_cmd(uint16_t flags, uint16_t request_id,
                                              bool crc_include_preamble,
                                              uint16_t channel_id = 0) {
    // Serial Prefix (Appendix A): preamble(0x0100) + CCD-CRC16 + SCD-CRC16 + channel_id
    // CCD: flags(2) + command_id(2) + length(2) + request_id(2)
    // ReadMem SCD: Address(8) + Reserved(2) + Length(2)  => length = 12 bytes
    const uint16_t PREAMBLE = 0x0100;
    const uint16_t READMEM_CMD = 0x0800; // ReadMem command id

    std::vector<uint8_t> pkt;
    pkt.reserve(8 + 8 + 12);

    auto be16 = [&](uint16_t v) { pkt.push_back(uint8_t(v >> 8)); pkt.push_back(uint8_t(v & 0xFF)); };
    auto be64 = [&](uint64_t v) {
        for (int i = 7; i >= 0; --i) pkt.push_back(uint8_t((v >> (8*i)) & 0xFF));
    };

    // preamble
    be16(PREAMBLE);
    // placeholders for CCD-CRC16 & SCD-CRC16
    size_t ccd_crc_off = pkt.size(); be16(0);
    size_t scd_crc_off = pkt.size(); be16(0);
    // channel_id
    be16(channel_id);

    // CCD
    be16(flags);
    be16(READMEM_CMD);
    be16(12);            // Length = SCD bytes
    be16(request_id);

    // SCD: address=0x08, reserved=0, read_length=64
    be64(0x08);
    be16(0);
    be16(64);

    // Compute CCD-CRC16 and SCD-CRC16 (one's complement checksum)
    // 變體A：涵蓋 [channel_id + CCD] / [channel_id + CCD + SCD]
    // 變體B：涵蓋 [preamble + channel_id + CCD] / [preamble + channel_id + CCD + SCD]
    auto checksum_range = [&](size_t from, size_t to_excl) -> uint16_t {
        return ones_complement_sum_be(pkt.data() + from, to_excl - from);
    };

    size_t preamble_off = 0;
    size_t channel_off  = 6; // after preamble(2)+ccdcrc(2)+scdcrc(2)
    size_t ccd_off      = 8; // after channel_id(2)
    size_t scd_off      = ccd_off + 8;
    size_t end_off      = pkt.size();

    uint16_t ccd_crc = 0, scd_crc = 0;
    if (crc_include_preamble) {
        ccd_crc = checksum_range(preamble_off, scd_off);
        scd_crc = checksum_range(preamble_off, end_off);
    } else {
        ccd_crc = checksum_range(channel_off, scd_off);
        scd_crc = checksum_range(channel_off, end_off);
    }

    pkt[ccd_crc_off + 0] = uint8_t(ccd_crc >> 8);
    pkt[ccd_crc_off + 1] = uint8_t(ccd_crc & 0xFF);
    pkt[scd_crc_off + 0] = uint8_t(scd_crc >> 8);
    pkt[scd_crc_off + 1] = uint8_t(scd_crc & 0xFF);

    return pkt;
}

static void print_ascii_hit(const std::vector<uint8_t>& rx) {
    const std::string needle = "OBSIDIAN";
    auto it = std::search(rx.begin(), rx.end(), needle.begin(), needle.end());
    if (it == rx.end()) return;

    size_t pos = (size_t)std::distance(rx.begin(), it);
    std::cout << ">>> FOUND ASCII \"OBSIDIAN\" at RX offset " << pos << "\n";
    // print a short window
    size_t start = (pos >= 32) ? pos - 32 : 0;
    size_t end = std::min(rx.size(), pos + 128);
    hexdump(rx.data() + start, end - start, start);
}

int main(int argc, char** argv) {
    std::vector<std::string> ports;
    if (argc >= 2) {
        for (int i = 1; i < argc; ++i) ports.push_back(argv[i]);
    } else {
        ports = {"/dev/ttyACM0", "/dev/ttyACM1"};
    }

    std::vector<int> bauds = {9600, 115200, 19200, 38400, 57600, 230400, 460800, 921600};

    std::cout << "=== GenCP Probe2 (ReadMem @0x08 len=64) ===\n";
    std::cout << "Try ports:";
    for (auto& p : ports) std::cout << " " << p;
    std::cout << "\n";

    for (auto& port : ports) {
        std::cout << "\n==============================\n";
        std::cout << "PORT: " << port << "\n";

        for (int baud : bauds) {
            int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd < 0) {
                std::cout << "  [open fail] baud=" << baud << " errno=" << errno << " (" << strerror(errno) << ")\n";
                continue;
            }

            if (!set_raw_serial(fd, baud)) {
                std::cout << "  [termios fail] baud=" << baud << " errno=" << errno << " (" << strerror(errno) << ")\n";
                close(fd);
                continue;
            }

            tcflush(fd, TCIOFLUSH);

            std::cout << "\n--- baud=" << baud << " ---\n";

            // passive read
            auto passive = read_for_ms(fd, 200);
            std::cout << "  [PASSIVE] " << passive.size() << " bytes\n";
            if (!passive.empty()) {
                hexdump(passive.data(), std::min<size_t>(passive.size(), 256));
            }

            // ASCII CLI poke
            const char* help = "\r\nhelp\r\n";
            write_all(fd, (const uint8_t*)help, strlen(help));
            auto ascii_rx = read_for_ms(fd, 200);
            std::cout << "  [ASCII help] RX " << ascii_rx.size() << " bytes\n";
            if (!ascii_rx.empty()) hexdump(ascii_rx.data(), std::min<size_t>(ascii_rx.size(), 256));

            // GenCP variants
            uint16_t req_id = 1;
            std::array<uint16_t,3> flag_variants = {0x4000, 0x8000, 0x0000};
            for (uint16_t flags : flag_variants) {
                for (int inc = 0; inc < 2; ++inc) {
                    bool include_preamble = (inc == 1);
                    auto pkt = build_readmem_cmd(flags, req_id++, include_preamble);

                    std::cout << "  [GenCP] flags=0x" << std::hex << flags << std::dec
                              << " checksum=" << (include_preamble ? "INCLUDE_PREAMBLE" : "NO_PREAMBLE")
                              << " TX=" << pkt.size() << " bytes\n";
                    hexdump(pkt.data(), pkt.size());

                    tcflush(fd, TCIFLUSH);
                    write_all(fd, pkt.data(), pkt.size());

                    auto rx = read_for_ms(fd, 1500);
                    std::cout << "  [GenCP] RX " << rx.size() << " bytes\n";
                    if (!rx.empty()) {
                        hexdump(rx.data(), std::min<size_t>(rx.size(), 512));
                        print_ascii_hit(rx);
                    } else {
                        std::cout << "  (no data)\n";
                    }
                }
            }

            close(fd);
        }
    }

    std::cout << "\n=== Done ===\n";
    return 0;
}
