#include "./include/tof_lib.hpp"

TOFSensor::TOFSensor(const std::string&port,int baudrate)
    :  START_BYTE(0x02),
       STOP_BYTE(0x03),
       CMD_ACK(0x0A),
       CMD_NAK(0x0B),
       ESC_BYTE(0x1B),
       running_(false),
       port_(port),
       baudrate_(baudrate),
       fd_(-1),
       initialized_(false),
       latestData_{}
{

}

TOFSensor::~TOFSensor(){
    if (running_){
        stop();
    }

}

void TOFSensor::stop() {
    if (!running_) return;
    running_ = false;
    if (readThread_.joinable()) {
        readThread_.join();
    }
    close_serial_port();
    std::cout << "Stop TOF Measurements" << std::endl;
}

void TOFSensor::close_serial_port() {
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
}

int TOFSensor::init_serial_port(const char* port, speed_t baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;   // 不等待字元
    tty.c_cc[VTIME] = 10;  // 最多等待 1 秒

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

bool TOFSensor::write_serial(int fd, const uint8_t* cmd, size_t len) {
    int n = write(fd, cmd, len);
    uint8_t txcmd = cmd[1];
    return wait_for_ack(fd,txcmd);
}

bool TOFSensor::wait_for_ack(int fd,uint8_t txcmd){
    uint8_t buffer[256];
    size_t buffer_len = 0;

    while (true) {
        flush_port(fd);
        memset(buffer, 0, sizeof(buffer));
        // int n = read(fd, buffer, sizeof(buffer));
        // std::cout << n << std::endl;
        if (!read_buffer(fd, buffer, sizeof(buffer),buffer_len)){
            std::cout << "failed to read\n";
            return false;
        }

        if (buffer_len < 3 || buffer[0] != START_BYTE || buffer[buffer_len - 1] != STOP_BYTE) {
            std::cerr << "Invalid frame format\n";
            // std::cout << "2";
            continue;
        }
        size_t new_len = remove_byte_stuffing(buffer, buffer_len);

        uint8_t rxcmd = buffer[1];
        if (rxcmd == txcmd) {
            std::cout << "rxcmd:  ";
            for (int i = 0; i < new_len; i++) {
                std::cout << "0x"
                        << std::hex << std::setw(2) << std::setfill('0')
                        << (static_cast<int>(buffer[i]) & 0xFF)
                        << " ";
            }
            // std::cout << "3";
            return true;
        } else if (rxcmd == CMD_ACK) {
            uint8_t ackcmd = buffer[2];
            if (ackcmd == txcmd) {
                // std::cout << "4";
                std::cout << "Command executed successfully\n";
                return true;
            } else {
                std::cerr << "Invalid ACK received\n";
                // std::cout << "5";
                return false;
            }
        } else if (rxcmd == CMD_NAK) {
            uint8_t nakcmd = buffer[2];
            if (nakcmd == txcmd) {
                std::cerr << "NAK received for cmd 0x" << std::hex << (int)txcmd << "\n";
                // std::cout << "6";
                return false;
            } else {
                std::cerr << "Invalid NAK received\n";
                // std::cout << "7";
                return false;
            }
        }
    }
    return false;
}

size_t TOFSensor::remove_byte_stuffing(uint8_t* buffer, size_t buffer_len) {
    size_t write_idx = 0;

    for (size_t read_idx = 0; read_idx < buffer_len; ++read_idx) {
        if (buffer[read_idx] == ESC_BYTE) {
            // Skip escape byte and invert the next byte
            ++read_idx;
            if (read_idx >= buffer_len) break;  // prevent out-of-bounds
            buffer[write_idx++] = buffer[read_idx] ^ 0xFF;
        } else {
            buffer[write_idx++] = buffer[read_idx];
        }
    }

    // buffer now contains cleaned data in-place
    buffer_len = write_idx;
    return write_idx;
}

void TOFSensor::flush_port(int fd){
    tcflush(fd, TCIOFLUSH);
}

bool TOFSensor::read_buffer(int fd, uint8_t* buffer, size_t max_len, size_t& buffer_len){
    buffer_len = 0; 
    uint8_t b;

    // find START_BYTE
    while (true) {
        if (read(fd, &b, 1) != 1) return false;
        if (b == START_BYTE) {
            buffer[buffer_len++] = b;
            break;
        }
    }

    // read still STOP_BYTE
    while (true) {
        if (read(fd, &b, 1) != 1) return false;

        // ensure not overflow buffer
        if (buffer_len >= max_len) return false;

        buffer[buffer_len++] = b;

        if (b == STOP_BYTE) break; // recieve STOP_BYTE than stop
    }

    return true;
}

void TOFSensor::parse1D(uint8_t* buffer, size_t buffer_len) {
    size_t new_len = remove_byte_stuffing(buffer, buffer_len);
    if (new_len >= 21 && buffer[1] == 0xB6) {
        int16_t status = (buffer[3] << 8) | buffer[4];

        uint32_t tsec = (buffer[5] << 24) | (buffer[6] << 16) | (buffer[7] << 8) | buffer[8];
        uint16_t tusec = (buffer[9] << 8) | buffer[10];
        double timestamp = tsec + tusec * 16.0 / 1e6;

        uint32_t raw_range = (buffer[15] << 16) | (buffer[16] << 8) | buffer[17];
        double range = raw_range / 16384.0;

        uint16_t amp = (buffer[18] << 8) | buffer[19];
        double amplitude = amp / 16.0;

        uint8_t quality = buffer[20];

        std::lock_guard<std::mutex> lock(dataMutex_);
        latestData_.timestamp = timestamp;
        latestData_.range = range;
        latestData_.amplitude = amplitude;
        latestData_.quality = (int)quality;
        latestData_.status = status;

        // std::cout << "Time: " << timestamp << "s, Range: " << range
        //             << " m, Amp: " << amplitude
        //             << ", Quality: " << (int)quality
        //             << ", Status: " << status << std::endl;
    }
}

uint8_t TOFSensor::crc8_sae_j1850(const uint8_t* data, size_t len) {
    const uint8_t poly = 0x1D; // Polynomial for SAE J1850
    uint8_t crc = 0x00;        // Initial value

    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    // Final XOR value is 0x00 (no change)
    return crc;
}

void TOFSensor::set_TOF_1Dmode(){
    uint8_t cmd1 []= { 0x02, 0x41, 0x07, 0xF5, 0x03 };
    write_serial(fd_,cmd1,sizeof(cmd1));
}

void TOFSensor::set_TOF_20fps(){
    int length = 7;
    //100fps
    // uint8_t cmd2 []= { 0x02, 0x43, 0x00, 0x00, 0x27, 0x10, 0x03  }; 
    //20fps
    uint8_t cmd2 []= { 0x02, 0x43, 0x00, 0x00, 0xc3, 0x50, 0x03  };
    // 10fps
    // uint8_t cmd2 []= { 0x02, 0x43, 0x00 ,0x01, 0x86, 0xA0, 0x03};
    uint8_t data[] = {cmd2[1],cmd2[2], cmd2[3], cmd2[4], cmd2[5]};
    uint8_t result = crc8_sae_j1850(data, sizeof(data));
    // std::cout << "0x"
    //     << std::hex << std::setw(2) << std::setfill('0')
    //     << (static_cast<int>(result) & 0xFF)
    //     << std::endl;
    // 從最後一個元素往後移動，直到位置 7（包含）
    for (int i = length; i > length-1; --i) {
        cmd2[i] = cmd2[i - 1];
    }
    cmd2[length-1] = result;

    // print frame
    // for (int i = 0; i < sizeof(cmd2)+1; i++) {
    //     std::cout << "0x"
    //             << std::hex << std::setw(2) << std::setfill('0')
    //             << (static_cast<int>(cmd2[i]) & 0xFF)
    //             << std::endl;
    // }
    length++;
    write_serial(fd_,cmd2,sizeof(cmd2)+1);
}

void TOFSensor::set_TOF_periodically_data_report(){
    uint8_t cmd3 []= { 0x02, 0x11, 0xD0, 0x03 };
    write_serial(fd_,cmd3,sizeof(cmd3));
}

void TOFSensor::set_TOF_stop(){
    uint8_t cmd4 []={ 0x02, 0x12, 0xF7, 0x03 };
    write_serial(fd_,cmd4,sizeof(cmd4));
}

// 初始化TOF
bool TOFSensor::initialize(){
    fd_ = init_serial_port(port_.c_str(), baudrate_);
    if (fd_ < 0) {
        std::cerr << "open serialport is failed\n";
        return false;
    }

    initialized_ = true;
    return true;
}

// 背景啟動TOF
void TOFSensor::start(){
    if(initialized_){
        set_TOF_1Dmode();
        set_TOF_20fps();
        set_TOF_periodically_data_report();
        running_ = true;
        readThread_ = std::thread(&TOFSensor::readloop, this);
        std::cout << "Start to read TOF distance data ...\n";
    }else{
        std::cout << "Start failed\n";
    }
}

void TOFSensor::readloop(){
    uint8_t buffer[256];
    size_t buffer_len = 0;
    while (running_) {
        flush_port(fd_);
        memset(buffer, 0, sizeof(buffer));
        // int n = read(fd, buffer, sizeof(buffer));
        // std::cout << n << std::endl;
        bool n =read_buffer(fd_, buffer, sizeof(buffer),buffer_len);
        if (n) {
            // 直接將讀取到的 n 個位元組印出
            // std::cout.write(buffer, n);
            // std::cout.flush();
            parse1D(buffer,buffer_len);
            // std::cout << buffer_len << std::endl;
        } else if (n < 0) {
            perror("read error");
            break;
        } else{
            //std::cout << "time out\n";
        }
    }
}

// 取得TOF的距離資料
DistanceData TOFSensor::getTofDistance(){
    std::lock_guard<std::mutex> lock(dataMutex_);
    return latestData_;
}
