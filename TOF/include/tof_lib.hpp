#ifndef  TOF_LIB
#define  TOF_LIB

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <stdint.h>
#include <signal.h>
#include <iomanip>  
#include <cstdint>
#include <cstddef>
#include <thread>
#include <atomic>
#include <mutex>

struct DistanceData{
    double timestamp; // 資料時間
    double range;     // 距離m
    double amplitude; // 反射強度
    int quality;  // 品質指標
    int16_t status;   // 狀態
};

class TOFSensor {
    public:
        TOFSensor(const std::string&port,int baudrate=115200);
        ~TOFSensor();

        // 初始化TOF
        bool initialize();

        // 背景啟動TOF
        void start();

        // 停止TOF
        void stop();

        // 取得TOF的距離資料
        DistanceData getTofDistance();

    private:
        uint8_t START_BYTE;
        uint8_t STOP_BYTE;
        uint8_t CMD_ACK;
        uint8_t CMD_NAK;
        uint8_t ESC_BYTE;
        std::atomic<bool> running_;
        std::string port_ ;
        int baudrate_;
        int fd_;
        bool initialized_;
        std::thread readThread_;
        std::mutex dataMutex_;
        DistanceData latestData_;

        void close_serial_port();
        int init_serial_port(const char* port_path, speed_t baudrate);
        bool write_serial(int fd, const uint8_t* cmd, size_t len);
        bool wait_for_ack(int fd,uint8_t txcmd);
        size_t remove_byte_stuffing(uint8_t* buffer, size_t buffer_len);
        void flush_port(int fd);
        bool read_buffer(int fd, uint8_t* buffer, size_t max_len, size_t& buffer_len);
        void parse1D(uint8_t* buffer, size_t buffer_len);
        uint8_t crc8_sae_j1850(const uint8_t* data, size_t len);
        void set_TOF_1Dmode();
        void set_TOF_20fps();
        void set_TOF_periodically_data_report();
        void set_TOF_stop();
        void readloop();
        
};       
#endif
