#include "./include/tof_lib.hpp"  // include tof library
#include <chrono>

bool running = true;

void sigint_handler(int) {
    running = false;
}

//測試Tof功能
int main(){
    signal(SIGINT, sigint_handler);
    // 建立TOF物件為tof
    // TOFSensor tof("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0",B115200); //使用USB to TTL
    TOFSensor tof("/dev/ttyS3",B115200); // 使用GPIO
    if (!tof.initialize()) {             // 初始化tof
        std::cerr << "初始化失敗，無法開啟 TOF 裝置\n";
        return 1;
    }
    tof.start();     // 啟動tof
    while (running){
        DistanceData distancedate = tof.getTofDistance();       //抓取tof的距離參數
        std::cout << "Time: " << distancedate.timestamp << "s, Range: " << distancedate.range
            << " m, Amp: " << distancedate.amplitude
            << ", Quality: " << distancedate.quality
            << ", Status: " << distancedate.status << std::endl;
        
        // 拿取資料間隔至少50ms
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    tof.stop();     // 停止tof
    return 0;
}
