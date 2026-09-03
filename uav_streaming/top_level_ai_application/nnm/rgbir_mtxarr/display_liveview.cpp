#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <ctime>    // for time(), localtime(), strftime()

#include <iostream>
#include <map>
#include <cmath>
#include <limits>
#include <regex>
#include <fstream>
#include <sstream>

extern "C" {
#include "kdp2_inf_app_yolo.h"
}

#include "example_shared_struct.h"
#include "kp_struct.h"

#if 0
#include "/root/Desktop/TOF/include/tof_lib.hpp"  // include tof library
#define TOF_EN
#endif

//資料夾
#include <limits.h>   // PATH_MAX
#include <errno.h>    // errno, EEXIST
#include <string.h>   // strlen, strcpy, strncpy
#include <deque>  // 1011新增(for存N秒的圖片)

// === 0822 新增：I2C / GPIO 相關 ===
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>

// ===== 計時用的 Simple scope profiler (with file logging) =====
#include <unordered_map>
#include <string>
#include <chrono>
#include <cstdio>   // FILE, fopen, fprintf, fflush

//For Webmin控制
#include <atomic>
#include <poll.h>
#include <sstream>
#include <algorithm>    // for std::clamp / std::min

//internet streaming
#if 1	//JDBG
#include "h26xenc_api.h"
#include <mem_broker.h>
#include <sync_shared_memory.h>
#include <sync_ring_buffer.h>
#include <ssm_info.h>
#include <resize.h>
#include <msg_broker.h>

#include <sys/ioctl.h>
#include "videodev2.h"

#include <iniparser/iniparser.h>
#define CAM_WIDTH  640
#define CAM_HEIGHT 480

#if 0	//JDBG
#define INC_MTX_ARR
#include "config/camera_params.txt"
#include "config/homography_table.txt"
#else
std::string camera_params_path = "/work/config/camera_params.txt"; 
std::string homography_table_path = "/work/config/homography_table.txt"; 
#endif

template <typename T>
static inline T clamp_compat(T v, T lo, T hi) {
    return std::min(std::max(v, lo), hi);
}

// ====== 0924 Config for dx/dy/cmap & auto-save ======
#include <fstream>
#include <cctype>   // for std::tolower

// forward declaration，避免在 ensure_parent_dir() 使用時找不到宣告
static int mkdir_p(const char* path, mode_t mode = 0755);

static inline bool file_exists(const std::string& p) {
    struct stat st;
    return (stat(p.c_str(), &st) == 0) && S_ISREG(st.st_mode);
}

// 若你檔案已有 trim()/mkdir_p() 就沿用；沒有就保留這兩個工具：
static inline void trim(std::string &s){
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) { s.clear(); return; } s = s.substr(a, b - a + 1);
}

struct AppConfig {
    std::string path = "/work/config/rgbir.cfg";
    int  save_cfg_minutes = 10;
    int  dx = 0, dy = 0;
    int  cmap = 2;               // 0..5
    int  alpha_idx = 2;          // 0..4

    // ==== 1011 新增：可設定的存圖路徑（預設沿用你原本的資料夾） ====
    std::string save_dir_rgb   = "/work/image/rgb";
    std::string save_dir_ir    = "/work/image/ir";
    std::string save_dir_fused = "/work/image/fused";
    // ====1011 新增：按鈕觸發時，要「回存」的秒數 ====
    int pre_snap_seconds = 10;    // 預設保留前10秒
    // 1011 新增 END
};
static AppConfig g_cfg;

static void ensure_parent_dir(const std::string& p){
    size_t slash = p.rfind('/'); if (slash==std::string::npos) return;
    mkdir_p(p.substr(0, slash).c_str(), 0755);
}

static void config_load(AppConfig& c){
    std::ifstream fin(c.path);
    if (!fin) return;
    std::string line;
    while (std::getline(fin, line)){
        auto pos = line.find('='); if (pos == std::string::npos) continue;
        std::string k = line.substr(0, pos), v = line.substr(pos+1); trim(k); trim(v);
        if (k=="save_cfg_minutes") c.save_cfg_minutes = std::max(1, atoi(v.c_str()));
        else if (k=="dx") c.dx = atoi(v.c_str());
        else if (k=="dy") c.dy = atoi(v.c_str());
        else if (k=="cmap") c.cmap = clamp_compat(atoi(v.c_str()), 0, 5);
        else if (k=="alpha_idx") {
            c.alpha_idx = clamp_compat(std::atoi(v.c_str()), 0, 4);
        }
        else if (k=="alpha_percent" || k=="alpha_pct" || k=="alpha") {
            int ap = clamp_compat(std::atoi(v.c_str()), 0, 100);
            // 轉成 5 檔索引（0,25,50,75,100 就近對齊）
            c.alpha_idx = std::min(4, (int)std::round(ap / 25.0));
        }
        // 1011 ==== 新增：路徑與秒數 ====
        else if (k=="save_dir_rgb")   c.save_dir_rgb   = v;
        else if (k=="save_dir_ir")    c.save_dir_ir    = v;
        else if (k=="save_dir_fused") c.save_dir_fused = v;
        else if (k=="pre_snap_seconds") c.pre_snap_seconds = std::max(1, atoi(v.c_str()));
        // 1011 新增 END
    }
}

// 原型從 3 參數 → 4 參數
static void config_save(const AppConfig& c, int cur_dx, int cur_dy, int cur_cmap, int cur_alpha_idx){
    ensure_parent_dir(c.path);
    std::ofstream fout(c.path, std::ios::trunc);
    if (!fout) { perror("config_save"); return; }
    fout << "save_cfg_minutes=" << c.save_cfg_minutes << "\n";
    fout << "dx=" << cur_dx << "\n";
    fout << "dy=" << cur_dy << "\n";
    fout << "cmap=" << cur_cmap << "\n";
    // ★ 新增兩行，兩者擇一讀也行（上面 B 已同時支援）
    fout << "alpha_idx=" << cur_alpha_idx << "\n";
    int pct = cur_alpha_idx * 25;              // 0..4 -> 0..100
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    fout << "alpha_percent=" << pct << "\n";
    // 1011 新增
    fout << "save_dir_rgb="   << c.save_dir_rgb   << "\n";
    fout << "save_dir_ir="    << c.save_dir_ir    << "\n";
    fout << "save_dir_fused=" << c.save_dir_fused << "\n";
    fout << "pre_snap_seconds=" << c.pre_snap_seconds << "\n";
    // 1011 新增 END
    fout.flush();
}
// ====== 0924 Config for dx/dy/cmap & auto-save END======

// 1011 新增(for存N秒的圖片)
struct RingItem {
    time_t t;
    std::string rgb, ir, fused;  // buffer 檔案路徑
};

static std::deque<RingItem> g_ring;   // 只放「buffer」中的檔案清單

static std::string ts_string(time_t t){
    char buf[32]; std::tm tm_info{};
    localtime_r(&t, &tm_info);
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_info);
    return std::string(buf);
}
// 1011 新增(for存N秒的圖片) END


//-------------計時用的-------------------

struct ProfilerWindow {
    std::unordered_map<std::string, double> sums_ms;
    FILE* logf = nullptr;  // 日誌檔案句柄（可選）

    void add(const std::string& key, double ms) { sums_ms[key] += ms; }

    // 設定輸出檔（append），失敗時只會印到 stdout
    bool open_log(const char* path) {
        if (logf) { fclose(logf); logf = nullptr; }
        logf = fopen(path, "a");
        if (!logf) { perror("open_log"); return false; }
        setvbuf(logf, nullptr, _IOLBF, 0);  // line-buffered，遇到 '\n' 就寫
        return true;
    }
    void close_log() {
        if (logf) { fflush(logf); fclose(logf); logf = nullptr; }
    }

    // 同時印到 stdout 與 log 檔（若有）
    void print_now(bool also_stdout = true) {
        auto emit = [&](FILE* out){
            if (!out) return;
            double total = sums_ms.count("TOTAL") ? sums_ms["TOTAL"] : 0.0;
            std::fprintf(out, "\n==== Profiling (this frame) ====\n");
            auto print_line = [&](const char* name){
                double ms = sums_ms.count(name) ? sums_ms[name] : 0.0;
                double pct = (total>0.0) ? (ms*100.0/total) : 0.0;
                std::fprintf(out, "%-22s %7.3f ms  (%5.1f%%)\n", name, ms, pct);
            };
            // print_line("TOF_get");
            print_line("Image_Sensor_format");
            print_line("Webcam_format");
            print_line("YOLO_Draw");
            print_line("Webcam_Remap");
            print_line("getNearestHomography");
            print_line("Colormap");
            print_line("Warp");
            print_line("Display_imshow");
            print_line("GPIO_I2C_Read");
            print_line("Save_Images");
            print_line("WaitKey");
            print_line("OTHER");
            print_line("TOTAL");
            std::fprintf(out, "================================\n");
            std::fflush(out);
        };
        if (also_stdout) emit(stdout);
        emit(logf);
        sums_ms.clear(); // 清空，避免下一圈累加
    }
};

struct ScopeTimer {
    std::chrono::steady_clock::time_point t0;
    ProfilerWindow* win;
    const char* key;
    ScopeTimer(ProfilerWindow* w, const char* k) : t0(std::chrono::steady_clock::now()), win(w), key(k) {}
    ~ScopeTimer(){
        using namespace std::chrono;
        double ms = duration_cast<duration<double, std::milli>>(steady_clock::now()-t0).count();
        if (win) win->add(key, ms);
    }
};
//-------------計時用的 end -------------------

// ---- I2C (ADS1115) 設定 ----
#define I2C_DEV0 "/dev/i2c-0"
#define I2C_DEV1 "/dev/i2c-1"   // 若 0 沒有，就試 1
#define ADS1115_ADDR 0x48       // 常見模組預設位址

// ADS1115 暫存器
#define ADS1115_REG_CONV   0x00
#define ADS1115_REG_CONFIG 0x01


extern NNM_SHARED_INPUT_T _input_data;
extern pthread_mutex_t _mutex_image;

//WEBCAM 
extern NNM_SHARED_INPUT_T _input_data_webcam;
extern pthread_mutex_t _mutex_image_webcam;

extern NNM_SHARED_RESULT_T _inf_result;
extern pthread_mutex_t _mutex_result;

extern unsigned int _image_count;
extern unsigned int _result_count;

extern bool _blDispatchRunning;
extern bool _blFifoqManagerRunning;

extern bool _blImageRunning;
extern bool _blSendInfRunning;
extern bool _blResultRunning;

bool _blDisplayRunning = true;

extern void sig_kill(int signo);


//鍵盤調整 位置
int dx = 0;
int dy = 0;
const int move_step = 2;  // 每次按鍵移動的像素量
//鍵盤調整 初始 IR 權重
double alpha_ir = 0.5;
double alpha_step = 0.1;

// -------------------------------
// IR 魚眼/廣角去畸變（使用 remap）
// -------------------------------
// 內參與畸變（依你的 IR 相機標定值）
cv::Mat K_webcam = (cv::Mat_<double>(3, 3) <<
    674.14078639, 0.0,          328.11609511,
    0.0,          680.46744864, 256.62274547,
    0.0,          0.0,          1.0);
cv::Mat D_webcam = (cv::Mat_<double>(1, 5) <<
    -0.50757016, 0.38615872, 0.0130162, 0.00440346, 0.0);

// remap 用的查表與旗標
cv::Mat map1_web, map2_web, newK_web;
bool map_ready = false;
double undist_alpha = 0.0;  // 0.0=盡量裁黑邊；1.0=保最大視野（可能留黑邊）


// ---- 0822 GPIO（搖桿按鈕） ----
// ---- 0924 Re-map buttons: keep 56/69/70; free 55/54 ----
static const int GPIO_BTN_RESET = 52;   // 中心鍵 → reset dx/dy
static const int GPIO_BTN_ALPHA = 53;   // 白色鍵 → 透明度 5 檔輪替
static const int GPIO_BTN_CMAP  = 56;   // 黃色鍵 → Cmap 六種輪替（原本就用 56）
static const int GPIO_BTN_SNAP  = 60;   // 新增：按一下存一張
// ---- 0924 Re-map buttons: keep 56/69/70; free 55/54 END----

// sysfs GPIO helpers（簡單做法，夠用）
static void gpio_export(int gpio) {
    char path[64];
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) {
        int len = snprintf(path, sizeof(path), "%d", gpio);
        write(fd, path, len);
        close(fd);
    }
}
static void gpio_set_direction_in(int gpio) {
    char path[64]; snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(path, O_WRONLY);
    if (fd >= 0) { write(fd, "in", 2); close(fd); }
}
static int gpio_read_value(int gpio) {
    char path[64]; snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    char v='1';
    int fd = open(path, O_RDONLY);
    if (fd >= 0) { read(fd, &v, 1); close(fd); }
    return (v=='0') ? 0 : 1; // 搖桿 SW 多為「按下=低態」
}

// I2C helpers
static int i2c_open_ads(const char **which_dev = nullptr) {
    int fd = open(I2C_DEV0, O_RDWR);
    if (fd < 0) fd = open(I2C_DEV1, O_RDWR);
    if (fd < 0) return -1;
    if (ioctl(fd, I2C_SLAVE, ADS1115_ADDR) < 0) { close(fd); return -1; }
    if (which_dev) *which_dev = (access(I2C_DEV0, F_OK)==0)? I2C_DEV0 : I2C_DEV1;
    return fd;
}

// 讀單一通道（AIN0=0, AIN1=1），單次轉換模式
static int16_t ads1115_read_channel(int fd, uint8_t ch) {
    // MUX: 100=A0, 101=A1, 110=A2, 111=A3（對 GND）
    uint16_t mux = (ch==0)? 0x4000 : (ch==1? 0x5000 : (ch==2? 0x6000 : 0x7000));
    // PGA：±4.096V (001) -> 0x0200；若全系統用 3.3V，也可改 ±2.048V (010)->0x0400
    uint16_t pga = 0x0200;
    // MODE：單次 1->0x0100；資料率 DR=475SPS(110)->0x00C0 或 860SPS(111)->0x00E0
    uint16_t mode_dr = 0x0100 | 0x00E0;
    // 觸發單次轉換 OS=1->0x8000；關閉比較器 QUE=11->0x0003
    uint16_t cfg = 0x8000 | mux | pga | mode_dr | 0x0003;

    uint8_t wbuf[3] = { ADS1115_REG_CONFIG, (uint8_t)(cfg>>8), (uint8_t)(cfg & 0xFF) };
    if (write(fd, wbuf, 3) != 3) return 0;

    // 等待轉換完成（860SPS 約 1.2ms，保守 2ms）
    usleep(2000);

    // 讀取轉換結果
    uint8_t reg = ADS1115_REG_CONV;
    if (write(fd, &reg, 1) != 1) return 0;
    uint8_t rbuf[2] = {0};
    if (read(fd, rbuf, 2) != 2) return 0;

    int16_t val = (int16_t)((rbuf[0] << 8) | rbuf[1]);
    return val;
}

// 把 ADC 值轉成「每一幀要加多少像素」的步進（含 deadzone 與動態倍率）
static int axis_to_step(int16_t raw, int16_t center, int deadzone, int max_step) {
    int diff = (int)raw - (int)center;
    if (std::abs(diff) < deadzone) return 0;
    double mag = (std::abs(diff) - deadzone) / (double)(32767 - deadzone);
    int step = 1 + (int)std::floor(mag * (max_step-1));
    return (diff >= 0) ? step : -step;
}

// IR 透明度五檔（0%,25%,50%,75%,100%）
static const double kAlphaLevels[5] = {0.0, 0.25, 0.50, 0.75, 1.00};
static int alpha_idx = 2;              // 初始 50%

// 六種常用 colormap（OpenCV 預設枚舉）
static const int kColorMaps[6] = {
    cv::COLORMAP_AUTUMN,
    cv::COLORMAP_JET,
    cv::COLORMAP_HOT,
    cv::COLORMAP_BONE,
    cv::COLORMAP_RAINBOW,
    cv::COLORMAP_TURBO
};
static const char* kColorMapNames[6] = {
    "AUTUMN","JET","HOT","BONE","RAINBOW","TURBO"
};
static int cmap_idx = 2;               // 初始用 HOT
// 0822 搖桿 end

// 初始化時檢查 + 建立資料夾
static int mkdir_p(const char* path, mode_t mode) {
    if (!path || !*path) return -1;

    char tmp[PATH_MAX];
    size_t len = strlen(path);
    if (len >= sizeof(tmp)) return -1;

    // 複製，並移除最後一個 '/'（避免多餘目錄層）
    strcpy(tmp, path);
    if (len > 1 && tmp[len-1] == '/') {
        tmp[len-1] = '\0';
    }

    // 逐層建立
    for (char* p = tmp + 1; *p; ++p) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, mode) != 0 && errno != EEXIST) return -1;
            *p = '/';
        }
    }
    if (mkdir(tmp, mode) != 0 && errno != EEXIST) return -1;
    return 0;
}
// 1011刪除 初始化時檢查 + 建立資料夾
// static void ensure_output_dirs() {
// }

// 1011新增 自動建立路徑
static inline std::string join_path(const std::string& a, const std::string& b){
    if (a.empty()) return b;
    if (a.back()=='/') return a + b;
    return a + "/" + b;
}
static void ensure_dirs_from_cfg(const AppConfig& c){
    // 主資料夾
    mkdir_p(c.save_dir_rgb.c_str(),   0755);
    mkdir_p(c.save_dir_ir.c_str(),    0755);
    mkdir_p(c.save_dir_fused.c_str(), 0755);
    // 讓第3點的「前 n 秒快取」與「快照」各有自己的子資料夾
    mkdir_p(join_path(c.save_dir_rgb,   "buffer").c_str(), 0755);
    mkdir_p(join_path(c.save_dir_ir,    "buffer").c_str(), 0755);
    mkdir_p(join_path(c.save_dir_fused, "buffer").c_str(), 0755);
    mkdir_p(join_path(c.save_dir_rgb,   "snapshots").c_str(), 0755);
    mkdir_p(join_path(c.save_dir_ir,    "snapshots").c_str(), 0755);
    mkdir_p(join_path(c.save_dir_fused, "snapshots").c_str(), 0755);
}
// 產生不重名的批次資料夾名稱，例如：snap_YYYYMMDD_HHMMSS、snap_YYYYMMDD_HHMMSS_2、_3...
// 只檢查「某個根路徑」下是否存在，回傳可用的批次名稱（不含根路徑）。
static std::string unique_batch_tag(const std::string& root_dir, const std::string& ts) {
    std::string base = "snap_" + ts;          // 例如 snap_20251011_142233
    struct stat st{};
    std::string full = root_dir + "/" + base;
    if (stat(full.c_str(), &st) != 0) return base; // 不存在 → 直接用

    // 已存在 → 從 _2 開始找
    for (int i = 2; ; ++i) {
        std::string name = base + "_" + std::to_string(i);
        full = root_dir + "/" + name;
        if (stat(full.c_str(), &st) != 0) return name;  // 找到第一個不存在的
    }
}
// 1011新增 自動建立路徑 END

// ===== Webmin 控制通道（named pipe）=====
static const char* CTL_FIFO = "/run/rgbir_ctl";

// 「待處理」命令（用原子變數暫存；主迴圈每幀一次統一套用）
static std::atomic<int> pend_move_dx{0};
static std::atomic<int> pend_move_dy{0};
static std::atomic<int> pend_reset{0};
static std::atomic<int> pend_alpha_next{0};
static std::atomic<int> pend_set_alpha{-1};   // -1=無；0..100（百分比）
static std::atomic<int> pend_cmap_next{0};
static std::atomic<int> pend_set_cmap{-1};    // -1=無；0..5
static std::atomic<int> pend_snap{0}; // 0924新增

static void applyPendingControls();           // 前向宣告
static void* ctrl_fifo_thread(void*);         // 前向宣告

// Webmin 控制
static void applyPendingControls() {
    if (pend_reset.exchange(0)) { dx = dy = 0; }

    int mdx = pend_move_dx.exchange(0);
    int mdy = pend_move_dy.exchange(0);
    if (mdx | mdy) { dx += mdx; dy += mdy; }

    // 透明度
    int an = pend_alpha_next.exchange(0);
    while (an-- > 0) {                   // 支援一次來多個 next
        alpha_idx = (alpha_idx + 1) % 5;
        alpha_ir  = kAlphaLevels[alpha_idx];
    }
    int ap = pend_set_alpha.exchange(-1);
    if (ap >= 0) {
        ap = clamp_compat(ap, 0, 100);
        alpha_ir  = ap / 100.0;
        // 讓 HUD / 按鈕行為與 5 檔一致（就近對齊）
        alpha_idx = std::min(4, (int)std::round(ap / 25.0));
    }

    // colormap
    int cn = pend_cmap_next.exchange(0);
    while (cn-- > 0) { cmap_idx = (cmap_idx + 1) % 6; }
    int cs = pend_set_cmap.exchange(-1);
    if (cs >= 0) { cmap_idx = clamp_compat(cs, 0, 5); }
}

static inline void to_lower(std::string &s) {
    for (auto &c : s) c = (char)std::tolower((unsigned char)c);
}

// Webmin 把一行文字指令轉成「待處理命令」
static void consume_line_cmd(const std::string &line) {
    std::string s = line; trim(s); if (s.empty()) return;
    std::istringstream iss(s);
    std::string cmd, a1, a2; iss >> cmd >> a1 >> a2; to_lower(cmd);

    if      (cmd == "alpha_next") { pend_alpha_next.fetch_add(1); }
    else if (cmd == "alpha")      { pend_set_alpha.store(std::strtol(a1.c_str(), nullptr, 10)); }
    else if (cmd == "cmap_next")  { pend_cmap_next.fetch_add(1); }
    else if (cmd == "cmap")       { pend_set_cmap.store(std::strtol(a1.c_str(), nullptr, 10)); }
    else if (cmd == "move") {
        int mx = std::strtol(a1.c_str(), nullptr, 10);
        int my = std::strtol(a2.c_str(), nullptr, 10);
        pend_move_dx.fetch_add(mx);
        pend_move_dy.fetch_add(my);
    }
    else if (cmd == "reset")      { pend_reset.store(1); }
    else if (cmd == "snap" || cmd == "capture") { pend_snap.fetch_add(1); }//0924新增
}

// Webmin 非阻塞讀 FIFO 的執行緒（200ms poll）
static void* ctrl_fifo_thread(void*) {
    // 建立 FIFO（若已存在會回錯誤，無妨）
    struct stat st{};
    if (stat(CTL_FIFO, &st) != 0 || !S_ISFIFO(st.st_mode)) {
        mkfifo(CTL_FIFO, 0660);
        chmod (CTL_FIFO, 0660);
    }

    int fd = -1; std::string acc;
    while (_blDisplayRunning) {                          // 你的主程式控制旗標:contentReference[oaicite:5]{index=5}
        if (fd < 0) {
            fd = open(CTL_FIFO, O_RDONLY | O_NONBLOCK);
            if (fd < 0) { usleep(200000); continue; }
        }
        struct pollfd pfd{fd, POLLIN, 0};
        int pr = poll(&pfd, 1, 200);
        if (pr > 0 && (pfd.revents & POLLIN)) {
            char buf[512]; ssize_t n = read(fd, buf, sizeof(buf));
            if (n > 0) {
                acc.append(buf, buf + n);
                size_t pos;
                while ((pos = acc.find('\n')) != std::string::npos) {
                    std::string line = acc.substr(0, pos);
                    acc.erase(0, pos + 1);
                    consume_line_cmd(line);
                }
            } else if (n == 0) { close(fd); fd = -1; }   // writer 關閉 → 重新 open
        }
    }
    if (fd >= 0) close(fd);
    return nullptr;
}

//internet streaming
struct Device {
    int iFd;
    enum v4l2_buf_type eBufType;
    enum v4l2_memory eMemType;
    unsigned int dwWidth;
    unsigned int dwHeight;
};

#ifndef MAKEFOURCC
#define MAKEFOURCC(a,b,c,d) (((uint32_t)a) | (((uint32_t)b)<<8) | \
		(((uint32_t)c)<<16) | (((uint32_t)d)<<24))
#endif
#define ptEncBuff_SIZE 1024*1024*8	//2000000
#define FOURCC_CONF		(MAKEFOURCC('C','O','N','F'))
#define FOURCC_H264		(MAKEFOURCC('H','2','6','4'))
#define FOURCC_H265		(MAKEFOURCC('H','2','6','5'))
#define UBUFFER_HEADERSIZE	256
VMF_CODEC_INITOPT_T codec_initopt;

char *g_szSsmReaderPin = NULL;
char *g_szSrbWriterPin = NULL;
int g_bTerminate = 0;
int g_eCodec = 0;
int g_bWriteEnable = 0;
bool g_bConfFlag = true;
unsigned int g_dwTargetW = 0;
unsigned int g_dwTargetH = 0;

SSM_WRITER_INIT_OPTION_T ssm_opt;
SSM_HANDLE_T* g_ptSsmWriterHandle = NULL;

SRB_HANDLE_T* g_ptVideoSrbHandle = NULL;
SSM_HANDLE_T* g_ptSsmReaderHandle = NULL;
VMF_RS_HANDLE_T* g_ptResizeHandle = NULL;
SSM_BUFFER_T g_tInSsmBuf;
SRB_BUFFER_T g_tVideoSrbWriterBuf;
VMF_VSRC_SSM_OUTPUT_INFO_T  g_tInSsmInfo;
static VMF_VENC_H265_STREAM_HDR * g_ptStreamHdr;

pthread_cond_t g_tVideoCond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t g_tVideoMutex = PTHREAD_MUTEX_INITIALIZER;

struct timeval tv_prev, tv;
unsigned long total_time = 0;



typedef enum
{
    SEND_CONF = 0,
    SEND_DATA
} SEND_STATE;

static int video_queue_buffer(struct Device *dev, int index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    int ret;

    memset(&buf, 0, sizeof buf);
    memset(&planes, 0, sizeof planes);

    buf.index = index;
    buf.type = dev->eBufType;
    buf.memory = dev->eMemType;

    ret = ioctl(dev->iFd, VIDIOC_QBUF, &buf);
    if (ret < 0)
        printf("Unable to queue buffer: %s (%d).\n",
               strerror(errno), errno);

    return ret;
}

static void send_video_data(SEND_STATE eState)
{
    unsigned int bStrmHeaderSize = 0;
    if(eState == SEND_CONF){

        if (g_ptStreamHdr->dwVpsSize == 0) {
            bStrmHeaderSize = 8 * sizeof(unsigned int);
            memcpy(g_tVideoSrbWriterBuf.buffer, g_ptStreamHdr, bStrmHeaderSize);
        }
        else {
            bStrmHeaderSize = 9 * sizeof(unsigned int);
            memcpy(g_tVideoSrbWriterBuf.buffer, g_ptStreamHdr, bStrmHeaderSize);
            memcpy(g_tVideoSrbWriterBuf.buffer + bStrmHeaderSize, g_ptStreamHdr->abyVpsData, g_ptStreamHdr->dwVpsSize);
        }
        memcpy(g_tVideoSrbWriterBuf.buffer + bStrmHeaderSize + g_ptStreamHdr->dwVpsSize, g_ptStreamHdr->abySpsData, g_ptStreamHdr->dwSpsSize);
        memcpy(g_tVideoSrbWriterBuf.buffer + bStrmHeaderSize + g_ptStreamHdr->dwVpsSize + g_ptStreamHdr->dwSpsSize, g_ptStreamHdr->abyPpsData, g_ptStreamHdr->dwPpsSize);

    
    }

    while(SRB_WriterCheckReader(g_ptVideoSrbHandle, &g_tVideoSrbWriterBuf))
        usleep(10);

    SRB_SendGetWriterBuff(g_ptVideoSrbHandle, &g_tVideoSrbWriterBuf);
}

static int do_h26xe(VMF_FRAME_BUF_T* pVirtFrameBuf, VMF_FRAME_BUF_T* pPhysFrameBuf, void* ptEncBuff, unsigned int* bIdr,  FILE* fp)
{
    H26XE_OUTPUT_INFO_T info;
    int enc_bytes = 0, ret = 0;
    gettimeofday(&tv_prev, NULL);
    ret = h26xe_process(pVirtFrameBuf, pPhysFrameBuf, ptEncBuff, ptEncBuff_SIZE, &info);
    if(0 != ret){
        printf("[%s] h26xe_process error \n", __func__);
        return -1;
    }
    gettimeofday(&tv, NULL);
    //! Calculate time comsuming
    total_time = ((tv.tv_sec - tv_prev.tv_sec)*1000000+tv.tv_usec)-tv_prev.tv_usec;
    //printf("VMF_H26xEnc_ProcessOneFrame use %lu usecs.\n", total_time);
    enc_bytes = info.dwEncBytes;
    //! output stream
    if (g_bWriteEnable && enc_bytes) {
        fwrite(ptEncBuff, enc_bytes, 1, fp);
    }
    //printf("[do_h26xe] enc_bytes %d\n", enc_bytes);
    *bIdr = info.bIdr;
    return enc_bytes;
}

void* h26xenc_loop(void* data __attribute__((unused)))
{

    FILE* fp = NULL;
    char azOutPath[128];
    unsigned char* g_pbyOutRsFrame = NULL;
    struct timespec tVideoStartTime, tVideoEndTime, tCurrentTime;
    int  idiff =0, bStrmHeaderSize = 0, iEncSize = 0;;
    unsigned int dwEncCount = 0;
    unsigned int bIdr = 0;
    void* ptEncBuff = NULL;
    VMF_VIDEO_BUF_T src_buf;
	VMF_VIDEO_BUF_T	tRsOutbuf;
    VMF_FRAME_BUF_T Virt_frame_buf, Phys_frame_buf;
    


    ptEncBuff = MemBroker_GetMemory(ptEncBuff_SIZE, VMF_ALIGN_TYPE_DEFAULT);
    g_ptStreamHdr = (VMF_VENC_H265_STREAM_HDR *)malloc(sizeof(VMF_VENC_H265_STREAM_HDR));
    memset(g_ptStreamHdr, 0, sizeof(VMF_VENC_H265_STREAM_HDR));
    /*memset is for simulate CPU operate*/
    memset(&g_tInSsmBuf, 0, sizeof(SSM_BUFFER_T));
    memset(&g_tInSsmInfo, 0, sizeof(VMF_VSRC_SSM_OUTPUT_INFO_T));
    memset(g_ptStreamHdr, 0, sizeof(VMF_VENC_H265_STREAM_HDR));
    memset(&tVideoStartTime, 0, sizeof(struct timespec));
    memset(&tVideoEndTime, 0, sizeof(struct timespec));

    if (g_bWriteEnable) {
        sprintf(azOutPath, "stream_%ux%u.%s", codec_initopt.dwEncWidth, codec_initopt.dwEncHeight, g_eCodec == 0?"h264":"h265");
        fp = fopen(azOutPath, "w+");
        if(NULL == fp) {
            printf("Open %s file error!\n", azOutPath);
            g_bTerminate = 1;
        }
    }

    //! prepare codec init options.
    codec_initopt.dwCompressionRatio = g_tInSsmInfo.dwType;
    codec_initopt.dwSrcWidth = codec_initopt.dwEncWidth = g_dwTargetW;
    codec_initopt.dwSrcHeight = codec_initopt.dwEncHeight = g_dwTargetH;
    codec_initopt.dwSrcStride = ((codec_initopt.dwSrcWidth+31)&(~31));
    codec_initopt.dwCropX = codec_initopt.dwCropY = 0;

    if(0 != h26xe_init(&codec_initopt)){
        printf("h26xe_init failed\n");
    }

    h26xe_get_header_info((VMF_CODEC_TYPE)g_eCodec, g_ptStreamHdr);

    if(g_ptResizeHandle) {
        g_pbyOutRsFrame = (unsigned char *)MemBroker_GetMemory(g_dwTargetW * g_dwTargetH * 3 >> 1, VMF_ALIGN_TYPE_128_BYTE);
        if (!g_pbyOutRsFrame) {
            printf("[%s] Allocate resize output frame123 buffer fail !!\n",__func__);
        } 
        memset(g_pbyOutRsFrame, 0, sizeof(unsigned char)*g_dwTargetW * g_dwTargetH * 3 >> 1);
    }

    clock_gettime(CLOCK_MONOTONIC, &tVideoStartTime);
    //! Do encode process
    while (!g_bTerminate) {

        if(SSM_Reader_ReturnReceiveBuff(g_ptSsmReaderHandle, &g_tInSsmBuf))
        //if(SSM_Reader_ReturnReceiveNewestBuff(g_ptSsmReaderHandle, &g_tInSsmBuf, 0))
        {
            continue;
        }

        if (g_bTerminate) {
            break;
        }
        if(g_bConfFlag == true) {
            pthread_mutex_lock(&g_tVideoMutex);
            g_ptStreamHdr->dwConfFourCC = FOURCC_CONF;
            g_ptStreamHdr->dwH265FourCC = g_eCodec == 1 ? FOURCC_H265 : FOURCC_H264;
            g_ptStreamHdr->dwEncWidth = g_dwTargetW;
            g_ptStreamHdr->dwEncHeight = g_dwTargetH;
            if (g_eCodec == 1) {
                bStrmHeaderSize = 9 * sizeof(unsigned int);
                memcpy(g_tVideoSrbWriterBuf.buffer, g_ptStreamHdr, bStrmHeaderSize);
                memcpy(g_tVideoSrbWriterBuf.buffer + bStrmHeaderSize, g_ptStreamHdr->abyVpsData, g_ptStreamHdr->dwVpsSize);
            } else{
                bStrmHeaderSize = 8 * sizeof(unsigned int);
                memcpy(g_tVideoSrbWriterBuf.buffer, g_ptStreamHdr, bStrmHeaderSize);
            }
            memcpy(g_tVideoSrbWriterBuf.buffer + bStrmHeaderSize + g_ptStreamHdr->dwVpsSize, g_ptStreamHdr->abySpsData, g_ptStreamHdr->dwSpsSize);
            memcpy(g_tVideoSrbWriterBuf.buffer + bStrmHeaderSize + g_ptStreamHdr->dwVpsSize + g_ptStreamHdr->dwSpsSize, g_ptStreamHdr->abyPpsData, g_ptStreamHdr->dwPpsSize);
            MemBroker_CacheCopyBack(g_tVideoSrbWriterBuf.buffer, bStrmHeaderSize + g_ptStreamHdr->dwVpsSize + g_ptStreamHdr->dwSpsSize + g_ptStreamHdr->dwPpsSize);
            SRB_SendGetWriterBuff(g_ptVideoSrbHandle, &g_tVideoSrbWriterBuf);
            printf("Waiting\n");
            pthread_cond_wait(&g_tVideoCond, &g_tVideoMutex);
            pthread_mutex_unlock(&g_tVideoMutex);
            g_bConfFlag = false;
        }

        VMF_VSRC_SSM_GetInfo(g_tInSsmBuf.buffer, &g_tInSsmInfo);
        if(g_ptResizeHandle) {
			src_buf.apbyVirtAddr[0] = g_tInSsmBuf.buffer + g_tInSsmInfo.dwOffset[0];
			src_buf.apbyVirtAddr[1] = g_tInSsmBuf.buffer + g_tInSsmInfo.dwOffset[1];
			src_buf.apbyVirtAddr[2] = g_tInSsmBuf.buffer + g_tInSsmInfo.dwOffset[2];
			src_buf.apbyPhysAddr[0] = g_tInSsmBuf.buffer_phys_addr + g_tInSsmInfo.dwOffset[0];
			src_buf.apbyPhysAddr[1] = g_tInSsmBuf.buffer_phys_addr + g_tInSsmInfo.dwOffset[1];
			src_buf.apbyPhysAddr[2] = g_tInSsmBuf.buffer_phys_addr + g_tInSsmInfo.dwOffset[2];
			tRsOutbuf.apbyVirtAddr[0] = g_pbyOutRsFrame;
			tRsOutbuf.apbyVirtAddr[1] = tRsOutbuf.apbyVirtAddr[0] + (g_dwTargetW * g_dwTargetH);
			tRsOutbuf.apbyVirtAddr[2] = tRsOutbuf.apbyVirtAddr[1] + (g_dwTargetW * g_dwTargetH >> 2);
			tRsOutbuf.apbyPhysAddr[0] = (unsigned char *)MemBroker_GetPhysAddr(tRsOutbuf.apbyVirtAddr[0]);
			tRsOutbuf.apbyPhysAddr[1] = (unsigned char *)MemBroker_GetPhysAddr(tRsOutbuf.apbyVirtAddr[1]);
			tRsOutbuf.apbyPhysAddr[2] = (unsigned char *)MemBroker_GetPhysAddr(tRsOutbuf.apbyVirtAddr[2]);
			VMF_RS_ProcessOneFrame(g_ptResizeHandle, &tRsOutbuf, &src_buf);

            memset(&Virt_frame_buf, 0, sizeof(VMF_FRAME_BUF_T));
			Virt_frame_buf.apdwData[0] = g_pbyOutRsFrame;//tRsOutbuf.apbyVirtAddr[0];
			Virt_frame_buf.apdwData[1] = Virt_frame_buf.apdwData[0] + (g_dwTargetW * g_dwTargetH);
			Virt_frame_buf.apdwData[2] = Virt_frame_buf.apdwData[1] + (g_dwTargetW * g_dwTargetH >> 2);
            memset(&Phys_frame_buf, 0, sizeof(VMF_FRAME_BUF_T));
			Phys_frame_buf.apdwData[0] = (unsigned char *)MemBroker_GetPhysAddr(Virt_frame_buf.apdwData[0]);
			Phys_frame_buf.apdwData[1] = (unsigned char *)MemBroker_GetPhysAddr(Virt_frame_buf.apdwData[1]);
			Phys_frame_buf.apdwData[2] = (unsigned char *)MemBroker_GetPhysAddr(Virt_frame_buf.apdwData[2]);
        } else {
            memset(&Virt_frame_buf, 0, sizeof(VMF_FRAME_BUF_T));
            Virt_frame_buf.apdwData[0] = (unsigned char*) g_tInSsmBuf.buffer + g_tInSsmInfo.dwOffset[0];
            Virt_frame_buf.apdwData[1] = (unsigned char*) g_tInSsmBuf.buffer + g_tInSsmInfo.dwOffset[1];
            Virt_frame_buf.apdwData[2] = (unsigned char*) g_tInSsmBuf.buffer + g_tInSsmInfo.dwOffset[2];
        
            memset(&Phys_frame_buf, 0, sizeof(VMF_FRAME_BUF_T));
            Phys_frame_buf.apdwData[0] = (unsigned char*) g_tInSsmBuf.buffer_phys_addr + g_tInSsmInfo.dwOffset[0];
            Phys_frame_buf.apdwData[1] = (unsigned char*) g_tInSsmBuf.buffer_phys_addr + g_tInSsmInfo.dwOffset[1];
            Phys_frame_buf.apdwData[2] = (unsigned char*) g_tInSsmBuf.buffer_phys_addr + g_tInSsmInfo.dwOffset[2];
        }
        iEncSize = do_h26xe(&Virt_frame_buf, &Phys_frame_buf, ptEncBuff, &bIdr,fp);
        if(iEncSize == -1 ){
            printf("[h26xe] do_h26xe() failed.\n");				
            break;
        }

        VMF_VENC_STREAM_DATA_HDR *hdr = (VMF_VENC_STREAM_DATA_HDR *) g_tVideoSrbWriterBuf.buffer;
        clock_gettime(CLOCK_MONOTONIC, &tCurrentTime);
        hdr->dwFourCC = g_eCodec == 1 ? FOURCC_H265 : FOURCC_H264;
        hdr->dwFrameUSec = tCurrentTime.tv_nsec/1000;
        hdr->dwFrameSec = tCurrentTime.tv_sec;
        hdr->dwDataBytes = iEncSize;
        hdr->dwSeqNum = dwEncCount;
        hdr->bIsKeyFrame = bIdr? 1 : 0;

        memcpy(g_tVideoSrbWriterBuf.buffer + UBUFFER_HEADERSIZE , ptEncBuff, hdr->dwDataBytes);
        dwEncCount++;
        send_video_data(SEND_DATA);

        SSM_Reader_ReturnBuff(g_ptSsmReaderHandle, &g_tInSsmBuf);

        if (dwEncCount % 100 == 0) {
            clock_gettime(CLOCK_MONOTONIC, &tCurrentTime);
            idiff = (tCurrentTime.tv_sec*1000000 + tCurrentTime.tv_nsec/1000) - 
                    (tVideoStartTime.tv_sec*1000000 + tVideoStartTime.tv_nsec/1000);
#if 0
            printf("[h26xe] average %f ms per frame, total frame count: %d\r",((float)(idiff)/1000/100), dwEncCount); //!idiff / 1000 is conv us to ms, 100 is frame cnt
#endif
            tVideoStartTime.tv_sec = tCurrentTime.tv_sec;
            tVideoStartTime.tv_nsec = tCurrentTime.tv_nsec;
        }
    }

    if(fp)
        fclose(fp);

    if(0 != h26xe_release()){
        printf("h26xe_init failed\n");
    }
    free(g_ptStreamHdr);
    MemBroker_FreeMemory(ptEncBuff);
    g_bTerminate = 1;
    printf("[%s] quit successfully!\n", __func__);
    return NULL;
}

void H26xEnc_loadConfig( dictionary* ini )
{
    const char *tmp = NULL;

    do {
        if ((tmp = iniparser_getstring(ini, "venc:ssm_reader_pin", 0)) == NULL) {
            printf("Need ssm pin to read\n");
            break;
        }
        if(tmp)
            g_szSsmReaderPin = strdup(tmp);

        if ((tmp = iniparser_getstring(ini, "venc:srb_writer_pin", 0)) == NULL) {
            printf("Need srb pin to write\n");
            break;
        }
        if(tmp)
            g_szSrbWriterPin = strdup(tmp);

        g_eCodec = iniparser_getint(ini, "venc:codec_type", 0);
        g_bWriteEnable = iniparser_getint(ini, "venc:enanble_write", 0);
        g_dwTargetW =  iniparser_getint(ini, "venc:Resize_Width", 0);
        g_dwTargetH =  iniparser_getint(ini, "venc:Resize_Height", 0);
    } while(0);

    printf("[VENC] ssm_reader_pin: %s\r\n", g_szSsmReaderPin);
    printf("[VENC] srb_writer_pin: %s\r\n", g_szSrbWriterPin);
}

void msg_callback( MsgContext* msg_context, void* user_data )
{
    (void) user_data;

    printf("[h26xEncStrm] Video msg_context->pszHost=%s, msg_context->pszCmd=%s\n", msg_context->pszHost, msg_context->pszCmd);

    if ( !strcasecmp(msg_context->pszHost, "encoder0") )  {
        if ( !strcasecmp(msg_context->pszCmd, "start") )  {
            pthread_cond_signal(&g_tVideoCond);
        }
        else  if ( !strcasecmp(msg_context->pszCmd, "stop") )  {
            g_bConfFlag = true;
        }
        else  if ( !strcasecmp(msg_context->pszCmd, "forceCI") )  {
            send_video_data(SEND_CONF);
        }
        else  {
            // Do nothing
        }
    }
}

int init_resize_handle(unsigned int dwSrcW, unsigned int dwSrcH)
{
    VMF_RS_INITOPT_T init_opt;
    VMF_RS_CONFIG_T Config_Opt;
    memset(&init_opt, 0, sizeof(VMF_RS_INITOPT_T));
    memset(&Config_Opt, 0, sizeof(VMF_RS_CONFIG_T));
    init_opt.dwSrcWidth  = dwSrcW;
    init_opt.dwSrcHeight = dwSrcH;
    init_opt.dwSrcStride = dwSrcW;
    init_opt.eItpMode = ISP_RS_ITP_MODE_BICUBIC;           
    init_opt.bAntiAliasing = 1;
    init_opt.dwResizeNum = 1; 
    init_opt.pszParamsDir = "./Resource/ISP/0/";
    Config_Opt.dwDstWidth  = g_dwTargetW;
    Config_Opt.dwDstHeight = g_dwTargetH;
    Config_Opt.dwDstStride = g_dwTargetW;
    g_ptResizeHandle = VMF_RS_Init(&init_opt,&Config_Opt);

    if (!g_ptResizeHandle)
        return -1;
    printf("[%s] Successed.\n", __func__);
    return 0;
}

void ssm_clear_header(unsigned char* virt_addr, unsigned int buf_size, void* pUserData)
{
    VMF_VSRC_SSM_OUTPUT_INFO_T* vsrc_ssm_writer_info = (VMF_VSRC_SSM_OUTPUT_INFO_T*) pUserData;

    if (buf_size > VMF_MAX_SSM_HEADER_SIZE)
        memset(virt_addr, 0, VMF_MAX_SSM_HEADER_SIZE);

    VMF_VSRC_SSM_SetInfo(virt_addr, vsrc_ssm_writer_info);
}

void H26xEnc_IPC_Init( void )
{
    memset(&codec_initopt, 0, sizeof(VMF_CODEC_INITOPT_T));

    //! init SSM Reader Pin, YUV Source.
    g_ptSsmReaderHandle = SSM_Reader_Init(g_szSsmReaderPin);
    SSM_Reader_ReturnReceiveBuff(g_ptSsmReaderHandle, &g_tInSsmBuf);
    VMF_VSRC_SSM_GetInfo(g_tInSsmBuf.buffer, &g_tInSsmInfo);
    SSM_Reader_ReturnBuff(g_ptSsmReaderHandle, &g_tInSsmBuf);

    //! init SRB Writer Pin, Write Encoded data to srb ring. for RTSP
    g_ptVideoSrbHandle = SRB_InitWriter(g_szSrbWriterPin, 1048576, 4);
    if (!g_ptVideoSrbHandle) {
        printf(" [%s,%d] Video srb writer handle init error \n", __func__, __LINE__);
        exit(1);
    }
    memset(&g_tVideoSrbWriterBuf, 0, sizeof(SRB_BUFFER_T));

    SRB_SendGetWriterBuff(g_ptVideoSrbHandle, &g_tVideoSrbWriterBuf);

    if (g_dwTargetW == 0 || g_dwTargetH == 0) {
        g_dwTargetW = g_tInSsmInfo.dwWidth;
        g_dwTargetH = g_tInSsmInfo.dwHeight;
    }
    if ((g_tInSsmInfo.dwWidth != g_dwTargetW) && (g_tInSsmInfo.dwHeight != g_dwTargetH) &&
            init_resize_handle(g_tInSsmInfo.dwWidth, g_tInSsmInfo.dwHeight)) {
        printf("[%s] Initial resize handle failed !!\n", __func__);
    }
}

void H26xEnc_IPC_Release( void )
{
    pthread_cond_destroy(&g_tVideoCond);
    pthread_mutex_destroy(&g_tVideoMutex);
    free(g_szSsmReaderPin);
    free(g_szSrbWriterPin);
    if(g_ptVideoSrbHandle != NULL)
        SRB_Release(g_ptVideoSrbHandle);
    if (g_ptSsmReaderHandle) {
        if (g_tInSsmBuf.buffer) {        // ★ 先還最後一顆（若有）
            SSM_Reader_ReturnBuff(g_ptSsmReaderHandle, &g_tInSsmBuf);
            memset(&g_tInSsmBuf, 0, sizeof(g_tInSsmBuf));
        }
        SSM_Release(g_ptSsmReaderHandle);
        g_ptSsmReaderHandle = NULL;      // 已有這行 👍
    }
    if (g_ptResizeHandle)
        VMF_RS_Release(g_ptResizeHandle);
}

void H26xEnc_IPC_Wake( void )
{
    pthread_cond_signal(&g_tVideoCond);
    if (g_ptSsmReaderHandle)
        SSM_Reader_Wakeup(g_ptSsmReaderHandle);
}

/**
 * @brief 將 BGR 格式的 cv::Mat 轉換為 YUV420 平面格式的位元組向量。
 *
 * @param bgr_image 輸入的 BGR cv::Mat 影像。
 * @param yuv_bytes 輸出 YUV420 位元組的向量。
 */
void bgrToYUV420(const cv::Mat& bgr_image, std::vector<unsigned char>& yuv_bytes)
{
    // 檢查影像是否為空
    if (bgr_image.empty()) {
        std::cerr << "輸入影像為空！" << std::endl;
        return;
    }

    // 檢查寬度和高度是否為偶數，YUV420 要求
    if (bgr_image.cols % 2 != 0 || bgr_image.rows % 2 != 0) {
        std::cerr << "影像的寬度和高度必須為偶數以轉換為 YUV420 格式！" << std::endl;
        return;
    }
    
    // 建立一個空的 Mat 物件來儲存 YUV 轉換結果
    cv::Mat yuv_image;

    // 使用 cv::cvtColor 函式將 BGR 轉換為 YUV_I420
    // 轉換後的 yuv_image 是一個單一的 Mat，其中 YUV 資料是堆疊排列的
    //fprintf(stderr, "[DEBUG] bgrToYUV420\n");
    cv::cvtColor(bgr_image, yuv_image, cv::COLOR_BGR2YUV_I420);

    // 將 Mat 物件的資料複製到 std::vector<unsigned char>
    size_t total_size = yuv_image.total() * yuv_image.elemSize();
    yuv_bytes.assign(yuv_image.data, yuv_image.data + total_size);
}

void YUV422To420P( char *yuv422, char *yuv420, int i32Height, int i32Width )
{
    if ( yuv422 == NULL || yuv420 == NULL )  {
        return;
    }

    char *y = NULL;
    char *u = NULL;
    char *v = NULL;
    int i = 0, j = 0, z = 0, getUvFlag = 0;

    char *start = yuv422;

    y = yuv420;
    u = y + i32Height * i32Width;
    v = u + i32Height * i32Width / 4;

    for ( int row = 0; row < i32Width * i32Height * 2; )  {
        y[i++] = start[row];
        y[i++] = start[row + 2];

        if ( getUvFlag % 2 == 0 )  {
            u[j++] = start[row + 1];	//u
            v[z++] = start[row + 3];	//v
        }

        row += 4;

        if ( row % (i32Width * 2) == 0 )  {
            getUvFlag++;
        }
    }
}
#endif

//Coco dataset class name
static const char* COCO80[80] = {
 "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
 "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog",
 "horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
 "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite",
 "baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
 "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
 "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant",
 "bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone",
 "microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors",
 "teddy bear","hair drier","toothbrush"
};

int draw_display_image(cv::Mat *cv_img_display, const char *strImgFPS, const char *strInfFPS)
{
    int ret = KP_SUCCESS;

    pthread_mutex_lock(&_mutex_result);
    if (_inf_result.result_buffer == 0) {
        pthread_mutex_unlock(&_mutex_result);
        return KP_SUCCESS;
    }

    auto *header_stamp = reinterpret_cast<kp_inference_header_stamp_t*>((void*)_inf_result.result_buffer);

    if (KDP2_INF_ID_APP_YOLO == header_stamp->job_id) {
        auto *res = reinterpret_cast<kdp2_ipc_app_yolo_result_t*>((void*)_inf_result.result_buffer);
        const kp_app_yolo_result_t &yd = res->yolo_data;

        for (uint32_t i = 0; i < yd.box_count; ++i) {
            const kp_bounding_box_t &b = yd.boxes[i];

            // 轉 int 並做邊界保護
            int x1 = std::max(0, std::min((int)std::lround(b.x1), cv_img_display->cols - 1));
            int y1 = std::max(0, std::min((int)std::lround(b.y1), cv_img_display->rows - 1));
            int x2 = std::max(0, std::min((int)std::lround(b.x2), cv_img_display->cols - 1));
            int y2 = std::max(0, std::min((int)std::lround(b.y2), cv_img_display->rows - 1));

            // 畫框
            cv::rectangle(*cv_img_display, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(50, 255, 50), 2);

            // 準備文字：類別名稱 + 置信度（%）
            const char *name = (b.class_num >= 0 && b.class_num < 80) ? COCO80[b.class_num] : "cls";
            char label[128];
            std::snprintf(label, sizeof(label), "%s %.1f%%", name, b.score * 100.0f);

            // 計算文字大小，畫底色條，寫字
            int baseline = 0;
            cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            int tx = x1;
            int ty = std::max(y1 - 5, ts.height + 5); // 儘量畫在框上方；不夠就往下擠

            cv::rectangle(*cv_img_display,
                          cv::Point(tx, ty - ts.height - baseline),
                          cv::Point(tx + ts.width + 2, ty + 2),
                          cv::Scalar(50, 255, 50), cv::FILLED);   // 底色
            cv::putText(*cv_img_display, label, cv::Point(tx + 1, ty),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);   // 黑字
        }
        pthread_mutex_unlock(&_mutex_result);

        // 其他資訊
        cv::putText(*cv_img_display, strImgFPS, {5, 30},  cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, {50, 50, 255}, 1);
        cv::putText(*cv_img_display, strInfFPS, {5, 60},  cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, {50, 50, 255}, 1);
        cv::putText(*cv_img_display, "Press 'ESC' to exit",
                    {10, cv_img_display->rows - 10}, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, {255,255,255}, 2);
    } else {
        pthread_mutex_unlock(&_mutex_result);
        ret = KP_FW_ERROR_UNKNOWN_APP;
    }

    return ret;
}

cv::Mat getNearestHomography(double distance, const std::map<double, cv::Mat>& table) {
    if (table.empty()) return cv::Mat(); // 防呆：空表直接回傳空 Mat

    double nearestKey = table.begin()->first;
    double minDiff = std::numeric_limits<double>::infinity();

    for (const auto& kv : table) {
        double diff = std::abs(kv.first - distance);  // ← 用 distance
        if (diff < minDiff) {
            minDiff = diff;
            nearestKey = kv.first;
        }
    }
    return table.at(nearestKey);
}

// std::map<double, cv::Mat> homographyTable = {
//     {1, (cv::Mat_<double>(3, 3) <<
//         0.9050877550, 0.0058800048, 3.9792681439,
//        -0.0250777811, 0.8513578223, 26.1397211293,
//        -0.0000255637, -0.0000405858, 1.0000000000)},

//     {2, (cv::Mat_<double>(3, 3) <<
//         0.8807876710, -0.0147831469, 21.6523893584,
//        -0.0354529114, 0.8008935278, 42.2526872709,
//        -0.0000278898, -0.0001173266, 1.0000000000)},

//     {3, (cv::Mat_<double>(3, 3) <<
//         0.8777235528, 0.0071806216, 22.2211301414,
//        -0.0441135165, 0.8420880947, 36.5511533147,
//        -0.0000483255, -0.0000401510, 1.0000000000)},

//     {4, (cv::Mat_<double>(3, 3) <<
//         0.9149316782, -0.0240853016, 23.8535958896,
//        -0.0058000350, 0.8149579421, 35.3156708921,
//         0.0000599728, -0.0001384213, 1.0000000000)},

//     {5, (cv::Mat_<double>(3, 3) <<
//         0.9148623383, -0.0079911622, 20.6747210237,
//        -0.0079176662, 0.8322671645, 29.4755315504,
//         0.0000399572, -0.0001021884, 1.0000000000)},

//     {6, (cv::Mat_<double>(3, 3) <<
//         0.8512227234, -0.0179970058, 34.5509577506,
//        -0.0374102606, 0.7897388373, 43.4700381765,
//        -0.0000772120, -0.0000686909, 1.0000000000)},

//     {7, (cv::Mat_<double>(3, 3) <<
//         0.9166666667, 0.0000000000, 24.0000000000,
//        -0.0075757576, 0.8481012658, 30.3394706559,
//         0.0000000000, 0.0000000000, 1.0000000000)},

//     {8, (cv::Mat_<double>(3, 3) <<
//         0.9738825790, -0.0376271078, 19.3739271390,
//         0.0159536117, 0.8248146004, 30.0979986041,
//         0.0001507288, -0.0001419891, 1.0000000000)},

//     {9, (cv::Mat_<double>(3, 3) <<
//         0.7970977742, -0.0415502360, 45.8786164360,
//        -0.0585435213, 0.7550767983, 48.6234080788,
//        -0.0001444276, -0.0001579857, 1.0000000000)},

//     {10, (cv::Mat_<double>(3, 3) <<
//         0.9452650849, 0.0000000000, 19.9684338996,
//         0.0071635614, 0.9023301473, 12.0733867063,
//         0.0000331646, 0.0000000000, 1.0000000000)},

//     {12.5, (cv::Mat_<double>(3, 3) <<
//         0.9047619048, 0.0000000000, 29.0000000000,
//         0.0000000000, 0.8684210526, 19.1315789474,
//         0.0000000000, 0.0000000000, 1.0000000000)},

//     {15, (cv::Mat_<double>(3, 3) <<
//         0.8888888889, 0.0000000000, 35.5555555556,
//         0.0000000000, 0.8709677419, 18.3225806452,
//         0.0000000000, 0.0000000000, 1.0000000000)},

//     {20, (cv::Mat_<double>(3, 3) <<
//         0.9268292683, 0.0000000000, 20.9512195122,
//         0.0000000000, 0.8800000000, 15.4800000000,
//         0.0000000000, 0.0000000000, 1.0000000000)},

//     {25, (cv::Mat_<double>(3, 3) <<
//         0.9090909091, 0.0000000000, 27.8181818182,
//         0.0000000000, 0.9473684211, 1.4736842105,
//         0.0000000000, 0.0000000000, 1.0000000000)},

//     {30, (cv::Mat_<double>(3, 3) <<
//         0.9259259259, 0.0000000000, 23.2962962963,
//         0.0000000000, 0.9375000000, -0.6875000000,
//         0.0000000000, 0.0000000000, 1.0000000000)},
// };

#ifndef INC_MTX_ARR	//JDBG
static bool loadHomographyTable(const std::string& path, std::map<double, cv::Mat>& table) {
    std::ifstream fin(path);
    if (!fin) return false;
    std::string content((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());

    // 每個 entry 形如：{ dist, (cv::Mat_<double>(3, 3) << a,b,c, d,e,f, g,h,i) },
    // 先抓到每個 { ... } 區塊，再：
    //   1) 第一個數字當 distance
    //   2) 從 '<<' 之後抓 9 個數當 H（跳過 (3, 3)）
    std::regex entry_re("\\{[^\\{\\}]*\\}");
    std::regex num_re("[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?");

    table.clear();
    bool any_ok = false;

    auto it  = std::sregex_iterator(content.begin(), content.end(), entry_re);
    auto end = std::sregex_iterator();
    for (; it != end; ++it) {
        const std::string block = it->str();

        // distance = 第一個數字
        std::smatch m;
        if (!std::regex_search(block, m, num_re)) continue;
        double dist = std::stod(m.str());

        // 找 '<<' 後面的 9 個數字
        size_t p = block.find("<<");
        std::vector<double> h;
        if (p != std::string::npos) {
            const std::string tail = block.substr(p);
            for (auto nb = std::sregex_iterator(tail.begin(), tail.end(), num_re);
                 nb != std::sregex_iterator() && h.size() < 9; ++nb) {
                h.push_back(std::stod((*nb).str()));
            }
        }
        // 後援：萬一找不到 '<<'，就嘗試拿 block 中「距離之後」的 9 個數
        if (h.size() < 9) {
            std::vector<double> all;
            for (auto nb = std::sregex_iterator(block.begin(), block.end(), num_re);
                 nb != std::sregex_iterator(); ++nb) {
                all.push_back(std::stod((*nb).str()));
            }
            if (all.size() >= 10) {
                h.assign(all.begin()+1, all.begin()+10); // 跳過 distance，取 9 個
            }
        }
        if (h.size() != 9) continue;

        cv::Mat H(3,3,CV_64F);
        for (int i=0;i<9;++i) H.at<double>(i/3, i%3) = h[i];
        table[dist] = H;
        any_ok = true;
    }
    return any_ok && !table.empty();
}

static bool loadCameraParams(const std::string& path, cv::Mat& K, cv::Mat& D) {
    std::ifstream fin(path);
    if (!fin) return false;

    std::string content((std::istreambuf_iterator<char>(fin)),
                        std::istreambuf_iterator<char>());

    std::regex num_re("[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?");

    // ---- 1) 先處理 mtx 那段 ----
    size_t mtx_pos = content.find("mtx");
    if (mtx_pos == std::string::npos) return false;
    size_t mtx_end = content.find(';', mtx_pos);
    if (mtx_end == std::string::npos) return false;

    std::string mtx_block = content.substr(mtx_pos, mtx_end - mtx_pos + 1);
    std::vector<double> k_vals;
    for (auto it = std::sregex_iterator(mtx_block.begin(), mtx_block.end(), num_re);
         it != std::sregex_iterator(); ++it) {
        k_vals.push_back(std::stod((*it).str()));
    }
    if (k_vals.size() < 9) return false;

    // 若有 (3,3)，就會多兩個數字，所以取「最後 9 個」
    int k_start = (int)k_vals.size() - 9;
    K = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        K.at<double>(i/3, i%3) = k_vals[k_start + i];
    }

    // ---- 2) 再處理 dist 那段 ----
    size_t dist_pos = content.find("dist");
    if (dist_pos == std::string::npos) return false;
    size_t dist_end = content.find(';', dist_pos);
    if (dist_end == std::string::npos) return false;

    std::string dist_block = content.substr(dist_pos, dist_end - dist_pos + 1);
    std::vector<double> d_vals;
    for (auto it = std::sregex_iterator(dist_block.begin(), dist_block.end(), num_re);
         it != std::sregex_iterator(); ++it) {
        d_vals.push_back(std::stod((*it).str()));
    }
    if (d_vals.size() < 5) return false;

    int d_start = (int)d_vals.size() - 5;
    D = cv::Mat(1, 5, CV_64F);
    for (int j = 0; j < 5; ++j) {
        D.at<double>(0, j) = d_vals[d_start + j];
    }

    return true;
}
#endif

void *example_display_liveview_thread(void *)
{
    // ensure_output_dirs(); // 1011 刪除
    ProfilerWindow prof;                 // <--- 新增： profiler 視窗
    mkdir_p("/work/logs");                // 多一個 logs 目錄（如果你想放在 /work/logs）
    prof.open_log("/work/logs/profiler.txt");   // ← 這行決定輸出檔路徑
    //-------------計時用的-------------------
    using std::chrono::steady_clock;
    auto frame_begin = steady_clock::now();
    //-------------計時用的 end -------------------

    time_t last_save_time = 0;  // 上一次儲存時間（秒）
    char filename_sensor[128];
    char filename_webcam[128];

    struct timeval time_begin;
    struct timeval time_end;
    float time_spent = 0.0;
    char strImgFPS[50] = "Image FPS: ";
    char strInfFPS[50] = "Inference FPS: ";
    char strDistance[50] = "Distance: ";
    char strQuality[50] = "Quality: ";
    cv::Mat cv_image_source;
    cv::Mat cv_image_display;
    

    //webcam
    cv::Mat cv_image_source_WEBCAM;
    cv::Mat cv_image_display_WEBCAM;

//internet streaming
#ifdef H26XE_API_H
    SSM_BUFFER_T tWriterSsmBuffer;
    std::vector<unsigned char> yuv_data;

    VMF_VSRC_SSM_OUTPUT_INFO_T vsrc_ssm_writer_info;

    vsrc_ssm_writer_info.dwYStride = CAM_WIDTH;
    vsrc_ssm_writer_info.dwYSize = vsrc_ssm_writer_info.dwYStride * CAM_HEIGHT;
    vsrc_ssm_writer_info.dwUVSize = vsrc_ssm_writer_info.dwYSize >> 2;
    vsrc_ssm_writer_info.dwOffset[0] = VMF_MAX_SSM_HEADER_SIZE;
    vsrc_ssm_writer_info.dwOffset[1] = vsrc_ssm_writer_info.dwOffset[0] + vsrc_ssm_writer_info.dwYSize;
    vsrc_ssm_writer_info.dwOffset[2] = vsrc_ssm_writer_info.dwOffset[1] + vsrc_ssm_writer_info.dwUVSize;
    vsrc_ssm_writer_info.dwWidth = CAM_WIDTH;
    vsrc_ssm_writer_info.dwHeight = CAM_HEIGHT;

    memset(&ssm_opt, 0, sizeof(SSM_WRITER_INIT_OPTION_T));
    ssm_opt.name = g_szSsmReaderPin;
    ssm_opt.buf_size = (CAM_WIDTH * CAM_HEIGHT * 3 / 2) + VMF_MAX_SSM_HEADER_SIZE;
    ssm_opt.alignment = VMF_ALIGN_TYPE_DEFAULT;
    ssm_opt.pUserData = &vsrc_ssm_writer_info;
    ssm_opt.fp_setup_buffer = ssm_clear_header;
    g_ptSsmWriterHandle = SSM_Writer_Init(&ssm_opt);
    if (!g_ptSsmWriterHandle)
    {
        printf("init g_ptSsmWriterHandle failed\n");
    }
#endif
    // cv::namedWindow("Inference Display", cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
    // cv::namedWindow("Inference Display", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    // cv::namedWindow("Webcam Display", cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);

#ifndef INC_MTX_ARR	//JDBG
    //load parameters
    cv::Mat K_webcam ,D_webcam;
    if (!loadCameraParams(camera_params_path, K_webcam, D_webcam)) {
    std::cerr << "[CAM] Failed to load camera params from "
                << camera_params_path << ", please check file format.\n";
    }
    std::map<double, cv::Mat> homographyTable;
    if (!loadHomographyTable(homography_table_path, homographyTable)) {
        printf("Failed to load homography table path\n");
    }
#endif

    gettimeofday(&time_begin, NULL);

#ifdef TOF_EN
    TOFSensor tof("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0",B115200);    // 建立TOF物件為tof
        if (!tof.initialize()) {             // 初始化tof
        std::cerr << "failed to initilize TOF\n";
        return NULL;
    }

    tof.start();     // 啟動tof
#endif

    // Webmin 啟動 FIFO 控制執行緒（提供 Webmin 控制）
    pthread_t th_ctl;
    pthread_create(&th_ctl, nullptr, ctrl_fifo_thread, nullptr);
    pthread_detach(th_ctl);

    // === 0822 搖桿：I2C ADC + 按鈕 GPIO 初始化 ===
    const char* used_i2c = nullptr;
    int i2c_fd = i2c_open_ads(&used_i2c); // 嘗試 /dev/i2c-0，失敗再試 1
    if (i2c_fd < 0) {
        std::cerr << "[JOYSTICK] open i2c device failed. please check i2c-dev and wiring\n";
    } else {
        std::cout << "[JOYSTICK] using " << used_i2c << " addr 0x" 
                << std::hex << ADS1115_ADDR << std::dec << std::endl;
    }

    // 0924 更新 GPIO初始化
    gpio_export(GPIO_BTN_RESET); gpio_set_direction_in(GPIO_BTN_RESET);
    gpio_export(GPIO_BTN_ALPHA); gpio_set_direction_in(GPIO_BTN_ALPHA);
    gpio_export(GPIO_BTN_CMAP ); gpio_set_direction_in(GPIO_BTN_CMAP );
    gpio_export(GPIO_BTN_SNAP ); gpio_set_direction_in(GPIO_BTN_SNAP );
    // 0924 更新 GPIO初始化END


    // 以 ADC 初讀的中位做為中心校正
    int16_t x_center = 0, y_center = 0;
    if (i2c_fd >= 0) {
        // 讀幾次取平均，做為中心
        const int N = 8;
        int sumx=0, sumy=0;
        for (int i=0;i<N;i++){ sumx += ads1115_read_channel(i2c_fd, 0); sumy += ads1115_read_channel(i2c_fd, 1); usleep(2000); }
        x_center = (int16_t)(sumx / N);
        y_center = (int16_t)(sumy / N);
        std::cout << "[JOYSTICK] center: X=" << x_center << ", Y=" << y_center << std::endl;
    }

    // 0924 讀取config設定
    // 1) 目錄先確保存在（/work/config）
    ensure_parent_dir(g_cfg.path);

    // 2) 如檔案不存在，先用目前的預設狀態「建立一份」
    if (!file_exists(g_cfg.path)) {
        // 預設值：10 分鐘；目前的 dx,dy,cmap,alpha_idx 也寫進去
        g_cfg.save_cfg_minutes = 10;

        // 這裡的 dx,dy,cmap_idx,alpha_idx 是你目前程式的全域狀態
        // （如果你想開機一律從 0,0 開始，把 dx,dy 改成 0,0 即可）
        config_save(g_cfg, dx, dy, cmap_idx, alpha_idx);
    }

    // 3) 載入檔案（若上面剛建立，就會載入剛寫入的預設）
    config_load(g_cfg);
    ensure_dirs_from_cfg(g_cfg); // 1011 新增載入設定後，確保資料夾

    // 4) 套用到執行中的狀態（下次開啟會還原到上次使用的狀態）
    dx = g_cfg.dx;
    dy = g_cfg.dy;
    cmap_idx  = clamp_compat(g_cfg.cmap, 0, 5);
    alpha_idx = clamp_compat(g_cfg.alpha_idx, 0, 4);
    alpha_ir  = kAlphaLevels[alpha_idx];

    // （你原本就有的週期回存計時器）
    auto last_cfg_save = std::chrono::steady_clock::now();
    // 0924 讀取config設定 END

    // === 1011 設定模式狀態 ===
    bool setting_mode = false;          // false=一般模式；true=設定模式
    int  setting_sel  = 0;              // 0=IR 透明度檔位；1=CMAP 映射
    auto setting_last_activity = std::chrono::steady_clock::now(); // 最近一次互動時間（按鍵或搖桿）
    auto mark_activity = [&](){ setting_last_activity = std::chrono::steady_clock::now(); };
    // === 1011 設定模式狀態 END ===

    while (true == _blDisplayRunning) 
    {
        applyPendingControls();     // ← Webmin 每幀把外部命令套入 dx/dy/alpha_ir/cmap_idx
#ifdef H26XE_API_H
        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);
#endif
        {
            // ---- 計時用的 TOTAL（每一圈總時間）----
            ScopeTimer T_total(&prof, "TOTAL");

#ifdef TOF_EN
            DistanceData distancedate;
#endif
            {
                ScopeTimer T_sensor(&prof, "Image_Sensor_format");
                pthread_mutex_lock(&_mutex_image);
                switch (_input_data.input_image_format) {
                case KP_IMAGE_FORMAT_RGB565:
                    cv_image_source = cv::Mat(_input_data.input_image_height, _input_data.input_image_width, CV_8UC2, (void *)_input_data.input_buf_address);
                    cv::cvtColor(cv_image_source, cv_image_display, cv::COLOR_BGR5652BGR);
                    break;
                case KP_IMAGE_FORMAT_RGBA8888:
                    cv_image_source = cv::Mat(_input_data.input_image_height, _input_data.input_image_width, CV_8UC4, (void *)_input_data.input_buf_address);
                    cv::cvtColor(cv_image_source, cv_image_display, cv::COLOR_RGBA2BGR);
                    break;
                case KP_IMAGE_FORMAT_YUV420:
                    cv_image_source = cv::Mat(_input_data.input_image_height * 1.5, _input_data.input_image_width, CV_8UC1, (void *)_input_data.input_buf_address);
                    cv::cvtColor(cv_image_source, cv_image_display, cv::COLOR_YUV2BGR_I420);
                    break;
                default:
                    cv_image_display = cv::Mat();
                    break;
                }
                pthread_mutex_unlock(&_mutex_image);
            }
            {
                ScopeTimer T_webcam(&prof, "Webcam_format");
                pthread_mutex_lock(&_mutex_image_webcam);
                switch (_input_data_webcam.input_image_format) {
                case KP_IMAGE_FORMAT_RGB565:
                    cv_image_source_WEBCAM = cv::Mat(_input_data_webcam.input_image_height, _input_data_webcam.input_image_width, CV_8UC2, (void *)_input_data_webcam.input_buf_address);
                    cv::cvtColor(cv_image_source_WEBCAM, cv_image_display_WEBCAM, cv::COLOR_BGR5652BGR);
                    break;
                case KP_IMAGE_FORMAT_RGBA8888:
                    cv_image_source_WEBCAM = cv::Mat(_input_data_webcam.input_image_height, _input_data_webcam.input_image_width, CV_8UC4, (void *)_input_data_webcam.input_buf_address);
                    cv::cvtColor(cv_image_source_WEBCAM, cv_image_display_WEBCAM, cv::COLOR_RGBA2BGR);
                    break;
                case KP_IMAGE_FORMAT_YUV420:
                    cv_image_source_WEBCAM = cv::Mat(_input_data_webcam.input_image_height * 1.5, _input_data_webcam.input_image_width, CV_8UC1, (void *)_input_data_webcam.input_buf_address);
                    cv::cvtColor(cv_image_source_WEBCAM, cv_image_display_WEBCAM, cv::COLOR_YUV2BGR_I420);
                    break;
                default:
                    cv_image_display_WEBCAM = cv::Mat();
                    break;
                }

                pthread_mutex_unlock(&_mutex_image_webcam);
            }

            /* Display image */
            if (false == cv_image_display.empty()) {
                if (true == _inf_result.result_ready_display) {
                    //---- 計時用的的YOLO 畫框 ----
                    ScopeTimer T_yolo(&prof, "YOLO_Draw");
#ifdef TOF_EN
                    distancedate = tof.getTofDistance();
                    sprintf(strDistance, "Distance: %.2lf", distancedate.range);
                    sprintf(strQuality, "Quality: %d", distancedate.quality);
#endif
                    draw_display_image(&cv_image_display, strDistance, strQuality);
                }

                // cv::imshow("Inference Display", cv_image_display);
            }
            // -------------------------------------
            // [IR 去畸變主流程：remap]
            // -------------------------------------
            if (false == cv_image_display_WEBCAM.empty()) {
                if (true == _input_data_webcam.input_ready_inf) {
                    // (1) 按硬體安裝方向先旋轉（與你原碼一致）
                    ScopeTimer T_remap(&prof, "Webcam_Remap");
                    cv::rotate(cv_image_display_WEBCAM,
                            cv_image_display_WEBCAM,
                            cv::ROTATE_90_CLOCKWISE);

                    // (2) 首次或尺寸改變時，重建 remap 查表
                    if (!map_ready || map1_web.size() != cv_image_display_WEBCAM.size()) {
                        const cv::Size sz = cv_image_display_WEBCAM.size();

                        // 2-1 由 K/D 與影像尺寸求「新相機矩陣」
                        newK_web = cv::getOptimalNewCameraMatrix(
                            K_webcam, D_webcam, sz, undist_alpha);

                        // 2-2 生成 map1/map2（CV_16SC2 較快、精度足夠）
                        cv::initUndistortRectifyMap(
                            K_webcam, D_webcam,
                            cv::noArray(),     // 不額外做 rectification 旋轉
                            newK_web,          // P
                            sz,
                            CV_16SC2,
                            map1_web, map2_web
                        );
                        map_ready = true;
                    }

                    // (3) 對當前畫面做重映射（去畸變）
                    cv::remap(cv_image_display_WEBCAM, cv_image_display_WEBCAM,
                            map1_web, map2_web, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

                    // cv::imshow("Webcam Display", cv_image_display_WEBCAM);
                }
            }

            // if (false == cv_image_display_WEBCAM.empty()) {
            //     if (true == _input_data_webcam.input_ready_inf){
            //         cv::rotate(cv_image_display_WEBCAM, cv_image_display_WEBCAM, cv::ROTATE_90_CLOCKWISE);
            //         cv_image_display_WEBCAM = undistortImage(cv_image_display_WEBCAM, /*alpha=*/0.0, /*crop=*/true);
            //         cv::imshow("Webcam Display", cv_image_display_WEBCAM);
            //     }
                
            // }

            cv::Mat fused_display;
            // ======= 疊合 webcam(IR) 到 sensor(RGB) ======= //
            if (!cv_image_display.empty() && !cv_image_display_WEBCAM.empty()) {
                {
#ifdef TOF_EN
                    DistanceData fuseddistance;
#endif
                    { 
                        ScopeTimer T_tof(&prof, "TOF_get");
#ifdef TOF_EN
                        fuseddistance = tof.getTofDistance();
                        // std::cout << "Time: " << fuseddistance.timestamp << "s, Range: " << fuseddistance.range
                        //     << " m, Amp: " << fuseddistance.amplitude
                        //     << ", Quality: " << fuseddistance.quality
                        //     << ", Status: " << fuseddistance.status << std::endl;
#endif
                        
                    }

                    cv::Mat H;

                    { 
                        ScopeTimer T_hom(&prof, "getNearestHomography");
#ifdef TOF_EN
                        H = getNearestHomography(fuseddistance.range, homographyTable);
#else
                        H = getNearestHomography(/*fuseddistance.range*/2, homographyTable);
#endif
                    }
                    // 加上平移矩陣
                    cv::Mat translation = (cv::Mat_<double>(3,3) <<
                        1, 0, dx,
                        0, 1, dy,
                        0, 0, 1);
                    cv::Mat H_adjusted = translation * H; // 注意乘法順序！

                    // === 0830 以目前選擇的 colormap 轉彩 ===
                    cv::Mat ir_gray, ir_color;
                    {
                        ScopeTimer T_colormap(&prof, "Colormap");
                        cv::cvtColor(cv_image_display_WEBCAM, ir_gray, cv::COLOR_BGR2GRAY);
                        cv::applyColorMap(ir_gray, ir_color, kColorMaps[cmap_idx]);
                    }
                    // warp 到 sensor 對齊（尺寸以 sensor 為準）
                    cv::Mat ir_warped;
                    {
                        ScopeTimer T_warp(&prof, "Warp");
                        cv::warpPerspective(ir_color, ir_warped, H_adjusted, cv_image_display.size());

                        // === 0824 疊合：語意化「IR 透明度」===
                        // 使 Camera 權重 = (1 - alpha_ir)、IR 權重 = alpha_ir，保持總權重為 1
                        cv::addWeighted(cv_image_display, (1.0 - alpha_ir), ir_warped, alpha_ir, 0.0, fused_display);
                    

                    // HUD：顯示目前透明度與 colormap 名稱
                        // === 1011更新 HUD ===
                        if (setting_mode) {
                            // 設定模式：顯示三行，並加粗目前選項
                            int base_y = 28;
                            cv::putText(fused_display, "SETUP MODE", cv::Point(10, base_y),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0,255,255), 2);

                            int th_alpha = (setting_sel==0) ? 3 : 1;
                            int th_cmap  = (setting_sel==1) ? 3 : 1;

                            char line1[64]; std::snprintf(line1, sizeof(line1), "IR alpha: %d%%", alpha_idx*25);
                            cv::putText(fused_display, line1, cv::Point(10, base_y+32),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255,255,255), th_alpha);

                            char line2[64]; std::snprintf(line2, sizeof(line2), "CMAP: %s", kColorMapNames[cmap_idx]);
                            cv::putText(fused_display, line2, cv::Point(10, base_y+64),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255,255,255), th_cmap);

                        } else {
                            // 一般模式：維持你原本的一行 HUD
                            char hud[128];
                            std::snprintf(hud, sizeof(hud), "IR alpha: %d%%  |  CMAP: %s  |  dx=%d dy=%d",
                                        (int)std::round(alpha_ir*100.0), kColorMapNames[cmap_idx], dx, dy);
                            cv::putText(fused_display, hud, cv::Point(10, 28),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 2);
                        }
                        // === 1011更新 HUD END===
                    }
                }

                // 顯示
                {
                    ScopeTimer T_show(&prof, "Display_imshow");
                    cv::imshow("Fused Display", fused_display);
#ifdef H26XE_API_H
            bgrToYUV420(fused_display, yuv_data);

            memcpy(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, yuv_data.data(), yuv_data.size());
            MemBroker_CacheFlush(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, yuv_data.size());
#endif
                }

                // 1011 每秒存一張到 buffer + 自動刪除過舊
                { 
                    static time_t last_buf_sec = 0;
                    time_t now_sec = time(NULL);
                    if (now_sec != last_buf_sec) {
                        last_buf_sec = now_sec;
                        // 只有在有取得影像時才存 buffer
                        if (!cv_image_display.empty() || !cv_image_display_WEBCAM.empty() || !fused_display.empty()) {
                            std::string ts = ts_string(now_sec);

                            // 準備目標檔名（都放在 buffer/ 子資料夾）
                            std::string p_rgb   = join_path(join_path(g_cfg.save_dir_rgb,   "buffer"), "rgb_"   + ts + ".jpg");
                            std::string p_ir    = join_path(join_path(g_cfg.save_dir_ir,    "buffer"), "ir_"    + ts + ".jpg");
                            std::string p_fused = join_path(join_path(g_cfg.save_dir_fused, "buffer"), "fused_" + ts + ".jpg");

                            // 分別寫入（哪個 Mat 有資料就存哪個）
                            if (!cv_image_display.empty())        cv::imwrite(p_rgb,   cv_image_display);
                            if (!cv_image_display_WEBCAM.empty()) cv::imwrite(p_ir,    cv_image_display_WEBCAM);
                            if (!fused_display.empty())           cv::imwrite(p_fused, fused_display);

                            g_ring.push_back({now_sec, p_rgb, p_ir, p_fused});

                            // 刪除超出 n 秒的「buffer」檔案（只刪 buffer，不影響 snapshots）
                            int keep = std::max(1, g_cfg.pre_snap_seconds);
                            while (!g_ring.empty() && (now_sec - g_ring.front().t) > keep) {
                                auto &old = g_ring.front();
                                if (!old.rgb.empty())   unlink(old.rgb.c_str());
                                if (!old.ir.empty())    unlink(old.ir.c_str());
                                if (!old.fused.empty()) unlink(old.fused.c_str());
                                g_ring.pop_front();
                            }
                        }
                    }
                }
                // 1011 每秒存一張到 buffer + 自動刪除過舊 END

                int key = cv::waitKey(1);

                {
                    ScopeTimer T_io(&prof, "GPIO_I2C_Read");
                    // === 0824 讀 IR 透明度切換鍵（GPIO54）與 Colormap 切換鍵（GPIO56） ===
                    static int last_alpha_btn = 1, last_cmap_btn = 1, last_snap_btn = 1; // 0924更新
                    static auto last_alpha_time = std::chrono::steady_clock::now();
                    static auto last_cmap_time  = std::chrono::steady_clock::now();
                    static auto last_snap_t = std::chrono::steady_clock::now(); // 0924新增

                    int alpha_btn = gpio_read_value(GPIO_BTN_ALPHA); // 低=按下
                    int cmap_btn  = gpio_read_value(GPIO_BTN_CMAP);
                    int snap_btn = gpio_read_value(GPIO_BTN_SNAP);// 0924新增

                    auto nowT = std::chrono::steady_clock::now();
                    auto dt_alpha = std::chrono::duration_cast<std::chrono::milliseconds>(nowT - last_alpha_time).count();
                    auto dt_cmap  = std::chrono::duration_cast<std::chrono::milliseconds>(nowT - last_cmap_time).count();
                    auto dt_snap = std::chrono::duration_cast<std::chrono::milliseconds>( // 0924新增這兩行是一起的
                        std::chrono::steady_clock::now() - last_snap_t).count();// 0924新增這兩行是一起的

                    // 0924 SNAP：按一下存一張
                    if (last_snap_btn==1 && snap_btn==0 && dt_snap>150) {
                        pend_snap.fetch_add(1);
                        last_snap_t = std::chrono::steady_clock::now();
                    }
                    last_snap_btn = snap_btn;
                    // 0924 SNAP：按一下存一張 END

                    // 透明度鍵：偵測由高→低的緣觸發，150ms 去彈跳
                    // 1011更新：GPIO53：設定模式 開/關（按一下切換），150ms 去彈跳
                    if (last_alpha_btn==1 && alpha_btn==0 && dt_alpha>150) {
                        setting_mode = !setting_mode;
                        setting_sel = 0;             // 進入時預設停在「IR 透明度檔位」
                        mark_activity();
                        std::cout << (setting_mode ? "[MODE] Enter SETUP" : "[MODE] Exit SETUP") << std::endl;
                        last_alpha_time = nowT;
                    }
                    last_alpha_btn = alpha_btn;
                    // 1011更新：GPIO53：設定模式 開/關（按一下切換），150ms 去彈跳 END

                    // Colormap 鍵：偵測由高→低，150ms 去彈跳 
                    // 1011 GPIO56 暫時取消功能（任何模式都不動作，但腳位保留）
                    // if (last_cmap_btn==1 && cmap_btn==0 && dt_cmap>150) {
                    //     cmap_idx = (cmap_idx + 1) % 6;               // 0..5
                    //     std::cout << "[BTN] Colormap -> " << kColorMapNames[cmap_idx] << "\n";
                    //     last_cmap_time = nowT;
                    // }
                    last_cmap_btn = cmap_btn;

                    //internet streaming
                    // 1011 GPIO52 「設定模式」下，搖桿左右切換項目、上下進下一檔；在「一般模式」下維持 dx/dy 調整
                    if (i2c_fd >= 0) {
                        // 讀 ADC 原始值
                        int16_t rx = ads1115_read_channel(i2c_fd, 0); // A0: X
                        int16_t ry = ads1115_read_channel(i2c_fd, 1); // A1: Y

                        // 將原始值映射為每幀的像素步進
                        const int DEADZONE = 3000;   // 約 10%
                        const int MAX_STEP = 6;
                        int stepX = axis_to_step(rx, x_center, DEADZONE, MAX_STEP);
                        int stepY = axis_to_step(ry, y_center, DEADZONE, MAX_STEP);

                        if (setting_mode) {
                            // 設定模式：左右切換「設定項目」，上下 →「下一檔」
                            static auto last_lr_t = std::chrono::steady_clock::now();
                            static auto last_ud_t = std::chrono::steady_clock::now();
                            auto now = std::chrono::steady_clock::now();
                            auto ms_lr = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_lr_t).count();
                            auto ms_ud = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ud_t).count();

                            if (stepX != 0 && ms_lr > 250) {
                                setting_sel ^= 1;              // 0 <-> 1 切換
                                last_lr_t = now;
                                mark_activity();
                                std::cout << "[SETUP] select " << (setting_sel==0 ? "ALPHA" : "CMAP") << "\n";
                            }
                            if (stepY != 0 && ms_ud > 250) {
                                if (setting_sel == 0) {
                                    alpha_idx = (alpha_idx + 1) % 5;
                                    alpha_ir  = kAlphaLevels[alpha_idx];
                                    std::cout << "[SETUP] ALPHA -> " << (alpha_idx*25) << "%\n";
                                } else {
                                    cmap_idx = (cmap_idx + 1) % 6;
                                    std::cout << "[SETUP] CMAP -> " << kColorMapNames[cmap_idx] << "\n";
                                }
                                last_ud_t = now;
                                mark_activity();
                            }
                        } else {
                            // 一般模式：維持原本 dx/dy 調整（上推=往上）
                            dx += stepX;
                            dy -= stepY;
                        }
                    }
                    // 1011 GPIO52「設定模式」下，搖桿左右切換項目、上下進下一檔；在「一般模式」下維持 dx/dy 調整 END

                    // GPIO52：中心鍵；在設定模式→退出；不在設定模式→reset dx/dy（150ms 去彈跳）
                    static int  last_rst_btn = 1;
                    static auto last_rst_t   = std::chrono::steady_clock::now();
                    int rst_btn = gpio_read_value(GPIO_BTN_RESET);
                    auto nowT2  = std::chrono::steady_clock::now();
                    auto dt_rst = std::chrono::duration_cast<std::chrono::milliseconds>(nowT2 - last_rst_t).count();
                    if (last_rst_btn==1 && rst_btn==0 && dt_rst>150) {
                        if (setting_mode) {
                            setting_mode = false;                 // 方式二：中心鍵退出設定模式
                            std::cout << "[MODE] Exit SETUP via center\n";
                        } else {
                            dx = 0; dy = 0;                       // 一般模式：reset dx/dy
                            std::cout << "[JOYSTICK] reset dx/dy to 0\n";
                        }
                        last_rst_t = nowT2;
                        mark_activity();
                    }
                    last_rst_btn = rst_btn;
                    // GPIO52：中心鍵；在設定模式→退出；不在設定模式→reset dx/dy（150ms 去彈跳）END

                    // 1011 10 秒無互動（按鈕或設定模式內的搖桿動作）自動退出設定模式
                    if (setting_mode) {
                        auto ms_idle = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - setting_last_activity).count();
                        if (ms_idle >= 10000) {
                            setting_mode = false;
                            std::cout << "[MODE] Exit SETUP (idle 10s)\n";
                        }
                    }
                    // 1011 10 秒無互動（按鈕或設定模式內的搖桿動作）自動退出設定模式 END
                }
                // 鍵盤當備援
                if (key == 82 || key == 'w') dy -= move_step;  // ↑ UP 或 w
                if (key == 84 || key == 's') dy += move_step;  // ↓ DOWN 或 s
                if (key == 81 || key == 'a') dx -= move_step;  // ← LEFT 或 a
                if (key == 83 || key == 'd') dx += move_step;  // → RIGHT 或 d
                //RESET
                if (key == 'r' || key == 'R') {
                    dx = 0;
                    dy = 0;
                    std::cout << "dx/dy reset to 0" << std::endl;
                }
                // 調整 IR 權重
                if (key == '+' || key == '=') alpha_ir = std::min(1.0, alpha_ir + alpha_step); // 增加 IR 權重
                if (key == '-' || key == '_') alpha_ir = std::max(0.0, alpha_ir - alpha_step); // 減少 IR 權重

                // 印出權重
                if (key == '+' || key == '=' || key == '-' || key == '_') {
                    std::cout << "IR weight (alpha_ir): " << alpha_ir << ", Camera weight (alpha_cam): " << 1.0 - alpha_ir << std::endl;
                }
            }

            // 1011 把前 n 秒 buffer 另存為一個快照資料夾
            {
                ScopeTimer T_save(&prof, "Save_Images");
                int times = pend_snap.exchange(0);
                while (times-- > 0) {
                    time_t now = time(NULL);
                    std::string ts_now = ts_string(now);

                    // 以 RGB 的 snapshots 當作「唯一性」的基準根目錄
                    std::string root_rgb   = join_path(g_cfg.save_dir_rgb,   "snapshots");
                    std::string root_ir    = join_path(g_cfg.save_dir_ir,    "snapshots");
                    std::string root_fused = join_path(g_cfg.save_dir_fused, "snapshots");
                    mkdir_p(root_rgb.c_str(),   0755);
                    mkdir_p(root_ir.c_str(),    0755);
                    mkdir_p(root_fused.c_str(), 0755);

                    // 取得不重名的批次名稱（例如 snap_20251011_142233、snap_20251011_142233_2、…）
                    std::string batch = unique_batch_tag(root_rgb, ts_now);

                    // 三類路徑都用同一個批次名稱
                    std::string dst_rgb   = join_path(root_rgb,   batch);
                    std::string dst_ir    = join_path(root_ir,    batch);
                    std::string dst_fused = join_path(root_fused, batch);
                    mkdir_p(dst_rgb.c_str(),   0755);
                    mkdir_p(dst_ir.c_str(),    0755);
                    mkdir_p(dst_fused.c_str(), 0755);

                    int keep = std::max(1, g_cfg.pre_snap_seconds);
                    // 從尾端往前收集 <= keep 秒的 buffer 檔（確保時間連續）
                    for (auto it = g_ring.rbegin(); it != g_ring.rend(); ++it) {
                        if ((now - it->t) > keep) break;
                        // 逐一複製（用 imread/imwrite 以避免依賴 <filesystem>）
                        if (!it->rgb.empty() && file_exists(it->rgb)) {
                            cv::Mat im = cv::imread(it->rgb);
                            if (!im.empty())
                                cv::imwrite(join_path(dst_rgb, "rgb_" + ts_string(it->t) + ".jpg"), im);
                        }
                        if (!it->ir.empty() && file_exists(it->ir)) {
                            cv::Mat im = cv::imread(it->ir);
                            if (!im.empty())
                                cv::imwrite(join_path(dst_ir, "ir_" + ts_string(it->t) + ".jpg"), im);
                        }
                        if (!it->fused.empty() && file_exists(it->fused)) {
                            cv::Mat im = cv::imread(it->fused);
                            if (!im.empty())
                                cv::imwrite(join_path(dst_fused, "fused_" + ts_string(it->t) + ".jpg"), im);
                        }
                    }
                    std::cout << "[SNAP] saved last " << keep << "s into batch '" << batch << "'\n";
                }
            }
            // 1011 把前 n 秒 buffer 另存為一個快照資料夾 END

            {
                ScopeTimer T_wait(&prof, "WaitKey");
                /* Press 'ESC' to exit */
                if (27 == cv::waitKey(10)) {
                    sig_kill(0);
                    break;
                }
            }
        } //total 計算到這
        // 0924 每迴圈最後加上「到點就回存config」
        using namespace std::chrono;
        int minutes_interval = (g_cfg.save_cfg_minutes>0) ? g_cfg.save_cfg_minutes : 10;
        if (duration_cast<minutes>(steady_clock::now() - last_cfg_save).count() >= minutes_interval) {
            config_save(g_cfg, dx, dy, cmap_idx, alpha_idx);
            last_cfg_save = steady_clock::now();
        }
        // 0924 每迴圈最後加上「到點就回存config」END

         // ---- OTHER（把沒包到的開銷歸類在此）----
        {
            using namespace std::chrono;
            static auto last_frame_end = steady_clock::now();
            auto now = steady_clock::now();
            double loop_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(now - frame_begin).count();
            double known_ms =
                (prof.sums_ms.count("TOF_get")?0:0) + 0; // placeholder 避免編譯器警告

            // 我們用 TOTAL 測一整圈，OTHER = TOTAL - (各段相加)
            // 這裡在 ScopeTimer 解構時就已經加總，因此只要把 TOTAL 減其他 key 即可：
            double sum_known = 0.0;
            for (auto &kv : prof.sums_ms) {
                const std::string &k = kv.first;
                if (k=="TOTAL") continue;
                sum_known += kv.second;
            }
            double other_ms = 0.0;
            if (prof.sums_ms.count("TOTAL")) {
                other_ms = prof.sums_ms["TOTAL"] - sum_known;
                if (other_ms < 0) other_ms = 0.0;
                prof.add("OTHER", other_ms);
            }
        } 
        // prof.print_now();
        frame_begin = steady_clock::now();
    }
    
    _blImageRunning = false;
    _blSendInfRunning = false;
    _blResultRunning = false;

    _blDispatchRunning = false;
    _blFifoqManagerRunning = false;

#ifdef H26XE_API_H
    if (g_ptSsmWriterHandle) {
        // ★ 提交最後一顆，再釋放
        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);
        SSM_Release(g_ptSsmWriterHandle);
        g_ptSsmWriterHandle = NULL;    // ★ 清空以防後續誤用
    }
#endif

#ifdef TOF_EN
    tof.stop();     // 停止tof
#endif
    prof.close_log();
    config_save(g_cfg, dx, dy, cmap_idx, alpha_idx); // 0924 結束前把最新狀態寫回設定檔，避免沒等到下一次週期
    printf("[%s] bye!\r\n", __func__);
    return NULL;
}
