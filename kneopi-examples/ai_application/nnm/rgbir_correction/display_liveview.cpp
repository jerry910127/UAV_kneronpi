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

// #include "/root/Desktop/TOF/include/tof_lib.hpp"  // include tof library

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

static int mkdir_p(const char* path, mode_t mode = 0755);

// base directory for saving images
static std::string output_base_dir = "/work/image";
//Thermal camera fisheye parameters
#if 1
#include "/work/config/camera_params.txt"

#else
#define LOAD_MTX
std::string camera_params_path = "/work/config/camera_params.txt"; 
#endif

static std::string output_dir(const char* subfolder) {
    if (!output_base_dir.empty() && output_base_dir.back() == '/')
        return output_base_dir + subfolder;
    return output_base_dir + "/" + subfolder;
}


int draw_display_image(cv::Mat *cv_img_display, const char *strImgFPS, const char *strInfFPS)
{
    int ret = KP_SUCCESS;
    kp_inference_header_stamp_t *header_stamp = (kp_inference_header_stamp_t *)_inf_result.result_buffer;

    if (KDP2_INF_ID_APP_YOLO == header_stamp->job_id)
    {
        pthread_mutex_lock(&_mutex_result);
        kdp2_ipc_app_yolo_result_t *app_yolo_result = (kdp2_ipc_app_yolo_result_t *)header_stamp;
        kp_app_yolo_result_t *yolo_result = (kp_app_yolo_result_t *)&app_yolo_result->yolo_data;

        // for (uint32_t i = 0; i < yolo_result->box_count; i++) {
        //     cv::rectangle(*cv_img_display, cv::Point(yolo_result->boxes[i].x1, yolo_result->boxes[i].y1),
        //                     cv::Point(yolo_result->boxes[i].x2, yolo_result->boxes[i].y2), cv::Scalar(50, 255, 50), 2);
        // }
        pthread_mutex_unlock(&_mutex_result);

        cv::putText(*cv_img_display, strImgFPS, cv::Point(5, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(50, 50, 255), 1);
        cv::putText(*cv_img_display, strInfFPS, cv::Point(5, 60), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(50, 50, 255), 1);
        cv::putText(*cv_img_display, "Press 'ESC' to exit", cv::Point(10, cv_img_display->rows - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 2);
    }
    else
    {
        ret = KP_FW_ERROR_UNKNOWN_APP;
    }

    return ret;
}

#ifdef LOAD_MTX
static bool loadCameraParams(const std::string& path, cv::Mat& K, cv::Mat& D) {
    std::ifstream fin(path);
    if (!fin) {
        std::cerr << "[CAM] cannot open " << path << "\n";
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(fin)),
                        std::istreambuf_iterator<char>());

    // 小工具：從 "tag" 開始，找後面 "<< ..." 的數字，塞進 rows x cols 的 Mat
    auto parseBlock = [&](const std::string& tag, int rows, int cols, cv::Mat& out) -> bool {
        size_t pos = content.find(tag);
        if (pos == std::string::npos) return false;

        // 找到 "<<" 之後開始抓數字
        pos = content.find("<<", pos);
        if (pos == std::string::npos) return false;
        pos += 2; // 跳過 "<<"

        std::regex num_re("[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?");
        std::sregex_iterator it(content.begin() + pos, content.end(), num_re);
        std::sregex_iterator end;

        std::vector<double> vals;
        for (; it != end && (int)vals.size() < rows * cols; ++it) {
            vals.push_back(std::stod((*it).str()));
        }
        if ((int)vals.size() != rows * cols) {
            std::cerr << "[CAM] parseBlock(" << tag << ") expect "
                      << (rows * cols) << " numbers, got " << vals.size() << "\n";
            return false;
        }

        out = cv::Mat(rows, cols, CV_64F);
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                out.at<double>(r, c) = vals[r * cols + c];
        return true;
    };

    bool okK = parseBlock("K_webcam", 3, 3, K);
    bool okD = parseBlock("D_webcam", 1, 5, D);

    if (!okK || !okD) {
        std::cerr << "[CAM] loadCameraParams(): parse K_webcam/D_webcam failed.\n";
        return false;
    }

    // 簡單檢查一下
    if (!K.isContinuous() || !D.isContinuous()) {
        std::cerr << "[CAM] matrices not continuous (unexpected).\n";
        K.release();
        D.release();
        return false;
    }

    // std::cerr << "[CAM] loaded K_webcam and D_webcam from " << path << "\n";
    return true;
}
#endif


cv::Mat undistortImage(
    const cv::Mat& img,
    double alpha = 0.0,   // 0: 無黑邊；1: 全視野（可能有黑邊）
    bool crop = true      // 是否依 ROI 裁切黑邊
) {
#ifdef LOAD_MTX
    cv::Mat mtx ,dist;
    if (!loadCameraParams(camera_params_path, mtx, dist)) {
    printf("Failed to load camera params\n");
    }
#endif

    // 計算最佳新相機矩陣
    cv::Mat newCameraMatrix;
    cv::Rect roi;
    cv::Size sz = img.size();
    newCameraMatrix = cv::getOptimalNewCameraMatrix(mtx, dist, sz, alpha, sz, &roi);

    // 去畸變
    cv::Mat undistorted;
    cv::undistort(img, undistorted, mtx, dist, newCameraMatrix);

    // ROI 裁切
    if (crop && roi.width > 0 && roi.height > 0) {
        return undistorted(roi).clone();
    }
    return undistorted;
}
static int mkdir_p(const char* path, mode_t mode) {
    if (!path || !*path) return -1;

    char tmp[PATH_MAX];
    size_t len = strlen(path);
    if (len >= sizeof(tmp)) return -1;

    strcpy(tmp, path);
    if (len > 1 && tmp[len-1] == '/') {
        tmp[len-1] = '\0';
    }

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
static void ensure_output_dirs() {

    const char* subs[] = {"rgb", "ir"};
    for (const char* s : subs) {
        std::string p = output_dir(s);
        if (mkdir_p(p.c_str()) != 0) {
            perror("mkdir_p failed");
        }
    }
}
void *example_display_liveview_thread(void *)
{
    ensure_output_dirs();
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

    cv::namedWindow("Inference Display", cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
    // cv::namedWindow("Inference Display", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::namedWindow("Webcam Display", cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);

    gettimeofday(&time_begin, NULL);



    //tof.start();     // 啟動tof

    while (true == _blDisplayRunning) {
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


        /* Display image */
        if (false == cv_image_display.empty()) {
            if (true == _inf_result.result_ready_display) {
                //sprintf(strDistance, "Distance: %.2lf", distancedate.range);
                //sprintf(strQuality, "Quality: %d", distancedate.quality);
                draw_display_image(&cv_image_display, strDistance, strQuality);
            }
            cv::imshow("Inference Display", cv_image_display);
        }

        if (false == cv_image_display_WEBCAM.empty()) {
            if (true == _input_data_webcam.input_ready_inf){
                cv::rotate(cv_image_display_WEBCAM, cv_image_display_WEBCAM, cv::ROTATE_90_CLOCKWISE);
                cv_image_display_WEBCAM = undistortImage(cv_image_display_WEBCAM, /*alpha=*/0.0, /*crop=*/true);
                cv::imshow("Webcam Display", cv_image_display_WEBCAM);
            }
            
        }

        /*儲存畫面*/
        time_t now = time(NULL);
        if (now != last_save_time)
        {
            struct tm* tm_info = localtime(&now);
            char time_str[32];
            strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", tm_info);

            char filename_sensor[128];
            char filename_webcam[128];

            snprintf(filename_sensor, sizeof(filename_sensor), "%s/sensor_%s.jpg",
            output_dir("rgb").c_str(), time_str);
            snprintf(filename_webcam, sizeof(filename_webcam), "%s/webcam_%s.jpg",
            output_dir("ir").c_str(), time_str);

            if (!cv_image_display.empty())
                cv::imwrite(filename_sensor, cv_image_display);
            if (!cv_image_display_WEBCAM.empty())
                cv::imwrite(filename_webcam, cv_image_display_WEBCAM);
            last_save_time = now;
        }

        /* Press 'ESC' to exit */
        if (27 == cv::waitKey(10)) {
            sig_kill(0);
            break;
        }
    }

    _blImageRunning = false;
    _blSendInfRunning = false;
    _blResultRunning = false;

    _blDispatchRunning = false;
    _blFifoqManagerRunning = false;

    return NULL;
}
