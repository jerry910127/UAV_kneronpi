#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <opencv2/opencv.hpp>

extern "C" {
#include "kdp2_inf_app_yolo.h"
}

#include "example_shared_struct.h"
#include "kp_struct.h"

extern NNM_SHARED_INPUT_T _input_data;
extern pthread_mutex_t _mutex_image;

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

static void send_metadata_to_ground_station(kp_app_yolo_result_t *yolo_result)
{
    static int sockfd = -1;
    static struct sockaddr_in servaddr;
    static unsigned int seq_num = 0;
    static char current_ip[64] = "192.168.168.17";

    if (sockfd < 0) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            printf("[UDP] Socket creation failed\n");
            return;
        }
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(9001); // Ground Station Port
        servaddr.sin_addr.s_addr = inet_addr(current_ip); // Ground Station IP
    }

    // Dynamic IP update from /tmp/ground_station_ip.txt if written by uav_daemon.py
    FILE *fp = fopen("/tmp/ground_station_ip.txt", "r");
    if (fp) {
        char buf[64] = {0};
        if (fgets(buf, sizeof(buf), fp)) {
            buf[strcspn(buf, "\r\n")] = 0;
            if (strlen(buf) > 6 && strcmp(buf, current_ip) != 0) {
                strncpy(current_ip, buf, sizeof(current_ip) - 1);
                servaddr.sin_addr.s_addr = inet_addr(current_ip);
                printf("[UDP] Dynamic Ground Station IP updated to: %s\n", current_ip);
            }
        }
        fclose(fp);
    }

    seq_num++;
    unsigned char send_buf[1024];
    unsigned int box_count = yolo_result->box_count;
    if (box_count > 40) box_count = 40; // Prevent overflow

    memcpy(send_buf, &seq_num, 4);
    memcpy(send_buf + 4, &box_count, 4);
    int offset = 8;

    for (uint32_t i = 0; i < box_count; i++) {
        unsigned int class_id = yolo_result->boxes[i].class_num;
        float score = yolo_result->boxes[i].score;
        unsigned int x1 = yolo_result->boxes[i].x1;
        unsigned int y1 = yolo_result->boxes[i].y1;
        unsigned int x2 = yolo_result->boxes[i].x2;
        unsigned int y2 = yolo_result->boxes[i].y2;

        memcpy(send_buf + offset, &class_id, 4);
        memcpy(send_buf + offset + 4, &score, 4);
        memcpy(send_buf + offset + 8, &x1, 4);
        memcpy(send_buf + offset + 12, &y1, 4);
        memcpy(send_buf + offset + 16, &x2, 4);
        memcpy(send_buf + offset + 20, &y2, 4);
        offset += 24;
    }

    sendto(sockfd, send_buf, offset, 0, (const struct sockaddr *)&servaddr, sizeof(servaddr));
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

        // Send metadata to Ground Station (Scheme B)
        send_metadata_to_ground_station(yolo_result);

        for (uint32_t i = 0; i < yolo_result->box_count; i++) {
            cv::rectangle(*cv_img_display, cv::Point(yolo_result->boxes[i].x1, yolo_result->boxes[i].y1),
                            cv::Point(yolo_result->boxes[i].x2, yolo_result->boxes[i].y2), cv::Scalar(50, 255, 50), 2);
        }
        pthread_mutex_unlock(&_mutex_result);

        cv::putText(*cv_img_display, strImgFPS, cv::Point(5, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(50, 50, 255), 1);
        cv::putText(*cv_img_display, strInfFPS, cv::Point(5, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(50, 50, 255), 1);
        cv::putText(*cv_img_display, "Press 'ESC' to exit", cv::Point(10, cv_img_display->rows - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 2);
    }
    else
    {
        ret = KP_FW_ERROR_UNKNOWN_APP;
    }

    return ret;
}

void *example_display_liveview_thread(void *)
{
    struct timeval time_begin;
    struct timeval time_end;
    float time_spent = 0.0;
    char strImgFPS[50] = "Image FPS: ";
    char strInfFPS[50] = "Inference FPS: ";
    cv::Mat cv_image_source;
    cv::Mat cv_image_display;

    cv::namedWindow("Inference Display", cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
    gettimeofday(&time_begin, NULL);

    while (true == _blDisplayRunning) {

        if (_result_count >= 60)
        {
            gettimeofday(&time_end, NULL);
            time_spent = (float)(time_end.tv_sec - time_begin.tv_sec) + (float)(time_end.tv_usec - time_begin.tv_usec) * .000001;
            sprintf(strImgFPS, "Image FPS: %.2lf", _image_count / time_spent);
            sprintf(strInfFPS, "Inference FPS: %.2lf", _result_count / time_spent);
            printf("[AI Liveview] %s | %s (Time spent: %.2fs)\n", strImgFPS, strInfFPS, time_spent);
            fflush(stdout);
            _image_count = 0;
            _result_count = 0;

            gettimeofday(&time_begin, NULL);
        }

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

        /* Display image */
        if (false == cv_image_display.empty()) {
            if (true == _inf_result.result_ready_display) {
                draw_display_image(&cv_image_display, strImgFPS, strInfFPS);
            }

            cv::imshow("Inference Display", cv_image_display);
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
