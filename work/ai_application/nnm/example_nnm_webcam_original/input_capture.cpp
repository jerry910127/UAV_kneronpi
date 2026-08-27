#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

#include <opencv2/opencv.hpp>

#include "example_shared_struct.h"
#include "kp_struct.h"

#if 0	//JDBG
#include "h26xenc_api.h"

#include <sync_shared_memory.h>
#include <ssm_info.h>

#define CAM_WIDTH	640
#define CAM_HEIGHT	480

extern char *g_szSsmReaderPin;

extern SSM_WRITER_INIT_OPTION_T ssm_opt;
extern SSM_HANDLE_T* g_ptSsmWriterHandle;

void ssm_clear_header(unsigned char* virt_addr, unsigned int buf_size, void* pUserData);
#endif

extern bool _blDispatchRunning;
extern bool _blFifoqManagerRunning;

extern bool _blSendInfRunning;
extern bool _blResultRunning;
extern bool _blDisplayRunning;

bool _blImageRunning = true;

NNM_SHARED_INPUT_T _input_data = {0};
pthread_mutex_t _mutex_image = PTHREAD_MUTEX_INITIALIZER;

void *example_webcam_input_thread(void *arg)
{
    EXAMPLE_WEBCAM_INIT_OPT_T* pInitOpt=(EXAMPLE_WEBCAM_INIT_OPT_T*)arg;

    cv::VideoCapture cv_camera_cap;
    cv::Mat cv_read_camera, cv_img_to_be_sent;

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
    ssm_opt.buf_size = (CAM_WIDTH * CAM_HEIGHT * 2) + VMF_MAX_SSM_HEADER_SIZE;
    ssm_opt.alignment = VMF_ALIGN_TYPE_DEFAULT;
    ssm_opt.pUserData = &vsrc_ssm_writer_info;
    ssm_opt.fp_setup_buffer = ssm_clear_header;
    g_ptSsmWriterHandle = SSM_Writer_Init(&ssm_opt);
    if (!g_ptSsmWriterHandle)
    {
        printf("init g_ptSsmWriterHandle failed\n");
    }
#endif

    if (false == cv_camera_cap.open(pInitOpt->pszCameraPath)) {
        printf("[%s] open camera failed %s\n", __FUNCTION__, pInitOpt->pszCameraPath);
        goto EXIT_FREAD_IMAGE_THREAD;
    }

    /* Setting frame size may failed in OpenCV */
    cv_camera_cap.set(cv::CAP_PROP_FRAME_WIDTH, pInitOpt->dwImageWidth);
    cv_camera_cap.set(cv::CAP_PROP_FRAME_HEIGHT, pInitOpt->dwImageHeight);
    cv_camera_cap.set(cv::CAP_PROP_FPS, pInitOpt->dwFps);

    pInitOpt->dwImageWidth = (unsigned int)cv_camera_cap.get(cv::CAP_PROP_FRAME_WIDTH);
    pInitOpt->dwImageHeight = (unsigned int)cv_camera_cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    while (true == _blImageRunning)
    {
#ifdef H26XE_API_H
        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);
#endif
        cv_camera_cap.read(cv_read_camera);

        pthread_mutex_lock(&_mutex_image);
        cv::cvtColor(cv_read_camera, cv_img_to_be_sent, cv::COLOR_BGR2RGBA);

        _input_data.input_buf_address = (uintptr_t)cv_img_to_be_sent.data;
        _input_data.input_image_width = cv_img_to_be_sent.cols;
        _input_data.input_image_height = cv_img_to_be_sent.rows;
        _input_data.input_image_format = KP_IMAGE_FORMAT_RGBA8888;

        _input_data.input_buf_size = _input_data.input_image_width * _input_data.input_image_height * 4;
        _input_data.input_ready_inf = true;
#ifdef H26XE_API_H
        bgrToYUV420(cv_read_camera, yuv_data);

        memcpy(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, yuv_data.data(), yuv_data.size());
        MemBroker_CacheFlush(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, yuv_data.size());
#endif
        pthread_mutex_unlock(&_mutex_image);
    }

EXIT_FREAD_IMAGE_THREAD:

#ifdef H26XE_API_H
    if (g_ptSsmWriterHandle) {
        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);
        SSM_Release(g_ptSsmWriterHandle);
        g_ptSsmWriterHandle = NULL;
    }
#endif

    _blSendInfRunning = false;
    _blResultRunning = false;
    _blDisplayRunning = false;

    _blDispatchRunning = false;
    _blFifoqManagerRunning = false;

    printf("[%s] bye!\r\n", __func__);
    return NULL;
}
