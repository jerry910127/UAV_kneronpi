/*
 *******************************************************************************
 *  Copyright (c) 2010-2022 VATICS(KNERON) Inc. All rights reserved.
 *
 *  +-----------------------------------------------------------------+
 *  | THIS SOFTWARE IS FURNISHED UNDER A LICENSE AND MAY ONLY BE USED |
 *  | AND COPIED IN ACCORDANCE WITH THE TERMS AND CONDITIONS OF SUCH  |
 *  | A LICENSE AND WITH THE INCLUSION OF THE THIS COPY RIGHT NOTICE. |
 *  | THIS SOFTWARE OR ANY OTHER COPIES OF THIS SOFTWARE MAY NOT BE   |
 *  | PROVIDED OR OTHERWISE MADE AVAILABLE TO ANY OTHER PERSON. THE   |
 *  | OWNERSHIP AND TITLE OF THIS SOFTWARE IS NOT TRANSFERRED.        |
 *  |                                                                 |
 *  | THE INFORMATION IN THIS SOFTWARE IS SUBJECT TO CHANGE WITHOUT   |
 *  | ANY PRIOR NOTICE AND SHOULD NOT BE CONSTRUED AS A COMMITMENT BY |
 *  | VATICS(KNERON) INC.                                             |
 *  +-----------------------------------------------------------------+
 *
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <getopt.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/resource.h>
#include <linux/videodev2.h>

#include <iniparser/iniparser.h> //webcam

#include <opencv2/opencv.hpp>

//C-Style 外部函式庫（Kneron SDK） 
extern "C" { 
#include <sync_shared_memory.h>
#include <ssm_info.h>
#include <vmf_nnm_inference_app.h>
#include <vmf_nnm_fifoq_manager.h>
#include "fec_api.h"
}

#include "application_init.h"
#include "example_shared_struct.h"

//fifo queue buffer setting 
#define IMAGE_BUFFER_COUNT      3 //

#define RESULT_BUFFER_COUNT     3
#define RESULT_BUFFER_SIZE_SENSOR     512 * 1024 //512 KB， 每一筆「推論結果」預留的記憶體上限 

//webcam
#define IMAGE_BUFFER_SIZE       (4 * 1920 * 1080 + 1024)
// #define RESULT_BUFFER_SIZE_WEBCAM      (1024 * 1024)

#define EXAMPLE_SENSOR_CONFIG_PATH "./ini/example_sensor.ini"
#define EXAMPLE_WEBCAM_CONFIG_PATH "./ini/example_webcam.ini"

#define IMU_CONFIG_PATH             "/sys/bus/iio/devices/iio:device1/buffer/enable"

extern void *example_sensor_image_thread(void *arg); // 取流
extern void *example_send_inf_thread(void *arg);  // 送推論
extern void *example_recv_result_thread(void *arg); // 收結果
extern void *example_display_liveview_thread(void *arg); // 顯示
//WEBCAM
extern void *example_webcam_input_thread(void *arg);
extern void *example_display_liveview_thread_webcam(void *arg);

bool _blDispatchRunning = true;       // 推論影像分派
bool _blFifoqManagerRunning = true;   // FIFO 管理
extern bool _blImageRunning;          // 取流
extern bool _blSendInfRunning;        // 送推論
extern bool _blResultRunning;         // 收推論
extern bool _blDisplayRunning;        // 顯示

extern ssm_handle_t  	*gptSsmHandle;

//pExampleSensorInit : 指向呼叫者準備好的結構體，用來存放解析後的設定值。
int loadConfig_sensor(const char* HostSensorConfigPath, EXAMPLE_SENSOR_INIT_OPT_T* pExampleSensorInit)
{
    dictionary* ini = NULL;
    const char* tmp = NULL;
    struct stat info;
    char search_str[30] = {0};

    //! check file
    if (0 != stat(HostSensorConfigPath, &info))
        return -1;

    if (!(info.st_mode & S_IFREG))
        return -1;

    //用 iniparser 載入 INI
    ini = iniparser_load(HostSensorConfigPath);

    tmp = iniparser_getstring(ini, "sensor:sensor_cfg", NULL);
    if (tmp)
        pExampleSensorInit->tSensorConf.pszSensorConfigPath = strdup(tmp); //strdup()：把字串複製到 heap，後面才好由 free_sensor_init() 統一釋放。

    tmp = iniparser_getstring(ini, "sensor:autoscene_config", NULL);
    if (tmp /*&& (app_mode != VMF_VSRC_APP_MODE_DMA422TO420)*/)	// Disable autoscene when app_mode == VMF_VSRC_APP_MODE_DMA422TO420
        pExampleSensorInit->tSensorConf.pszAutoSceneConfigPath = strdup(tmp);

    tmp = iniparser_getstring(ini, "sensor:fusion_cfg", NULL);
    if (tmp)
        pExampleSensorInit->tSensorConf.pszFusionConfigPath = strdup(tmp);

    tmp = iniparser_getstring(ini, "sensor:fec_calibrate_path", NULL);
    if (tmp)
        pExampleSensorInit->tSensorConf.pszFecCalibratePath = strdup(tmp);

    tmp = iniparser_getstring(ini, "sensor:fec_conf_path", NULL);
    if (tmp)
        pExampleSensorInit->tSensorConf.pszFecConfPath = strdup(tmp);

    pExampleSensorInit->tSensorConf.dwFecMode = iniparser_getint(ini, "sensor:fec_mode", 0);
    pExampleSensorInit->tSensorConf.dwFecAppType = iniparser_getint(ini, "sensor:initial_fec_app_type", 0);
    pExampleSensorInit->dwEisEnable = iniparser_getint(ini, "sensor:eis_enable", 0);

    printf("[NNM] sensor_cfg: %s autoscene_config: %s \n", pExampleSensorInit->tSensorConf.pszSensorConfigPath, pExampleSensorInit->tSensorConf.pszAutoSceneConfigPath);
    if (pExampleSensorInit->tSensorConf.pszFusionConfigPath) {
        printf("[NNM] fusion_cfg: %s\n", pExampleSensorInit->tSensorConf.pszFusionConfigPath);
    }
    printf("[NNM] fec_calibrate: %s fec_conf: %s \n", pExampleSensorInit->tSensorConf.pszFecCalibratePath, pExampleSensorInit->tSensorConf.pszFecConfPath);

    pExampleSensorInit->pszModelPath = strdup(iniparser_getstring(ini, "nnm:ModelPath", "model.nef"));
    pExampleSensorInit->dwImageWidth = iniparser_getint(ini, "nnm:ImageWidth", 640);
	pExampleSensorInit->dwImageHeight = iniparser_getint(ini, "nnm:ImageHeight", 640);
    pExampleSensorInit->dwInferenceStream = iniparser_getint(ini, "nnm:InferenceStream", 2);
    pExampleSensorInit->dwJobId = iniparser_getint(ini, "nnm:JobId", 11);
    pExampleSensorInit->dwGetImageBufMode = iniparser_getint(ini, "nnm:GetImageBufMode", 0);

    if (pExampleSensorInit->dwEisEnable == 1) {
        FILE *fDeviceBufferEnable = NULL;
        if (pExampleSensorInit->tSensorConf.dwFecMode != FEC_MODE_1O && pExampleSensorInit->tSensorConf.dwFecMode != FEC_MODE_1R ) {
            printf("EIS mode does NOT support mode %u [Disable EIS]\n", pExampleSensorInit->tSensorConf.dwFecMode);
            pExampleSensorInit->dwEisEnable = 0;
        }
        fDeviceBufferEnable = fopen(IMU_CONFIG_PATH,"r");
        if (!fDeviceBufferEnable) {
            printf("[streamer] Cannot read %s in EIS mode. [Disable EIS]\n", IMU_CONFIG_PATH);
            pExampleSensorInit->dwEisEnable = 0;
        }
        if (fDeviceBufferEnable)
            fclose(fDeviceBufferEnable);
    }

    printf("[NNM] Model: %s ImageWidth: %d ImageHeight: %d\n", pExampleSensorInit->pszModelPath, pExampleSensorInit->dwImageWidth, pExampleSensorInit->dwImageHeight);
    printf("[NNM] Model: %s dwJobId: %d \n", pExampleSensorInit->pszModelPath, pExampleSensorInit->dwJobId);
    iniparser_freedict(ini);
    return 0;
}
int loadConfig_webcam(const char* HostVerifyConfigPath, EXAMPLE_WEBCAM_INIT_OPT_T* pExampleWebCamInit)
{
	dictionary* ini = NULL;
	struct stat info;

	//! check file
	if (0 != stat(HostVerifyConfigPath, &info))
		return -1;

	if (!(info.st_mode & S_IFREG))
		return -1;

	printf("iniparser_load %s \n", HostVerifyConfigPath);
	ini = iniparser_load(HostVerifyConfigPath);

	pExampleWebCamInit->pszModelPath = strdup(iniparser_getstring(ini, "nnm:ModelPath", "model.nef"));
	pExampleWebCamInit->dwImageWidth = iniparser_getint(ini, "nnm:ImageWidth", 640);
	pExampleWebCamInit->dwImageHeight = iniparser_getint(ini, "nnm:ImageHeight", 480);
	pExampleWebCamInit->dwJobId = iniparser_getint(ini, "nnm:JobId", 11);
	pExampleWebCamInit->dwFps = iniparser_getint(ini, "nnm:Fps", 30);
    if (pExampleWebCamInit->dwFps == 0) {
        pExampleWebCamInit->dwFps = 30;
        printf("[Fps = 0], set a default fps %f.\n", pExampleWebCamInit->dwFps);
    }
    pExampleWebCamInit->pszCameraPath = strdup(iniparser_getstring(ini, "nnm:CameraPath", "/dev/video0"));

    printf("[NNM] Model: %s pszCameraPath: %s \n", pExampleWebCamInit->pszModelPath, pExampleWebCamInit->pszCameraPath);
	printf("[NNM] Model: %s ImageWidth: %d ImageHeight: %d Fps: %u \n", pExampleWebCamInit->pszModelPath, pExampleWebCamInit->dwImageWidth, pExampleWebCamInit->dwImageHeight, pExampleWebCamInit->dwFps);
	printf("[NNM] Model: %s dwJobId: %d \n", pExampleWebCamInit->pszModelPath, pExampleWebCamInit->dwJobId);
    iniparser_freedict(ini);
	return 0;
}

void sig_kill(int signo)
{
    if (gptSsmHandle)//get image sensor yuv data on HICO/HOST mode
        SSM_Reader_Wakeup(gptSsmHandle);

    _blDispatchRunning = false;
    _blFifoqManagerRunning = false;

    _blImageRunning = false;
    _blSendInfRunning = false;
    _blResultRunning = false;
    _blDisplayRunning = false;

    VMF_NNM_Fifoq_Manager_Wakeup();
}

void print_usage(char* argv[])
{
    printf("Usage 1, setting by ini: %s, default auto load [%s] \r\n", argv[0], EXAMPLE_SENSOR_CONFIG_PATH);
}

char* parse_argument(int argc, char* argv[])
{
    int ch;
    char* pszConfigPath = NULL;

    while ((ch = getopt(argc, argv, "c:")) != -1) {
        switch (ch) {
            case 'c':
                if(pszConfigPath == NULL) {
                    pszConfigPath = strdup(optarg);
                    printf("pszHostSensorConfigPath = %s\n", pszConfigPath);
                }
                break;

            default:
                print_usage(argv);
        }
    }

    if (NULL == pszConfigPath ) {
        print_usage(argv);

        return NULL;
    }

    return pszConfigPath;
}

void free_sensor_init(EXAMPLE_SENSOR_INIT_OPT_T* pExampleSensorInit) { 
    if (pExampleSensorInit->tSensorConf.pszSensorConfigPath) {
        free(pExampleSensorInit->tSensorConf.pszSensorConfigPath);
        pExampleSensorInit->tSensorConf.pszSensorConfigPath = NULL;
    }

    if (pExampleSensorInit->tSensorConf.pszFusionConfigPath) {
        free(pExampleSensorInit->tSensorConf.pszFusionConfigPath);
        pExampleSensorInit->tSensorConf.pszFusionConfigPath = NULL;
    }

    if (pExampleSensorInit->tSensorConf.pszAutoSceneConfigPath) {
        free(pExampleSensorInit->tSensorConf.pszAutoSceneConfigPath);
        pExampleSensorInit->tSensorConf.pszAutoSceneConfigPath = NULL;
    }

    if (pExampleSensorInit->tSensorConf.pszFecCalibratePath) {
        free(pExampleSensorInit->tSensorConf.pszFecCalibratePath);
        pExampleSensorInit->tSensorConf.pszFecCalibratePath = NULL;
    }

    if (pExampleSensorInit->tSensorConf.pszFecConfPath) {
        free(pExampleSensorInit->tSensorConf.pszFecConfPath);
        pExampleSensorInit->tSensorConf.pszFecConfPath = NULL;
    }

    if (pExampleSensorInit->pszModelPath) {
        free(pExampleSensorInit->pszModelPath);
        pExampleSensorInit->pszModelPath = NULL;
    }
}

int main (int argc, char* argv[])
{
    uint32_t major, minor, patch, build;
    char* pszHostSensorConfigPath = NULL;

    VMF_NNM_Get_Version(&major, &minor, &patch, &build);

    printf("\n\n**********************************************************\n");
    printf("Kneron Firmware\n");
    printf("Ver. %d.%d.%d.%d\n", major, minor, patch, build);
    printf("Build Time: %s %s\n", __DATE__, __TIME__);
    printf("**********************************************************\n");
    printf("HOST Sensor mode \n");

    pthread_t task_inf_data_handle; //VMF_NNM_Inference_Image_Dispatcher_Thread;
    pthread_t task_buf_mgr_handle;  //VMF_NNM_Fifoq_Manager_Enqueue_Image_Thread;

    pthread_t task_sensor_image_handle;
    pthread_t task_send_inf_handle;
    pthread_t task_recv_result_handle;
    pthread_t task_display_handle;
    //WEBCAM
    pthread_t task_webcam_image_handle;
    pthread_t task_display_handle_webcam;

    int ret = KP_SUCCESS;
    EXAMPLE_SENSOR_INIT_OPT_T ExampleSensorInit; //for host sensor mode/thread
    uint32_t ImageBufferSize = 0;
    EXAMPLE_WEBCAM_INIT_OPT_T ExampleWebCamInit;

    memset(&ExampleSensorInit, 0, sizeof(EXAMPLE_SENSOR_INIT_OPT_T));
    memset(&ExampleWebCamInit, 0, sizeof(EXAMPLE_WEBCAM_INIT_OPT_T));

    if(0 != loadConfig_sensor(EXAMPLE_SENSOR_CONFIG_PATH, &ExampleSensorInit))                  // load ini file
        goto EXIT;
    if(0 != loadConfig_webcam(EXAMPLE_WEBCAM_CONFIG_PATH, &ExampleWebCamInit))                // load ini file
        goto EXIT;

    // if (argc > 1) {
    //     pszHostSensorConfigPath = parse_argument(argc, argv);
    //     if(pszHostSensorConfigPath == NULL)   //parse argument
    //         goto EXIT;

    //     if(0 != loadConfig_SENSOR(pszHostSensorConfigPath, &ExampleSensorInit))                  // load ini file
    //         goto EXIT;

    // } else {
    //     if(0 != loadConfig_SENSOR(EXAMPLE_SENSOR_CONFIG_PATH, &ExampleSensorInit))                  // load ini file
    //         goto EXIT;
    // }

    ImageBufferSize = ExampleSensorInit.dwImageWidth * ExampleSensorInit.dwImageHeight * 2 + 1024;

    //! register signal
    signal(SIGTERM, sig_kill);
    signal(SIGKILL, sig_kill);
    signal(SIGINT, sig_kill);

    //SIGSEGV
    struct sigaction sa;
    memset(&sa, 0, sizeof(struct sigaction));
    sigemptyset(&sa.sa_mask);
    sa.sa_handler  = sig_kill;
    sa.sa_flags = SA_SIGINFO|SA_RESETHAND;  // Reset signal handler to system default after signal triggered
    sigaction(SIGSEGV, &sa, NULL);

    printf("[%s] app_initialize \n", __func__);
    app_initialize();

    VMF_NNM_Load_Model_From_File(ExampleSensorInit.pszModelPath);

    VMF_NNM_Fifoq_Manager_Allocate_Buffer(IMAGE_BUFFER_COUNT, ImageBufferSize, RESULT_BUFFER_COUNT, RESULT_BUFFER_SIZE_SENSOR);

    pthread_create(&task_sensor_image_handle, NULL, example_sensor_image_thread, &ExampleSensorInit);
    pthread_create(&task_send_inf_handle, NULL, example_send_inf_thread, &ExampleSensorInit.dwJobId);
    pthread_create(&task_recv_result_handle, NULL, example_recv_result_thread, NULL);
    pthread_create(&task_display_handle, NULL, example_display_liveview_thread, NULL);

    //WEBCAM
    pthread_create(&task_webcam_image_handle, NULL, example_webcam_input_thread, &ExampleWebCamInit);
    // pthread_create(&task_display_handle_webcam, NULL, example_display_liveview_thread_webcam, NULL);

    pthread_create(&task_buf_mgr_handle, NULL, VMF_NNM_Fifoq_Manager_Enqueue_Image_Thread, &_blFifoqManagerRunning);
    pthread_create(&task_inf_data_handle, NULL, VMF_NNM_Inference_Image_Dispatcher_Thread, &_blDispatchRunning);

    pthread_join(task_sensor_image_handle, NULL);
    pthread_join(task_send_inf_handle, NULL);
    pthread_join(task_recv_result_handle, NULL);
    pthread_join(task_display_handle, NULL);

    //WEBCAM
    pthread_join(task_webcam_image_handle, NULL);
    // pthread_join(task_display_handle_webcam, NULL);

    pthread_join(task_buf_mgr_handle, NULL);
    pthread_join(task_inf_data_handle, NULL);

    app_destroy();  //VMF_NNM_Inference_App_Destroy();
    VMF_NNM_Fifoq_Manager_Release_All_Buffer();
    ret = 0;

EXIT:
    printf("%s free\n", __func__);


    if (ExampleWebCamInit.pszModelPath)
        free(ExampleWebCamInit.pszModelPath);

    if (ExampleWebCamInit.pszCameraPath)
        free(ExampleWebCamInit.pszCameraPath);

    if(pszHostSensorConfigPath) {
        free(pszHostSensorConfigPath);
        pszHostSensorConfigPath = NULL;
    }

    free_fec_def_str();
    free_sensor_init(&ExampleSensorInit);

    printf("%s end\n", __func__);
    return ret;
}
