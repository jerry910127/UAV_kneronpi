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

#include <opencv2/opencv.hpp>

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
#define IMAGE_BUFFER_COUNT      3

#define RESULT_BUFFER_COUNT     3
#define RESULT_BUFFER_SIZE      512 * 1024

#define EXAMPLE_SENSOR_CONFIG_PATH "./ini/example_sensor.ini"

#define IMU_CONFIG_PATH             "/sys/bus/iio/devices/iio:device1/buffer/enable"

extern void *example_sensor_image_thread(void *arg);
extern void *example_send_inf_thread(void *arg);
extern void *example_recv_result_thread(void *arg);
extern void *example_display_liveview_thread(void *arg);

bool _blDispatchRunning = true;
bool _blFifoqManagerRunning = true;
extern bool _blImageRunning;
extern bool _blSendInfRunning;
extern bool _blResultRunning;
extern bool _blDisplayRunning;

extern ssm_handle_t  	*gptSsmHandle;

#if 1	//JDBG
#include <msg_broker.h>

#include "encode.h"

#define VENC_CMD_FIFO       "/tmp/venc/c0/command.fifo"

#define UNSET_INT	65536

#define VMF_H26XENC_MIN_QP	0
#define VMF_H26XENC_MAX_QP	51

int g_bTerminate = 0;
unsigned int g_dwStreamingNum = 0;

extern VEncoder gVenc;

void set_vic_fps_option(unsigned int dwFrameRate);
#endif

int loadConfig(const char* HostSensorConfigPath, EXAMPLE_SENSOR_INIT_OPT_T* pExampleSensorInit)
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

    ini = iniparser_load(HostSensorConfigPath);

    tmp = iniparser_getstring(ini, "sensor:sensor_cfg", NULL);
    if (tmp)
        pExampleSensorInit->tSensorConf.pszSensorConfigPath = strdup(tmp);

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
#ifdef VENC_API_H
    g_bTerminate = 1;
#endif

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

#ifdef VENC_API_H
void chk_fill_uint(unsigned int* target, int value)
{
	if( UNSET_INT != value )
		*target = value;
}

void chk_fill_float(float* target, int value)
{
	if( UNSET_INT != value )
		*target = value;
}

void change_video( void* data )
{
	msg_video_t* video_msg = (msg_video_t*) data;

	if (video_msg)  {
		printf("[streamer_msg] %s() changeVideo: type(%u), width(%u), height(%u), fps(%f), gop(%u), qp(%u), bitrate(%u), min_IQP(%u), max_IQP(%u), min_PQP(%u), max_PQP(%u)\n",
				__func__, video_msg->type, video_msg->w, video_msg->h, video_msg->fps, video_msg->gop, video_msg->qp, video_msg->bitrate, video_msg->minIQp, video_msg->maxIQp, video_msg->minPQp, video_msg->maxPQp);

		VEncoder *encoder = &gVenc;
		VMF_VENC_CONFIG_T* config = &encoder->config;
		unsigned int dwOldWidth = config->dwEncWidth;
		unsigned int dwOldHeight = config->dwEncHeight;

		if ( ((UNSET_INT == (int)video_msg->type) && (config->eCodecType == VMF_VENC_CODEC_TYPE_H264 || config->eCodecType == VMF_VENC_CODEC_TYPE_H265)) || 
			((UNSET_INT != (int)video_msg->type) && (video_msg->type == VMF_VENC_CODEC_TYPE_H264 || video_msg->type == VMF_VENC_CODEC_TYPE_H265))) {
			if ( (video_msg->qp != UNSET_INT && video_msg->qp > VMF_H26XENC_MAX_QP) ||
				(video_msg->maxIQp != UNSET_INT && video_msg->maxIQp > VMF_H26XENC_MAX_QP) ||
				(video_msg->maxPQp != UNSET_INT && video_msg->maxPQp > VMF_H26XENC_MAX_QP) ) {
				printf("[streamer_msg] %s() QP should be 0 ~ 51\n", __func__);
	 			return;
			}
		}

		config->dwReInit = 0;
		if ((video_msg->bitrate == 0 && encoder->bitrate != 0) || 
			(UNSET_INT != video_msg->bitrate && video_msg->bitrate != 0 && encoder->bitrate == 0) ) {
			config->dwReInit = 1;
		}

		if( (UNSET_INT != (int)video_msg->type) && (config->eCodecType != video_msg->type) ) {
			config->eCodecType = (VMF_VENC_CODEC_TYPE)video_msg->type;
    
			if (video_msg->type == VMF_VENC_CODEC_TYPE_H264) {
				encoder->profile = VMF_H264ENC_HIGH;
			} else if (video_msg->type == VMF_VENC_CODEC_TYPE_H265) {
				encoder->profile = VMF_H265ENC_MAIN;
			}
                
			if( -1 == venc_init_codec(config) ) {
				printf("[streamer_msg] Init codec structure fail.\n");
				return;
			}
		}
		config->bEnableCropping = 0;
		config->dwCropStartX = 0;
		config->dwCropStartY = 0;
		config->dwRequestWidth = 0;
		config->dwRequestHeight = 0;
		config->bWaitComplete = 1;

		chk_fill_uint((unsigned int*)&config->eCodecType, video_msg->type);
		chk_fill_uint(&config->dwEncWidth, video_msg->w);
		chk_fill_uint(&config->dwEncHeight, video_msg->h);
#if 1	//JDBG
		printf("[chgV] oldWH:%d, %d ?= encWH:%d, %d\n",
			dwOldWidth, dwOldHeight,
			config->dwEncWidth, config->dwEncHeight
			);
#else
		if(dwOldWidth != config->dwEncWidth || dwOldHeight != config->dwEncHeight) {
			if(0 != check_layout_and_size(fec_get_mode(gStreams[0].fec_handle))) {
				config->dwEncWidth = dwOldWidth;
				config->dwEncHeight = dwOldHeight;
			}
		}
#endif
		chk_fill_float(&encoder->fps, video_msg->fps);
		chk_fill_uint(&encoder->bitrate, video_msg->bitrate);
		chk_fill_uint(&encoder->gop, video_msg->gop);
		chk_fill_uint(&encoder->qp, video_msg->qp);
		chk_fill_uint(&encoder->minIQp, video_msg->minIQp);
		chk_fill_uint(&encoder->maxIQp, video_msg->maxIQp);
		chk_fill_uint(&encoder->minPQp, video_msg->minPQp);
		chk_fill_uint(&encoder->maxPQp, video_msg->maxPQp);

		if (SRB == g_eType)
			VMF_VENC_OUT_SRB_Setup_Config(&encoder->config, encoder->config.eCodecType, encoder->config.pCodecConfig, encoder->srb_handle);
		else if (SCM == g_eType)
			VMF_VENC_OUT_SCM_Setup_Config(&encoder->config, encoder->config.eCodecType, encoder->config.pCodecConfig, encoder->scm_handle);
		venc_config(encoder);
		VMF_VENC_ProduceStreamHdr(encoder->handle);
	}
	else  {
		printf("[streamer_msg] %s() video_msg is NULL\n", __func__);
	}
}

void msg_callback(MsgContext* msg_context, void* user_data)
{
	(void) user_data;
	printf("msg_context->pszHost=%s, msg_context->pszCmd=%s \n", 
			msg_context->pszHost, msg_context->pszCmd);

	if (!strcasecmp(msg_context->pszHost, "encoder0")) {
		if (!strcasecmp(msg_context->pszCmd, "start")) {
			if (++g_dwStreamingNum == 1) {
				venc_start(&gVenc);
			}
		} else if (!strcasecmp(msg_context->pszCmd, "stop")) {
			if (g_dwStreamingNum) {
				if (--g_dwStreamingNum == 0) {
					venc_stop(&gVenc);
				}
			}
		} else if (!strcasecmp(msg_context->pszCmd, "forceCI")) {
			VMF_VENC_ProduceStreamHdr(gVenc.handle);
		} else if (!strcasecmp(msg_context->pszCmd, "forceIntra")) {
			VMF_VENC_SetIntra(gVenc.handle);
		}
		else  if (!strcasecmp(msg_context->pszCmd, "changeVideo"))  {
			change_video(msg_context->pbyData);
		}
	}
	else if ( !strncasecmp(msg_context->pszHost, "vic", 3) )  {
		if( !strcasecmp(msg_context->pszCmd, "setVicFps") )  {
			unsigned int* dwUvcYuvFps = (unsigned int*) msg_context->pbyData;
			set_vic_fps_option(*dwUvcYuvFps);
		}
	}

	if (msg_context->bHasResponse) {
		msg_context->dwDataSize = 0;
	}
}
#endif

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

    int ret = KP_SUCCESS;
    EXAMPLE_SENSOR_INIT_OPT_T ExampleSensorInit; //for host sensor mode/thread
    uint32_t ImageBufferSize = 0;

    memset(&ExampleSensorInit, 0, sizeof(EXAMPLE_SENSOR_INIT_OPT_T));

    if (argc > 1) {
        pszHostSensorConfigPath = parse_argument(argc, argv);
        if(pszHostSensorConfigPath == NULL)   //parse argument
            goto EXIT;

        if(0 != loadConfig(pszHostSensorConfigPath, &ExampleSensorInit))                  // load ini file
            goto EXIT;

    } else {
        if(0 != loadConfig(EXAMPLE_SENSOR_CONFIG_PATH, &ExampleSensorInit))                  // load ini file
            goto EXIT;
    }

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

#if 0	//JDBG
    pthread_create(&task_sensor_image_handle, NULL, example_sensor_image_thread, &ExampleSensorInit);
    pthread_create(&task_display_handle, NULL, example_display_liveview_thread, NULL);

    /* register the message communication pipe */
    MsgBroker_RegisterMsg(VENC_CMD_FIFO);
    MsgBroker_Run(VENC_CMD_FIFO, msg_callback, NULL, &g_bTerminate);
    MsgBroker_UnRegisterMsg();

    pthread_join(task_sensor_image_handle, NULL);
    pthread_join(task_display_handle, NULL);

#else
    printf("[%s] app_initialize \n", __func__);
    app_initialize();

    VMF_NNM_Load_Model_From_File(ExampleSensorInit.pszModelPath);

    VMF_NNM_Fifoq_Manager_Allocate_Buffer(IMAGE_BUFFER_COUNT, ImageBufferSize, RESULT_BUFFER_COUNT, RESULT_BUFFER_SIZE);

    pthread_create(&task_sensor_image_handle, NULL, example_sensor_image_thread, &ExampleSensorInit);
    pthread_create(&task_send_inf_handle, NULL, example_send_inf_thread, &ExampleSensorInit.dwJobId);
    pthread_create(&task_recv_result_handle, NULL, example_recv_result_thread, NULL);
    pthread_create(&task_display_handle, NULL, example_display_liveview_thread, NULL);

    pthread_create(&task_buf_mgr_handle, NULL, VMF_NNM_Fifoq_Manager_Enqueue_Image_Thread, &_blFifoqManagerRunning);
    pthread_create(&task_inf_data_handle, NULL, VMF_NNM_Inference_Image_Dispatcher_Thread, &_blDispatchRunning);

#ifdef VENC_API_H
    /* register the message communication pipe */
    MsgBroker_RegisterMsg(VENC_CMD_FIFO);
    MsgBroker_Run(VENC_CMD_FIFO, msg_callback, NULL, &g_bTerminate);
    MsgBroker_UnRegisterMsg();
#endif

    pthread_join(task_sensor_image_handle, NULL);
    pthread_join(task_send_inf_handle, NULL);
    pthread_join(task_recv_result_handle, NULL);
    pthread_join(task_display_handle, NULL);

    pthread_join(task_buf_mgr_handle, NULL);
    pthread_join(task_inf_data_handle, NULL);

    app_destroy();  //VMF_NNM_Inference_App_Destroy();
    VMF_NNM_Fifoq_Manager_Release_All_Buffer();
#endif
    ret = 0;

EXIT:
    printf("%s free\n", __func__);

    if(pszHostSensorConfigPath) {
        free(pszHostSensorConfigPath);
        pszHostSensorConfigPath = NULL;
    }

    free_fec_def_str();
    free_sensor_init(&ExampleSensorInit);

    printf("%s end\n", __func__);
    return ret;
}
