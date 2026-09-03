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
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include "video_buf.h"
#include "ssm_info.h"
#include "sync_shared_memory.h"

#include <sys/stat.h>
#include <stdbool.h>
#include <sync_ring_buffer.h>
#include <resize.h>
#include <msg_broker.h>

#include <iniparser/iniparser.h>
#include "h26xenc_api.h"

#if 1	//JDBG
#include <sched.h>

#define PTH_PRIO
#endif

#include "buildcfg.h"

#define EXAMPLE_WEBCAM_CONFIG_PATH "./ini/example_webcam.ini"

#define VENC_CMD_FIFO	"/tmp/venc/c0/command.fifo" //! communicate with rtsps, vrec, etc.

#ifndef MAKEFOURCC
#define MAKEFOURCC(a,b,c,d) (((uint32_t)a) | (((uint32_t)b)<<8) | \
		(((uint32_t)c)<<16) | (((uint32_t)d)<<24))
#endif
#define ptEncBuff_SIZE 1024*1024*8	//2000000
#define FOURCC_CONF		(MAKEFOURCC('C','O','N','F'))
#define FOURCC_H264		(MAKEFOURCC('H','2','6','4'))
#define FOURCC_H265		(MAKEFOURCC('H','2','6','5'))
#define UBUFFER_HEADERSIZE	256

/**
 * @brief describe example webcam configuration
 */
typedef struct
{
    char* pszModelPath;             //! Path of NN model file
    unsigned int dwJobId;           //e.g. KDP2_INF_ID_APP_YOLO;

    char* pszCameraPath;
    unsigned int dwImageWidth;      //! Input image width
    unsigned int dwImageHeight;     //! Input image height
    unsigned int dwFps;             // feed image fps
} EXAMPLE_WEBCAM_INIT_OPT_T;

VMF_CODEC_INITOPT_T codec_initopt;

char *g_szSsmReaderPin = NULL;
char *g_szSrbWriterPin = NULL;
int g_bTerminate = 0;
int g_eCodec = 0;
int g_bWriteEnable = 0;
bool g_bConfFlag = true;
unsigned int g_dwTargetW = 0;
unsigned int g_dwTargetH = 0;

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

#ifdef __MAIN__
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

    //internet streaming
    H26xEnc_loadConfig(ini);

    iniparser_freedict(ini);
	return 0;
}
#endif

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
    VMF_VIDEO_BUF_T tRsOutbuf;
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
#if 0	//v1.2.0
    codec_initopt.dwImgWidth = codec_initopt.dwEncWidth = g_dwTargetW;
    codec_initopt.dwImgHeight = codec_initopt.dwEncHeight = g_dwTargetH;
    codec_initopt.dwMaxWidth = VMF_32_ALIGN(codec_initopt.dwImgWidth);
    codec_initopt.dwMaxHeight = VMF_16_ALIGN(codec_initopt.dwImgHeight);
    codec_initopt.dwMaxUvWidth = codec_initopt.dwMaxWidth>>1;

#else	//v1.0.0
    codec_initopt.dwSrcWidth = codec_initopt.dwEncWidth = g_dwTargetW;
    codec_initopt.dwSrcHeight = codec_initopt.dwEncHeight = g_dwTargetH;
    codec_initopt.dwSrcStride = ((codec_initopt.dwSrcWidth+31)&(~31));
#endif
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

#if 0	//JDBG, why added this?
        SSM_Reader_ReturnBuff(g_ptSsmReaderHandle, &g_tInSsmBuf);
#endif

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


static void signal_handler(int signo)
{
    (void) signo;
    g_bTerminate = 1;
    H26xEnc_IPC_Wake();

    if (g_ptSsmReaderHandle)
        SSM_Reader_Wakeup(g_ptSsmReaderHandle);
}

void msg_callback( MsgContext* msg_context, void* user_data )
{
    (void) user_data;

    printf("[h26xEncStrm] Video msg_context->pszHost=%s, msg_context->pszCmd=%s\n", msg_context->pszHost, msg_context->pszCmd);

    if ( !strncasecmp(msg_context->pszHost, "encoder", 7) )  {
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

#ifdef __MAIN__
int main(int argc, char **argv)
{
    pthread_t task_h26xenc_handle;
#ifdef PTH_PRIO
    pthread_attr_t attr;
    struct sched_param param;
    int policy = SCHED_FIFO;
    int max_prio;

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, policy);
    max_prio = sched_get_priority_max(policy);
    param.sched_priority = 1;//max_prio > 0 ? max_prio : 1;
    pthread_attr_setschedparam(&attr, &param);
    printf("[%s] pth prio: %d\r\n", __func__, param.sched_priority);
#endif

    EXAMPLE_WEBCAM_INIT_OPT_T ExampleWebCamInit;

    signal(SIGTERM, signal_handler);
    signal(SIGKILL, signal_handler);
    signal(SIGINT, signal_handler);

    struct sigaction sa;
    memset(&sa, 0, sizeof(struct sigaction));
    sigemptyset(&sa.sa_mask);
    sa.sa_handler  = signal_handler;
    sa.sa_flags = SA_SIGINFO|SA_RESETHAND;  // Reset signal handler to system default after signal triggered
    sigaction(SIGSEGV, &sa, NULL);

    memset(&ExampleWebCamInit, 0, sizeof(EXAMPLE_WEBCAM_INIT_OPT_T));
    if(0 != loadConfig_webcam(EXAMPLE_WEBCAM_CONFIG_PATH, &ExampleWebCamInit))                // load ini file
        return -1;

    H26xEnc_IPC_Init();
#ifdef PTH_PRIO
    pthread_create(&task_h26xenc_handle, &attr, h26xenc_loop, NULL);
#else
    pthread_create(&task_h26xenc_handle, NULL, h26xenc_loop, NULL);
#endif

    /* register the message communication pipe */
    MsgBroker_RegisterMsg(VENC_CMD_FIFO);
    MsgBroker_Run(VENC_CMD_FIFO, msg_callback, NULL, &g_bTerminate);
    MsgBroker_UnRegisterMsg();

    pthread_join(task_h26xenc_handle, NULL);
    H26xEnc_IPC_Release();

    if (ExampleWebCamInit.pszModelPath)
        free(ExampleWebCamInit.pszModelPath);

    if (ExampleWebCamInit.pszCameraPath)
        free(ExampleWebCamInit.pszCameraPath);

#ifdef PTH_PRIO
    pthread_attr_destroy(&attr);
#endif

    printf("%s end\r\n", __func__);
    return 0;
}
#endif
