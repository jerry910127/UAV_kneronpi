#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

#include <algorithm>
#include <cmath>

#include <opencv2/opencv.hpp>

extern "C" {
#include "kdp2_inf_app_yolo.h"
}

#include "example_shared_struct.h"
#include "kp_struct.h"
#include "miramar_ctrl.h"

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

//mmlab 4/9
#include <atomic>
extern cv::Mat g_raw16_latest;
extern pthread_mutex_t g_mutex_raw16;
extern std::atomic<bool> g_capture_y16;
bool cur_y16 = g_capture_y16.load();
//mmlab end
bool _blDisplayRunning = true;

extern void sig_kill(int signo);

#if 1	//JDBG
#include "h26xenc_api.h"
#include <mem_broker.h>
#include <sync_shared_memory.h>
#include <sync_ring_buffer.h>
#include <ssm_info.h>
#include <resize.h>
#include <msg_broker.h>

#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <iniparser/iniparser.h>

#define CAM_WIDTH  480
#define CAM_HEIGHT 640

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
    if(g_ptSsmReaderHandle != NULL)
        SSM_Release(g_ptSsmReaderHandle);
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

int draw_display_image(cv::Mat *cv_img_display, const char *strImgFPS, const char *strInfFPS)
{
    int ret = KP_SUCCESS;
    kp_inference_header_stamp_t *header_stamp = (kp_inference_header_stamp_t *)_inf_result.result_buffer;

    if (KDP2_INF_ID_APP_YOLO == header_stamp->job_id)
    {
        pthread_mutex_lock(&_mutex_result);
        kdp2_ipc_app_yolo_result_t *app_yolo_result = (kdp2_ipc_app_yolo_result_t *)header_stamp;
        kp_app_yolo_result_t *yolo_result = (kp_app_yolo_result_t *)&app_yolo_result->yolo_data;

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

//#define H26XE_DIS
#if defined(H26XE_API_H) && !defined(H26XE_DIS)
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

    cv::namedWindow("Inference Display", cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_NORMAL);
    gettimeofday(&time_begin, NULL);

    // --- Miramar runtime status (AGC/CLAHE) ---
    int miramar_agc = -1;
    int miramar_clahe = -1;
    bool miramar_ctrl_ok = false;

    //rwei mmlab 4/2
    // --- Miramar runtime status (Color) ---
    int miramar_color = -1;
    // --- Miramar runtime status (ROI) --- 用不到暫時
    int miramar_roi = -1;

    // --- Host-side contrast stretch for display (helps when camera-side AGC is OFF) ---
    // Similar spirit to UVCThermalSensor.py: clip a dynamic range and linearly map to 0..255.
    // Here we do it on the host display image (8-bit) so the preview doesn't look "foggy" when AGC is disabled.
    int disp_min = 0;
    int disp_max = 255;
    int disp_frame_cnt = 0;

    // When camera-side AGC is toggled OFF, we can *freeze* the host-side stretch range
    // (lock disp_min/disp_max) so the rendering keeps the same appearance even if the
    // scene content changes. This mimics the ThermalCam app's "freeze" style behavior.
    bool disp_freeze = false;
    bool disp_freeze_pending = false;
    int miramar_agc_prev = -1;

    // --- Heating monitor (S: auto setup + baseline, X: start/stop monitoring) ---
    enum SetupState { SETUP_IDLE = 0, SETUP_AGC_CLAHE_ON, SETUP_CLAHE_OFF, SETUP_AGC_OFF, SETUP_DONE };
    SetupState setup_state = SETUP_IDLE;
    uint64_t setup_step_ms = 0;

    enum MonitorState { MON_OFF = 0, MON_CAPTURE_BASELINE, MON_RUNNING };
    MonitorState monitor_state = MON_OFF;
    bool baseline_ready = false;

    std::vector<cv::Rect> hot_boxes_cache;   // 升溫
    std::vector<cv::Rect> cold_boxes_cache;  // 降溫

    // Embedded-friendly params
    const double monitor_scale = 0.25;      // 640x480 -> 160x120
    float monitor_window_sec = 60.0f;       // time window (seconds)
    int monitor_thr = 12;                   // threshold in gray levels (tune)
    int monitor_min_area = 40;              // min blob area in SMALL image pixels
    int monitor_process_interval_ms = 200;  // run detection at 5Hz to save CPU

    uint64_t monitor_prev_ms = 0;
    uint64_t monitor_last_proc_ms = 0;

    cv::Mat mon_ref_f;      // CV_32F moving reference (EMA)
    cv::Mat mon_fixed_f;    // CV_32F fixed baseline right after S finishes
    cv::Mat mon_accum_f;    // CV_32F accumulator for baseline averaging
    int mon_accum_cnt = 0;
    const int mon_accum_N = 5;
    cv::Mat mon_hot_mask, mon_cold_mask;


    cv::Mat mon_gray8, mon_small8, mon_cur_f, mon_diff_f;
    cv::Mat mon_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    auto now_ms = []() -> uint64_t {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)tv.tv_usec / 1000ULL;
    };

    if (miramar_is_open()) {
        if (0 == miramar_get_agc_clahe(&miramar_agc, &miramar_clahe)) {
            miramar_ctrl_ok = true;
        }
    }

    miramar_agc_prev = miramar_agc;

    while (true == _blDisplayRunning) {

#if defined(H26XE_API_H) && !defined(H26XE_DIS)
        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);
#endif

        if (_result_count >= 60)
        {
            gettimeofday(&time_end, NULL);
            time_spent = (float)(time_end.tv_sec - time_begin.tv_sec) + (float)(time_end.tv_usec - time_begin.tv_usec) * .000001;
            sprintf(strImgFPS, "Image FPS: %.2lf", _image_count / time_spent);
            sprintf(strInfFPS, "Inference FPS: %.2lf", _result_count / time_spent);
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
        bool cur_y16 = g_capture_y16.load();

        // If camera-side AGC is OFF, the preview can look very low-contrast ("foggy").
        // Do a host-side contrast stretch (clip + linear mapping) similar to UVCThermalSensor.py.
        // This only affects the displayed image (cv_image_display), not the inference input.
        if (!cur_y16 || (false == cv_image_display.empty() && miramar_ctrl_ok && (miramar_agc == 0))){     //mmlab rwei 06/11  //(false == cv_image_display.empty() && miramar_ctrl_ok && (miramar_agc == 0)) {
            cv::Mat gray;
//rwei mmlab 4/2 Y16 邏輯
            pthread_mutex_lock(&g_mutex_raw16);
            cv::Mat disp_y16 = g_raw16_latest.clone();
            pthread_mutex_unlock(&g_mutex_raw16);
            gray = disp_y16.clone();        //使用raw 來AGC

            // 1. 計算總像素數量以及前 3% 所對應的數量 (k值)
            int total_pixels = disp_y16.total();
            int top_3_percent_count = total_pixels * 0.02;

            // 2. 將 Mat 的資料複製到 1D 的 std::vector 中，方便演算法處理
            // 這裡假設你的資料型態是 16-bit unsigned (CV_16U)。如果是 signed 請改為 int16_t。
            // 備註：因為你前面使用了 .clone()，記憶體保證是連續的，所以可以直接拷貝。
            std::vector<uint16_t> flat_data(disp_y16.begin<uint16_t>(), disp_y16.end<uint16_t>());

            // 3. 使用 std::nth_element 找出前 3% 的門檻值 (Threshold)
            // 這裡使用 std::greater<uint16_t>() 讓資料以「由大到小」的概念來尋找
            std::nth_element(flat_data.begin(), 
                            flat_data.begin() + top_3_percent_count, 
                            flat_data.end(), 
                            std::greater<uint16_t>());

            // 取出前 3% 的最低門檻值
            uint16_t threshold_value = flat_data[top_3_percent_count];
//            std::cout << "前 2% 的門檻值為: " << threshold_value << "對應溫度" << std::endl;
            
//rwei mmlab 4/2 END


//            cv::cvtColor(cv_image_display, gray, cv::COLOR_BGR2GRAY); //origin code

            // Update display range using the center ROI statistics (mean ± 3*std).
            //
            // Modes:
            //   - disp_freeze == false : auto-update periodically (similar to UVCThermalSensor.py)
            //   - disp_freeze == true  : keep disp_min/disp_max fixed (LOCK)
            //
            // When we detect camera-side AGC ON->OFF, we set disp_freeze_pending=true so the *next* frame
            // recomputes the range once and then locks it.
            const bool auto_update = (disp_freeze == false) && (disp_freeze_pending == false);
            const bool update_once_then_lock = (disp_freeze_pending == true);

            if (auto_update) {
                disp_frame_cnt++;
            }
//rwei mmlab 4/13  6/11 提醒我要將這個hardware-max 移除，變回255
            int hardware_max = 255; //int hardware_max = (!(miramar_agc || miramar_color)) ? 32767 : 255;
//            if(hardware_max==32767) printf("65535 !\n");
//rwei mmlab 4/13 end            
            if (update_once_then_lock || auto_update) {
                if (update_once_then_lock || disp_frame_cnt >= 8 || disp_max <= disp_min) {
                    disp_frame_cnt = 0;

                    const int cx = gray.cols / 2;
                    const int cy = gray.rows / 2;
                    const int qx = gray.cols / 4;
                    const int qy = gray.rows / 4;

                    const int x0 = std::max(0, cx - qx);
                    const int y0 = std::max(0, cy - qy);
                    const int x1 = std::min(gray.cols, cx + qx);
                    const int y1 = std::min(gray.rows, cy + qy);

                    cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
                    cv::Scalar mean, stddev;
                    cv::meanStdDev(gray(roi), mean, stddev);
                    disp_min = (int)std::lround(mean[0] - 3.0 * stddev[0]);
                    disp_max = (int)std::lround(mean[0] + 3.0 * stddev[0]);
//rwei mmlab 4/13
                    disp_min = std::max(0, std::min(hardware_max, disp_min));
                    disp_max = std::max(0, std::min(hardware_max, disp_max));
//rwei mmlab 4/13 end
                    /* origin code
                    disp_min = std::max(0, std::min(255, disp_min));
                    disp_max = std::max(0, std::min(255, disp_max));
                    */

                    // Avoid divide-by-zero
                    if (disp_max <= disp_min) {
                        disp_min = 0;
 //rwei mmlab 4/13
                        disp_max = hardware_max;
 //rwei mmlab 4/13 end                       
                        //disp_max = 255; //origin code
                    }

                    if (update_once_then_lock) {
                        disp_freeze = true;
                        disp_freeze_pending = false;
                    }
                }
            }
            cv::Mat clipped, scaled;
            cv::min(gray, disp_max, clipped);
            cv::max(clipped, disp_min, clipped);
            const double gain = -255.0 / (double)(disp_max - disp_min);
//rwei mmlab 4/13            
            const double offset = -disp_max * gain;
//origin code            const double offset = -disp_min * gain;

            clipped.convertTo(scaled, CV_8U, gain, offset);
            cv::cvtColor(scaled, cv_image_display, cv::COLOR_GRAY2BGR);
        }

        uint64_t tnow = now_ms();

        // ----------------------------
        // 1) S: auto setup state machine (NON-BLOCKING)
        //    Order MUST be:
        //      (AGC,CLAHE) = (1,1)  -> wait 1s -> (1,0) -> wait 1s -> (0,0)
        // ----------------------------
        if (setup_state == SETUP_AGC_CLAHE_ON) {
            if (tnow - setup_step_ms >= 1000) {
                if (miramar_is_open() &&
                    0 == miramar_set_agc_clahe(1, 0) &&
                    0 == miramar_get_agc_clahe(&miramar_agc, &miramar_clahe)) {
                    miramar_ctrl_ok = true;
                    setup_state = SETUP_CLAHE_OFF;
                    setup_step_ms = tnow;
                } else {
                    miramar_ctrl_ok = false;
                    setup_state = SETUP_IDLE;
                }
            }
        } else if (setup_state == SETUP_CLAHE_OFF) {
            if (tnow - setup_step_ms >= 1000) {
                int old_agc = miramar_agc;
                if (miramar_is_open() &&
                    0 == miramar_set_agc_clahe(0, 0) &&
                    0 == miramar_get_agc_clahe(&miramar_agc, &miramar_clahe)) {
                    miramar_ctrl_ok = true;

                    // mimic the existing A-key behavior: AGC ON->OFF triggers "recompute once then LOCK"
                    if (old_agc == 1 && miramar_agc == 0) {
                        disp_freeze = false;
                        disp_freeze_pending = true;
                        disp_frame_cnt = 0;
                    }

                    // reset baseline capture
                    baseline_ready = false;
                    mon_ref_f.release();
                    mon_fixed_f.release();
                    mon_accum_f.release();
                    mon_accum_cnt = 0;

                    setup_state = SETUP_AGC_OFF;
                    setup_step_ms = tnow;
                } else {
                    miramar_ctrl_ok = false;
                    setup_state = SETUP_IDLE;
                }
            }
        } else if (setup_state == SETUP_AGC_OFF) {
            // wait until host stretch is truly LOCKed, then mark setup DONE
            if (!cv_image_display.empty() && miramar_ctrl_ok &&
                (miramar_agc == 0) && (miramar_clahe == 0) &&
                (disp_freeze == true) && (disp_freeze_pending == false)) {

                // S 不再抓 baseline，只把環境固定到「可比較」狀態
                baseline_ready = false;
                mon_ref_f.release();
                mon_fixed_f.release();
                mon_accum_f.release();
                mon_accum_cnt = 0;

                hot_boxes_cache.clear();
                cold_boxes_cache.clear();

                setup_state = SETUP_DONE;
            }
        }


        // ----------------------------
        // 2) X: heating detection (run at low rate to save CPU)
        //    IMPORTANT: detect BEFORE drawing any overlays/boxes, so overlays won't create false alarms
        // ----------------------------
        
// ----------------------------
// 2) X: temperature change detection (run at low rate to save CPU)
//    IMPORTANT: detect BEFORE drawing any overlays/boxes, so overlays won't create false alarms
// ----------------------------
if ((monitor_state == MON_CAPTURE_BASELINE || monitor_state == MON_RUNNING) && !cv_image_display.empty()) {

    // Require stable rendering condition (AGC OFF, CLAHE OFF, Stretch LOCK) to make "difference" meaningful
    bool stable_mode = (miramar_ctrl_ok && (miramar_agc == 0) && (miramar_clahe == 0) && (disp_freeze == true));

    if (stable_mode) {
        if (monitor_last_proc_ms == 0 || (tnow - monitor_last_proc_ms) >= (uint64_t)monitor_process_interval_ms) {
            monitor_last_proc_ms = tnow;

            // current small frame (8-bit display image)
            cv::cvtColor(cv_image_display, mon_gray8, cv::COLOR_BGR2GRAY);
            cv::resize(mon_gray8, mon_small8, cv::Size(), monitor_scale, monitor_scale, cv::INTER_AREA);
            mon_small8.convertTo(mon_cur_f, CV_32F);

            if (mon_cur_f.empty()) {
                hot_boxes_cache.clear();
                cold_boxes_cache.clear();
            } else if (monitor_state == MON_CAPTURE_BASELINE) {
                // Capture baseline AFTER X: average first N frames
                if (mon_accum_f.empty()) {
                    mon_accum_f = cv::Mat::zeros(mon_cur_f.size(), CV_32F);
                    mon_accum_cnt = 0;
                }
                mon_accum_f += mon_cur_f;
                mon_accum_cnt++;

                if (mon_accum_cnt >= mon_accum_N) {
                    mon_fixed_f = mon_accum_f / (float)mon_accum_cnt;
                    mon_ref_f = mon_fixed_f.clone();

                    baseline_ready = true;
                    monitor_prev_ms = tnow;

                    hot_boxes_cache.clear();
                    cold_boxes_cache.clear();

                    monitor_state = MON_RUNNING;
                }
                // While capturing baseline, do not run diff detection
            } else if (monitor_state == MON_RUNNING && baseline_ready) {

                // EMA update with time window
                float dt = (monitor_prev_ms == 0) ? 0.0f : (float)(tnow - monitor_prev_ms) / 1000.0f;
                monitor_prev_ms = tnow;

                float alpha = (monitor_window_sec > 0.1f) ? (dt / monitor_window_sec) : 1.0f;
                if (alpha > 0.25f) alpha = 0.25f;   // cap for stability
                if (alpha < 0.0f) alpha = 0.0f;

                if (mon_ref_f.empty()) mon_ref_f = mon_cur_f.clone();
                mon_ref_f = mon_ref_f + alpha * (mon_cur_f - mon_ref_f);

                mon_diff_f = mon_cur_f - mon_ref_f;

                // hot: diff > +thr
                cv::threshold(mon_diff_f, mon_hot_mask, (double)monitor_thr, 255.0, cv::THRESH_BINARY);
                mon_hot_mask.convertTo(mon_hot_mask, CV_8U);

                // cold: diff <= -thr
                cv::threshold(mon_diff_f, mon_cold_mask, (double)(-monitor_thr), 255.0, cv::THRESH_BINARY_INV);
                mon_cold_mask.convertTo(mon_cold_mask, CV_8U);

                // If almost whole frame triggers, it's likely global shift (FFC/NUC); re-sync baseline
                cv::Mat any_mask;
                cv::bitwise_or(mon_hot_mask, mon_cold_mask, any_mask);

                if (cv::countNonZero(any_mask) > (int)(0.60 * any_mask.total())) {
                    mon_ref_f = mon_cur_f.clone();
                    hot_boxes_cache.clear();
                    cold_boxes_cache.clear();
                } else {
                    // denoise
                    cv::morphologyEx(mon_hot_mask, mon_hot_mask, cv::MORPH_OPEN, mon_kernel);
                    cv::morphologyEx(mon_hot_mask, mon_hot_mask, cv::MORPH_CLOSE, mon_kernel);

                    cv::morphologyEx(mon_cold_mask, mon_cold_mask, cv::MORPH_OPEN, mon_kernel);
                    cv::morphologyEx(mon_cold_mask, mon_cold_mask, cv::MORPH_CLOSE, mon_kernel);

                    const double inv_scale = 1.0 / monitor_scale;

                    // hot boxes (red)
                    {
                        std::vector<std::vector<cv::Point>> contours;
                        cv::findContours(mon_hot_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                        std::vector<cv::Rect> new_boxes;
                        for (auto &c : contours) {
                            if (cv::contourArea(c) < monitor_min_area) continue;

                            cv::Rect r = cv::boundingRect(c);
                            cv::Rect r_full(
                                (int)std::lround(r.x * inv_scale),
                                (int)std::lround(r.y * inv_scale),
                                (int)std::lround(r.width * inv_scale),
                                (int)std::lround(r.height * inv_scale)
                            );
                            r_full &= cv::Rect(0, 0, cv_image_display.cols, cv_image_display.rows);
                            if (r_full.area() > 0) new_boxes.push_back(r_full);
                        }
                        hot_boxes_cache.swap(new_boxes);
                    }

                    // cold boxes (blue)
                    {
                        std::vector<std::vector<cv::Point>> contours;
                        cv::findContours(mon_cold_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                        std::vector<cv::Rect> new_boxes;
                        for (auto &c : contours) {
                            if (cv::contourArea(c) < monitor_min_area) continue;

                            cv::Rect r = cv::boundingRect(c);
                            cv::Rect r_full(
                                (int)std::lround(r.x * inv_scale),
                                (int)std::lround(r.y * inv_scale),
                                (int)std::lround(r.width * inv_scale),
                                (int)std::lround(r.height * inv_scale)
                            );
                            r_full &= cv::Rect(0, 0, cv_image_display.cols, cv_image_display.rows);
                            if (r_full.area() > 0) new_boxes.push_back(r_full);
                        }
                        cold_boxes_cache.swap(new_boxes);
                    }
                }
            }
        }
    } else {
        // not stable -> clear boxes (avoid meaningless diff)
        hot_boxes_cache.clear();
        cold_boxes_cache.clear();
    }
} else {
    hot_boxes_cache.clear();
    cold_boxes_cache.clear();
}

/* Display image */
        if (false == cv_image_display.empty()) {
            if (true == _inf_result.result_ready_display) {
                draw_display_image(&cv_image_display, strImgFPS, strInfFPS);
            }

            // Draw temperature change boxes: HOT=red, COLD=blue
            for (const auto &r : hot_boxes_cache) {
                cv::rectangle(cv_image_display, r, cv::Scalar(0, 0, 255), 2);
            }
            for (const auto &r : cold_boxes_cache) {
                cv::rectangle(cv_image_display, r, cv::Scalar(255, 0, 0), 2);
            }
            // Show monitor/setup status
            char strHeat[128];
            
const char* setup_str =
    (setup_state == SETUP_IDLE ? "IDLE" :
     setup_state == SETUP_AGC_CLAHE_ON ? "AGC+CLAHE ON" :
     setup_state == SETUP_CLAHE_OFF ? "CLAHE OFF" :
     setup_state == SETUP_AGC_OFF ? "AGC OFF" :
     "DONE");
const char* mon_str =
    (monitor_state == MON_OFF ? "OFF" :
     monitor_state == MON_CAPTURE_BASELINE ? "BASELINE" :
     "ON");

sprintf(strHeat, "Setup[S]: %s   Mon[X]: %s  Win=%.0fs Thr=%d",
        setup_str, mon_str, monitor_window_sec, monitor_thr);
//            cv::putText(cv_image_display, strHeat, cv::Point(5, 140),
//                        cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 0), 1);


            // --- Overlay Miramar AGC/CLAHE status & hotkeys ---
            char strMiramar1[64];
            char strMiramar2[64];
            char strMiramar3[64];
            if (miramar_ctrl_ok) {
                sprintf(strMiramar1, "AGC [A]: %s", (miramar_agc == 1) ? "ON" : "OFF");
                sprintf(strMiramar2, "CLAHE [C]: %s", (miramar_clahe == 1) ? "ON" : "OFF");
                if (miramar_agc == 0) {
                    const char* stretch_mode = disp_freeze_pending ? "LOCK*" : (disp_freeze ? "LOCK" : "AUTO");
                    sprintf(strMiramar3, "Stretch [F]: %s (min=%d max=%d)", stretch_mode, disp_min, disp_max);
                } else {
                    sprintf(strMiramar3, "Stretch: OFF");
                }
            } else if (miramar_is_open()) {
                sprintf(strMiramar1, "AGC [A]: N/A");
                sprintf(strMiramar2, "CLAHE [C]: N/A");
                sprintf(strMiramar3, "Stretch: N/A");
            } else {
                sprintf(strMiramar1, "Miramar control: not open");
                sprintf(strMiramar2, "AGC[A]/CLAHE[C]: N/A");
                sprintf(strMiramar3, "Stretch: N/A");
            }

            cv::putText(cv_image_display, strMiramar1, cv::Point(5, 60), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 0), 1);
            cv::putText(cv_image_display, strMiramar2, cv::Point(5, 80), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 0), 1);
            cv::putText(cv_image_display, "Toggle: A=AGC, C=CLAHE, F=Stretch", cv::Point(5, 100), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 0), 1);
            int yStretch = std::max(20, cv_image_display.rows - 10);
            cv::putText(cv_image_display, strMiramar3, cv::Point(5, yStretch), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 0), 1);

            cv::imshow("Inference Display", cv_image_display);

#if defined(H26XE_API_H) && !defined(H26XE_DIS)
            bgrToYUV420(cv_image_display, yuv_data);

            memcpy(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, yuv_data.data(), yuv_data.size());
            MemBroker_CacheFlush(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, yuv_data.size());
#endif
        }

        // Keyboard controls:
        //   ESC : exit
        //   A/a : toggle AGC
        //   C/c : toggle CLAHE
        //   F/f : toggle host-side stretch mode (AUTO/LOCK) when AGC is OFF
        int key = cv::waitKey(10);
        key = (key >= 0) ? (key & 0xFF) : key;

        if (27 == key) { // ESC
            sig_kill(0);
            break;
        } else if ('a' == key || 'A' == key) {
            if (miramar_is_open()) {
                // Read current states from camera to avoid accidentally changing the other bit.
                if (0 == miramar_get_agc_clahe(&miramar_agc, &miramar_clahe)) {
                    int old_agc = miramar_agc;
                    int new_agc = (miramar_agc == 1) ? 0 : 1;
                    if (0 == miramar_set_agc_clahe(new_agc, miramar_clahe)) {
                        // verify
                        if (0 == miramar_get_agc_clahe(&miramar_agc, &miramar_clahe)) {
                            miramar_ctrl_ok = true;

                            // Host-side stretch behavior:
                            // - When AGC switches ON -> OFF, recompute once on next frame and then LOCK.
                            // - When AGC switches OFF -> ON, stop stretch and clear LOCK state.
                            if (old_agc == 1 && miramar_agc == 0) {
                                disp_freeze = false;
                                disp_freeze_pending = true;
                                disp_frame_cnt = 0;
                            } else if (old_agc == 0 && miramar_agc == 1) {
                                disp_freeze = false;
                                disp_freeze_pending = false;
                                disp_min = 0;
                                disp_max = 255;
                                disp_frame_cnt = 0;
                            }
                            miramar_agc_prev = miramar_agc;
                        } else {
                            miramar_ctrl_ok = false;
                        }
                    } else {
                        miramar_ctrl_ok = false;
                    }
                } else {
                    miramar_ctrl_ok = false;
                }
            } else {
                miramar_ctrl_ok = false;
            }
        } else if ('c' == key || 'C' == key) {
            if (miramar_is_open()) {
                // Read current states from camera to avoid accidentally changing the other bit.
                if (0 == miramar_get_agc_clahe(&miramar_agc, &miramar_clahe)) {
                    int new_clahe = (miramar_clahe == 1) ? 0 : 1;
                    if (0 == miramar_set_agc_clahe(miramar_agc, new_clahe)) {
                        // verify
                        if (0 == miramar_get_agc_clahe(&miramar_agc, &miramar_clahe)) {
                            miramar_ctrl_ok = true;
                        } else {
                            miramar_ctrl_ok = false;
                        }
                    } else {
                        miramar_ctrl_ok = false;
                    }
                } else {
                    miramar_ctrl_ok = false;
                }
            } else {
                miramar_ctrl_ok = false;
            }
        } else if ('f' == key || 'F' == key) {
            // Toggle host-side stretch AUTO/LOCK when camera-side AGC is OFF
            if (miramar_ctrl_ok && (miramar_agc == 0)) {
                disp_freeze = !disp_freeze;
                if (!disp_freeze) {
                    // Switching to AUTO: recompute quickly
                    disp_frame_cnt = 0;
                }
            }
        }
        else if ('z' == key || 'Z' == key) {
            // Z: preset stretch range to [110,140] and force LOCK (disable AUTO)
            // Only meaningful when camera-side AGC is OFF (host stretch is used)
            if (miramar_ctrl_ok && (miramar_agc == 0)) {
                disp_freeze = true;          // LOCK
                disp_freeze_pending = false; // ensure no pending auto-lock overrides
                disp_min = 110;
                disp_max = 140;
            }
        }

        else if ('s' == key || 'S' == key) {
            // S: start the REQUIRED setup sequence: (1,1)->1s->(1,0)->1s->(0,0), baseline is captured AFTER X
            if (miramar_is_open()) {
                // force (AGC,CLAHE) = (1,1)
                if (0 == miramar_set_agc_clahe(1, 1) &&
                    0 == miramar_get_agc_clahe(&miramar_agc, &miramar_clahe)) {

                    miramar_ctrl_ok = true;

                    // When AGC is ON, host stretch should be cleared
                    disp_freeze = false;
                    disp_freeze_pending = false;
                    disp_min = 0;
                    disp_max = 255;
                    disp_frame_cnt = 0;

                    // reset monitoring/baseline (S does NOT capture baseline; baseline is captured after X)
                    monitor_state = MON_OFF;
                    baseline_ready = false;
                    mon_ref_f.release();
                    mon_fixed_f.release();
                    mon_accum_f.release();
                    mon_accum_cnt = 0;
                    hot_boxes_cache.clear();
                    cold_boxes_cache.clear();

                    setup_state = SETUP_AGC_CLAHE_ON;
                    setup_step_ms = now_ms();
                } else {
                    miramar_ctrl_ok = false;
                    setup_state = SETUP_IDLE;
                }
            }
        }
        else if ('x' == key || 'X' == key) {
            // 只能在 S 已經 DONE 後才允許開始監控
            if (setup_state == SETUP_DONE) {
                if (monitor_state == MON_OFF) {
                    // 開始：先抓 baseline
                    baseline_ready = false;
                    mon_ref_f.release();
                    mon_fixed_f.release();
                    mon_accum_f.release();
                    mon_accum_cnt = 0;

                    hot_boxes_cache.clear();
                    cold_boxes_cache.clear();

                    monitor_state = MON_CAPTURE_BASELINE;
                } else {
                    // 停止
                    monitor_state = MON_OFF;
                    baseline_ready = false;
                    hot_boxes_cache.clear();
                    cold_boxes_cache.clear();
                }
            }
        }
        else if ('k' == key || 'K' == key) {
            if (miramar_is_open()) {
                // Read current states from camera to avoid accidentally changing the other bit.
                if (0 == miramar_set_color(&miramar_color)) {
                } else {
                    miramar_ctrl_ok = false;
                }
            } else {
                miramar_ctrl_ok = false;
            }
        }
        else if ('l' == key || 'L' == key) {
            if (miramar_is_open()) {
                // Read current states from camera to avoid accidentally changing the other bit.
                if (0 == miramar_read_ctl_roi(&miramar_roi)) {
                } else {
                    miramar_ctrl_ok = false;
                }
                printf("miramar_roi: %d \n",miramar_roi);
            } else {
                miramar_ctrl_ok = false;
            }
        }

        // (Optional) quick presets for window/threshold tuning
        else if ('1' == key) { monitor_window_sec = 60.0f; }
        else if ('3' == key) { monitor_window_sec = 180.0f; }
        else if ('h' == key || 'H' == key) { monitor_window_sec = 3600.0f; }
        else if ('+' == key || '=' == key) { monitor_thr = std::min(255, monitor_thr + 1); }
        else if ('-' == key || '_' == key) { monitor_thr = std::max(0, monitor_thr - 1); }

    }

    _blImageRunning = false;
    _blSendInfRunning = false;
    _blResultRunning = false;

    _blDispatchRunning = false;
    _blFifoqManagerRunning = false;

#if defined(H26XE_API_H) && !defined(H26XE_DIS)
    if (g_ptSsmWriterHandle)
    {
        SSM_Release(g_ptSsmWriterHandle);
    }
#endif

    printf("[%s] bye!\r\n", __func__);
    return NULL;
}
