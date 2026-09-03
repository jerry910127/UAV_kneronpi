#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

//#include <opencv2/opencv.hpp>

extern "C" {
#include "kdp2_inf_app_yolo.h"
}

#include "example_shared_struct.h"
#include "kp_struct.h"

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

#if 0	//JDBG
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
        if (g_tInSsmBuf.buffer) {
            SSM_Reader_ReturnBuff(g_ptSsmReaderHandle, &g_tInSsmBuf);
            memset(&g_tInSsmBuf, 0, sizeof(g_tInSsmBuf));
        }
        SSM_Release(g_ptSsmReaderHandle);
        g_ptSsmReaderHandle = NULL;
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

#if 0	//JDBG
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
#endif
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

#if 0	//JDBG
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

            int x1 = std::max(0, std::min((int)std::lround(b.x1), cv_img_display->cols - 1));
            int y1 = std::max(0, std::min((int)std::lround(b.y1), cv_img_display->rows - 1));
            int x2 = std::max(0, std::min((int)std::lround(b.x2), cv_img_display->cols - 1));
            int y2 = std::max(0, std::min((int)std::lround(b.y2), cv_img_display->rows - 1));

            cv::rectangle(*cv_img_display, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(50, 255, 50), 2);

            const char *name = (b.class_num >= 0 && b.class_num < 80) ? COCO80[b.class_num] : "cls";
            char label[128];
            std::snprintf(label, sizeof(label), "%s %.1f%%", name, b.score * 100.0f);

            int baseline = 0;
            cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            int tx = x1;
            int ty = std::max(y1 - 5, ts.height + 5);

            cv::rectangle(*cv_img_display,
                          cv::Point(tx, ty - ts.height - baseline),
                          cv::Point(tx + ts.width + 2, ty + 2),
                          cv::Scalar(50, 255, 50), cv::FILLED);   // 底色
            cv::putText(*cv_img_display, label, cv::Point(tx + 1, ty),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);   // 黑字
        }
        pthread_mutex_unlock(&_mutex_result);

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
#endif

void *example_display_liveview_thread(void *)
{
    struct timeval time_begin;
    struct timeval time_end;
    float time_spent = 0.0;
    char strImgFPS[50] = "Image FPS: ";
    char strInfFPS[50] = "Inference FPS: ";
#if 0	//JDBG
    cv::Mat cv_image_source;
    cv::Mat cv_image_display;
#endif

//internet streaming
#define H26XE_DIS
#if defined(H26XE_API_H) && !defined(H26XE_DIS)
    SSM_BUFFER_T tWriterSsmBuffer;
    //std::vector<unsigned char> yuv_data;

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

    gettimeofday(&time_begin, NULL);

    while (true == _blDisplayRunning) 
    {
#if defined(H26XE_API_H) && !defined(H26XE_DIS)
        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);
#endif
        {
#if 0	//JDBG
            {
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
#endif

#if 0	//JDBG
            /* Display image */
            if (false == cv_image_display.empty()) {
                if (true == _inf_result.result_ready_display) {
                    //---- 計時用的的YOLO 畫框 ----
                    ScopeTimer T_yolo(&prof, "YOLO_Draw");
                    draw_display_image(&cv_image_display, strDistance, strQuality);
                }

                // cv::imshow("Inference Display", cv_image_display);
            }
#endif

#if 1	//JDBG
#if defined(H26XE_API_H) && !defined(H26XE_DIS)
            memcpy(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, (unsigned char *)_input_data.input_buf_address, _input_data.input_buf_size);
            MemBroker_CacheFlush(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, _input_data.input_buf_size);
#endif

#else
            if (!cv_image_display.empty()) {
                {
                    cv::imshow("Inference Display", cv_image_display);
#ifdef H26XE_API_H
            bgrToYUV420(cv_image_display, yuv_data);

            memcpy(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, yuv_data.data(), yuv_data.size());
            MemBroker_CacheFlush(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, yuv_data.size());
#endif
                }
            }
#endif

#if 0	//JDBG
            {
                /* Press 'ESC' to exit */
                if (27 == cv::waitKey(10)) {
                    sig_kill(0);
                    break;
                }
            }
#endif
        } //total 計算到這
    }
    
    _blImageRunning = false;
    _blSendInfRunning = false;
    _blResultRunning = false;

    _blDispatchRunning = false;
    _blFifoqManagerRunning = false;

#if defined(H26XE_API_H) && !defined(H26XE_DIS)
    if (g_ptSsmWriterHandle) {
        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);
        SSM_Release(g_ptSsmWriterHandle);
        g_ptSsmWriterHandle = NULL;
    }
#endif

    printf("[%s] bye!\r\n", __func__);
    return NULL;
}
