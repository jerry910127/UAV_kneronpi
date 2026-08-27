
/*
*******************************************************************************
*  Copyright (c) 2010-2015 VATICS Inc. All rights reserved.
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
*  | VATICS INC.                                                     |
*  +-----------------------------------------------------------------+
*
*******************************************************************************
*/
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <fcntl.h>
#include <time.h>
#include <sys/stat.h>
#include <sched.h>
#include <errno.h>
#include "h26xenc_api.h"
#include <iniparser.h>
#include <mem_broker.h>
#include <sync_shared_memory.h>
#include <ssm_info.h>
#include <vmf_log.h>
#include <video_encoder_output_srb.h>
#include <video_encoder.h>
#include <resize.h>
#include <msg_broker.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include "videodev2.h"

#if 1	//JDBG
//#define V4L2_PIX_FMT	V4L2_PIX_FMT_MJPEG
#define V4L2_PIX_FMT	V4L2_PIX_FMT_YUYV

#define CAM_WIDTH  480	//640
#define CAM_HEIGHT 640	//480
#else
#define CAM_WIDTH 1280 //640
#define CAM_HEIGHT 720 //480
#endif
#define CAM_FPS 15
#define BUF_NUMBER 4
struct Device {
    int iFd;
    enum v4l2_buf_type eBufType;
    enum v4l2_memory eMemType;
    unsigned int dwWidth;
    unsigned int dwHeight;
};

struct MapBuf {
    unsigned int dwSize[VIDEO_MAX_PLANES];
    void *pbyMem[VIDEO_MAX_PLANES];
};

extern VMF_H26XENC_HANDLE_T* h26xe_handle;
extern VMF_H26XENC_STATE_T* h26xe_state;
#ifndef MAKEFOURCC
#define MAKEFOURCC(a,b,c,d) (((uint32_t)a) | (((uint32_t)b)<<8) | \
		(((uint32_t)c)<<16) | (((uint32_t)d)<<24))
#endif
#define ptEncBuff_SIZE 1024*1024*8	//2000000
#define FOURCC_CONF			 (MAKEFOURCC('C','O','N','F'))
#define FOURCC_H264			 (MAKEFOURCC('H','2','6','4'))
#define FOURCC_H265			 (MAKEFOURCC('H','2','6','5'))
#define VENC_CMD_FIFO		   "/tmp/venc/c1/command.fifo" //! communicate with rtsps, vrec, etc.
#define UBUFFER_HEADERSIZE	  256
VMF_CODEC_INITOPT_T codec_initopt;

char *g_azH26xe_conf =NULL;
char *g_szSsmReaderPin = NULL;
char *g_szSrbWriterPin = NULL;
char *g_szCameraPath = NULL;
int g_bTerminate = 0;
int g_eCodec = 0;
int g_bWriteEnable = 0;
bool g_bConfFlag = true;
unsigned int g_dwTargetW = 0;
unsigned int g_dwTargetH = 0;

SSM_WRITER_INIT_OPTION_T ssm_opt;
SSM_HANDLE_T* g_ptSsmWriterHandle = NULL;

//static bool bVideoConfFlag = true;
static SRB_HANDLE_T* g_ptVideoSrbHandle = NULL;
static SSM_HANDLE_T* g_ptSsmReaderHandle = NULL;
static VMF_RS_HANDLE_T* g_ptResizeHandle = NULL;
static SSM_BUFFER_T g_tInSsmBuf;
static SRB_BUFFER_T g_tVideoSrbWriterBuf;
static VMF_VSRC_SSM_OUTPUT_INFO_T  g_tInSsmInfo;
static VMF_VENC_H265_STREAM_HDR * g_ptStreamHdr;

static pthread_cond_t g_tVideoCond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t g_tVideoMutex = PTHREAD_MUTEX_INITIALIZER;

struct timeval tv_prev, tv;
unsigned long total_time = 0;
typedef enum
{
    SEND_CONF = 0,
    SEND_DATA
} SEND_STATE;

#ifdef V4L2_PIX_FMT
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

static int init_cam(struct Device *pDev, const char *pszDeviceName, unsigned int dwWidth, unsigned int dwHeight)
{
    pDev->eBufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    pDev->eMemType = V4L2_MEMORY_MMAP;
    pDev->dwWidth = dwWidth;
    pDev->dwHeight = dwHeight;
    pDev->iFd = open(pszDeviceName, O_RDWR);
    if (pDev->iFd < 0)
    {
        pDev->iFd = 0;
        printf("Open device failed\n");
    }
    return -1;
}

static int close_cam(struct Device *pDev)
{
    if (pDev->iFd)
    {
        close(pDev->iFd);
        pDev->iFd = 0;
    }
    return pDev->iFd;
}

static int config_cam(struct Device *pDev)
{
    struct v4l2_format tFormat;
    struct v4l2_fmtdesc tFmtdesc;
    struct v4l2_streamparm tParm;
    unsigned int i = 0;
    //! enum format
    printf("Available formats:\n");
    memset(&tFmtdesc, 0, sizeof tFmtdesc);
    tFmtdesc.index = i;
    tFmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (ioctl(pDev->iFd, VIDIOC_ENUM_FMT, &tFmtdesc) != -1)
    {
        printf("\t%d.%s (0x%08x)\n", tFmtdesc.index + 1, tFmtdesc.description, tFmtdesc.pixelformat);
        tFmtdesc.index++;
    }

    //! set format
    tFormat.type = pDev->eBufType;
    tFormat.fmt.pix.width = pDev->dwWidth;
    tFormat.fmt.pix.height = pDev->dwHeight;
#ifdef V4L2_PIX_FMT	//JDBG
    tFormat.fmt.pix.pixelformat = V4L2_PIX_FMT;
    printf("Set pixelformat 0x%08x\r\n", tFormat.fmt.pix.pixelformat);
#else
    tFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
#endif
    tFormat.fmt.pix.field = V4L2_FIELD_NONE;
    tFormat.fmt.pix.bytesperline = 0;
    tFormat.fmt.pix.sizeimage = 0;
    tFormat.fmt.pix.priv = V4L2_PIX_FMT_PRIV_MAGIC;
    tFormat.fmt.pix.flags = 0;

    if (ioctl(pDev->iFd, VIDIOC_S_FMT, &tFormat) < 0)
    {
        printf("Config cam failed\n");
        return -1;
    }
#ifdef V4L2_PIX_FMT	//JDBG
    {
    char pxFmt[5] = {0};
    char *p_s8 = (char *)&tFormat.fmt.pix.pixelformat;
    memcpy( pxFmt, p_s8, sizeof(tFormat.fmt.pix.pixelformat) );
    printf("Config format: %s (0x%08x) %ux%u (stride %u) field %s buffer size %u\n",
            pxFmt, tFormat.fmt.pix.pixelformat,
	    tFormat.fmt.pix.width, tFormat.fmt.pix.height,
	    tFormat.fmt.pix.bytesperline, "none", tFormat.fmt.pix.sizeimage);
    }
#else
    printf("Config format: %s (0x%08x) %ux%u (stride %u) field %s buffer size %u\n", "MJPEG", tFormat.fmt.pix.pixelformat, tFormat.fmt.pix.width, tFormat.fmt.pix.height, tFormat.fmt.pix.bytesperline, "none", tFormat.fmt.pix.sizeimage);
#endif

    //! set frame rate
    memset(&tParm, 0, sizeof tParm);
    tParm.type = pDev->eBufType;
    ioctl(pDev->iFd, VIDIOC_G_PARM, &tParm);
    printf("Current frame rate %u/%u\n", tParm.parm.capture.timeperframe.numerator, tParm.parm.capture.timeperframe.denominator);

    tParm.parm.capture.timeperframe.numerator = 1;
    tParm.parm.capture.timeperframe.denominator = CAM_FPS;
    printf("Config frame rate %u/%u\n", tParm.parm.capture.timeperframe.numerator, tParm.parm.capture.timeperframe.denominator);

    ioctl(pDev->iFd, VIDIOC_S_PARM, &tParm);

    ioctl(pDev->iFd, VIDIOC_G_PARM, &tParm);
    printf("Frame rate %u/%u\n", tParm.parm.capture.timeperframe.numerator, tParm.parm.capture.timeperframe.denominator);

    return 0;
}

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

static void* h26xenc_loop(void* data __attribute__((unused)))
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
    printf("[%s] Ratio:%d, WH:%dx%d, Stride:%d\r\n", __func__,
            codec_initopt.dwCompressionRatio,
            codec_initopt.dwSrcWidth, codec_initopt.dwSrcHeight,
            codec_initopt.dwSrcStride
            );

    if(0 != h26xe_init(&codec_initopt)){
        printf("h26xe_init failed\n");
    }

    h26xe_get_header_info(g_eCodec, g_ptStreamHdr);

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
        #if 0              

        int ret = -1;
        if (dwEncCount == 8) {
            
            /* Set ROI window */                          
            VMF_CODEC_ROI_WINDOW_T wInfo;
            wInfo.bEnable = 1;
            wInfo.dwIndex = 0;
            wInfo.dwStartX = 0;	
            wInfo.dwStartY = 0;
            wInfo.dwEndX = 128;
            wInfo.dwEndY = 128;
            wInfo.dwQp = 0;

            ret = VMF_H26xEnc_ROI_SetWindow(h26xe_handle, &wInfo);
            if(-1 == ret) {
                printf("[%s] H5E set ROI failed!!\n", __func__);				
                break;
            } 
            

            //! GOP
            VMF_CODEC_OPTION_T option;
            memset(&option, 0, sizeof(option));
            option.eOptionFlag = VMF_CODEC_H26XE_CHANGE_INTRA_INTERVAL;
            option.adwData[0] = 10;
            ret = VMF_H26xEnc_SetOptions(h26xe_handle, &option);
            if (ret)
            {
                printf("[%s] Change to GOP(%lu) failed, ret = %d\n", __func__, option.adwData[0], ret);			
                break;
            }

            //! ForceIntra
            memset(&option, 0, sizeof(option));
            option.eOptionFlag = VMF_CODEC_H26XE_FORCE_INTRA;                    

            ret = VMF_H26xEnc_SetOptions(h26xe_handle, &option);
            if (ret)
            {
                printf("[%s] ForceIntra failed, ret = %d\n", __func__ , ret);				
                break;
            }

            //! Set fps rate
            memset(&option, 0, sizeof(option));
            option.eOptionFlag = VMF_CODEC_H26XE_CHANGE_FRAME_RATE;  
            option.adwData[0] = 10;

            ret = VMF_H26xEnc_SetOptions(h26xe_handle, &option);
            if (ret)
            {
                printf("[%s] Set fps rate failed, ret = %d\n", __func__ , ret);             
                break;
            }    

            //! Set bit rate
            memset(&option, 0, sizeof(option));
            option.eOptionFlag = VMF_CODEC_H26XE_CHANGE_BITRATE;  
            option.adwData[0] = 1000000;

            ret = VMF_H26xEnc_SetOptions(h26xe_handle, &option);
            if (ret)
            {
                printf("[%s] Set bit rate failed, ret = %d\n", __func__ , ret);               
                break;
            }  
        }

        #endif
        if (dwEncCount % 100 == 0) {
            clock_gettime(CLOCK_MONOTONIC, &tCurrentTime);
            idiff = (tCurrentTime.tv_sec*1000000 + tCurrentTime.tv_nsec/1000) - 
                    (tVideoStartTime.tv_sec*1000000 + tVideoStartTime.tv_nsec/1000);
#if 1	//JDBG
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
    printf("[%s] quit successfully!\n", __func__);	//JDBG
    return NULL;
}

static void print_usage(const char *name)
{
    printf("Usage:\t %s -c test.conf\n" , name);
}

static void sig_kill(int signo)
{
    printf("[vmf_h26xe] receive SIGNAL: %d\n", signo);
    g_bTerminate = 1;
    pthread_cond_signal(&g_tVideoCond);
    if (g_ptSsmReaderHandle)
        SSM_Reader_Wakeup(g_ptSsmReaderHandle);

    
} //! sig_kill


static int loadConfig(char *configPath)
{
	if (!configPath) {
		printf("%s failed \n", __func__);
		return -1;
	}

	dictionary *ini;
	const char *tmp = NULL;
	ini = iniparser_load(configPath);
	if (ini == NULL) {
		printf("can't parse file: %s\n", configPath);
		return -1;
	}

	int ret = -1;

	do {
		if ((tmp = iniparser_getstring(ini, "main:ssm_reader_pin", 0)) == NULL) {
			printf("Need ssm pin to read\n");
			break;
		}
		if(tmp)
			g_szSsmReaderPin = strdup(tmp);

        if ((tmp = iniparser_getstring(ini, "main:srb_writer_pin", 0)) == NULL) {
            printf("Need srb pin to write\n");
            break;
        }
        if(tmp)
            g_szSrbWriterPin = strdup(tmp);

		g_eCodec = iniparser_getint(ini, "main:codec_type", 0);
		g_bWriteEnable = iniparser_getint(ini, "main:enanble_write", 0);
        g_dwTargetW =  iniparser_getint(ini, "main:Resize_Width", 0);
        g_dwTargetH =  iniparser_getint(ini, "main:Resize_Height", 0);
		g_szCameraPath = strdup(iniparser_getstring(ini, "main:CameraPath", "/dev/video0"));

		ret = 0;
	} while(0);
    
	iniparser_freedict(ini);
	return ret;
}

static void msg_callback(MsgContext* msg_context, void* user_data)
{
    (void) user_data;

    printf("[h26xEncStrm] Video msg_context->pszHost=%s, msg_context->pszCmd=%s\n", msg_context->pszHost, msg_context->pszCmd);

    if(!strcasecmp(msg_context->pszHost, "encoder2")){
        if(!strcasecmp(msg_context->pszCmd, "start")){
            pthread_cond_signal(&g_tVideoCond);
        }else if (!strcasecmp(msg_context->pszCmd, "stop")){
            g_bConfFlag = true;
        }else if (!strcasecmp(msg_context->pszCmd, "forceCI")){
            send_video_data(SEND_CONF);
        }else{
            // Do nothing
            }
    }
}
static int init_resize_hanle(unsigned int dwSrcW, unsigned int dwSrcH)
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

static void *v4l2_loop(void *data __attribute__((unused)))
{
    struct timespec tGetStartTime, tGetStopTime;
    unsigned int dwWorkingTime = 0;
    unsigned int dwPeriodTime = 1000000 / CAM_FPS;

    SSM_BUFFER_T tWriterSsmBuffer;
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
#ifdef V4L2_PIX_FMT	//JDBG
    ssm_opt.buf_size = (CAM_WIDTH * CAM_HEIGHT * 2) + VMF_MAX_SSM_HEADER_SIZE;
#else
    ssm_opt.buf_size = (CAM_WIDTH * CAM_HEIGHT * 3 / 2) + VMF_MAX_SSM_HEADER_SIZE;
#endif
    ssm_opt.alignment = VMF_ALIGN_TYPE_DEFAULT;
    ssm_opt.pUserData = &vsrc_ssm_writer_info;
    ssm_opt.fp_setup_buffer = ssm_clear_header;
    g_ptSsmWriterHandle = SSM_Writer_Init(&ssm_opt);
    if (!g_ptSsmWriterHandle)
    {
        printf("init g_ptSsmWriterHandle failed\n");
    }

    int ret = 0;
    struct Device tDev;
    struct v4l2_requestbuffers rb;
    struct MapBuf *pBufs = NULL;
    unsigned int i;
    struct MapBuf *pBuf = NULL;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    struct v4l2_buffer buf;

    //! open device
    memset(&tDev, 0, sizeof tDev);
    init_cam(&tDev, g_szCameraPath, CAM_WIDTH, CAM_HEIGHT);
    if (config_cam(&tDev))
    {
        printf("config cam failed\n");
        close_cam(&tDev);
    }

    //! allocate buffer
    memset(&rb, 0, sizeof rb);
    rb.count = BUF_NUMBER;
    rb.type = tDev.eBufType;
    rb.memory = tDev.eMemType;

    if (ioctl(tDev.iFd, VIDIOC_REQBUFS, &rb) < 0)
    {
        printf("[error] Request buffer failed\n");
    }
    pBufs = malloc(rb.count * sizeof pBufs[0]);
    if (pBufs == NULL)
    {
        printf("[error] Allocate buffer failed\n");
    }
    for (i = 0; i < rb.count; i++)
    {
        pBuf = pBufs + i;
        memset(&buf, 0, sizeof buf);
        memset(planes, 0, sizeof planes);
        buf.index = i;
        buf.type = tDev.eBufType;
        buf.memory = tDev.eMemType;
        buf.length = VIDEO_MAX_PLANES; //?
        buf.m.planes = planes;
        if (ioctl(tDev.iFd, VIDIOC_QUERYBUF, &buf))
        {
            printf("[error] Query buffer failed\n");
        }
        printf("[info] buf[%d] length %d offset %d\n", i, buf.length, buf.m.offset);

        //! map to user space
        pBuf->pbyMem[0] = mmap(0, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, tDev.iFd, buf.m.offset);
        if (pBuf->pbyMem[0] == MAP_FAILED)
        {
            printf("Map buffer failed\n");
        }
        pBuf->dwSize[0] = buf.length;
    }

    ioctl(tDev.iFd, VIDIOC_STREAMON, &tDev.eBufType);

    for (i = 0; i < BUF_NUMBER; i++)
    {
        video_queue_buffer(&tDev, i);
    }

    while (!g_bTerminate)
    {
        clock_gettime(CLOCK_MONOTONIC, &tGetStartTime);

        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);

        memset(&buf, 0, sizeof buf);
        memset(planes, 0, sizeof planes);
        buf.type = tDev.eBufType;
        buf.memory = tDev.eMemType;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;
        if (ioctl(tDev.iFd, VIDIOC_DQBUF, &buf) < 0)
        {
            printf("Dequeue buffer failed\n");
        }
        pBuf = pBufs + buf.index;

#if 0	//JDBG
        printf("[info] %s(): buffer(%02d) sz %d\r", __func__, buf.index, buf.bytesused);  fflush(stdout);
#endif

#ifdef V4L2_PIX_FMT
        YUV422To420P( pBuf->pbyMem[0], (char *)tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, CAM_HEIGHT, CAM_WIDTH );
#else
        memcpy(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, pBuf->pbyMem[0], buf.bytesused);
#endif
        // MemBroker_CacheCopyBack(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, buf.bytesused);
        MemBroker_CacheFlush(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, buf.bytesused);

#if 0
        //! save capture frame
        static unsigned int idx = 0;
        char file[32] = {0};
        snprintf(file, 32, "output/cam-%04d.yuv", idx++);
        FILE *pf = fopen(file, "wb");
        if (pf) {
            fwrite(pBuf->pbyMem[0], 1, buf.bytesused, pf);
            fclose(pf);
        }
#endif

        ret = video_queue_buffer(&tDev, buf.index);
        if (ret < 0)
            printf("Unable to queue buffer: %s (%d).\n", strerror(errno), errno);

        clock_gettime(CLOCK_MONOTONIC, &tGetStopTime);
        dwWorkingTime = tGetStopTime.tv_sec * 1000000 - tGetStartTime.tv_sec * 1000000 + (tGetStopTime.tv_nsec - tGetStartTime.tv_nsec) / 1000;
        if (dwWorkingTime < dwPeriodTime)
        {
            // printf("[info] working time %d dwPeriodTime %d\n", dwWorkingTime, dwPeriodTime);
            usleep(dwPeriodTime - dwWorkingTime);
        }
    }

    ioctl(tDev.iFd, VIDIOC_STREAMOFF, &tDev.eBufType);
    close_cam(&tDev);
    if (g_ptSsmWriterHandle)
    {
        SSM_Release(g_ptSsmWriterHandle);
    }

    printf("[%s] quit successfully!\n", __func__);	//JDBG
    return NULL;
}

int main(int argc, char** argv)
{
    int opt;
    pthread_t tid[5] = {0};
    pthread_t tsrc;
    memset(&codec_initopt, 0, sizeof(VMF_CODEC_INITOPT_T));

	while (-1 != (opt = getopt(argc, argv, "c:h:"))) {
		switch(opt)
		{
		case 'c':
        g_azH26xe_conf = strdup(optarg);
			break;
		case 'h':
		default:
			print_usage(argv[0]);
			exit(1);
		}
	}

    if (0 != loadConfig(g_azH26xe_conf)) {
        printf("Failed to parser conf!\n");
        exit(1);
    }
    printf("ssm reader pin: %s\n", g_szSsmReaderPin);
    printf("srb writer pin: %s\n", g_szSrbWriterPin);
    printf("codec type: %s\n", g_eCodec ? "H.265": "H.264");
    printf("write file enable: %s\n", g_bWriteEnable ? "Enabled": "Disabled");
    //! register signal
    signal(SIGTERM, sig_kill);
    signal(SIGINT, sig_kill);
    //VMF_SetDebugMessageLevel(VMF_DML_TRACE| VMF_DML_DEBUG | VMF_DML_ERROR);

    if (0 != pthread_create(&tsrc, NULL, v4l2_loop, NULL)) {
        printf("[vmf_h26xe] create thread failed. \n");
    }

    //! init SSM Reader Pin, YUV Source.
    g_ptSsmReaderHandle = SSM_Reader_Init(g_szSsmReaderPin);
    SSM_Reader_ReturnReceiveBuff(g_ptSsmReaderHandle, &g_tInSsmBuf);
    VMF_VSRC_SSM_GetInfo(g_tInSsmBuf.buffer, &g_tInSsmInfo);
    SSM_Reader_ReturnBuff(g_ptSsmReaderHandle, &g_tInSsmBuf);

    //! init SRB Writer Pin, Write Encoded data to srb ring. for RTSP
    g_ptVideoSrbHandle = SRB_InitWriter(g_szSrbWriterPin, 1048576, 4);
    if (!g_ptVideoSrbHandle){
        printf(" [%s,%d] Video srb writer handle init error \n", __func__, __LINE__);
        exit(1);
    }
    memset(&g_tVideoSrbWriterBuf, 0, sizeof(SRB_BUFFER_T));

    SRB_SendGetWriterBuff(g_ptVideoSrbHandle, &g_tVideoSrbWriterBuf);

    if (g_dwTargetW == 0 || g_dwTargetH == 0) {
        g_dwTargetW = g_tInSsmInfo.dwWidth;
        g_dwTargetH = g_tInSsmInfo.dwHeight;
    }
    if((g_tInSsmInfo.dwWidth != g_dwTargetW) && (g_tInSsmInfo.dwHeight != g_dwTargetH) &&
         init_resize_hanle(g_tInSsmInfo.dwWidth, g_tInSsmInfo.dwHeight)) {
		printf("[%s] Initial resize handle failed !!\n", __func__);
	}

    if (0 != pthread_create(&tid[0], NULL, h26xenc_loop, NULL)) {
        printf("[vmf_h26xe] create thread failed. \n");
    }
    MsgBroker_RegisterMsg(VENC_CMD_FIFO);
    MsgBroker_Run(VENC_CMD_FIFO, msg_callback, NULL, &g_bTerminate);
    MsgBroker_UnRegisterMsg();

    //! bye
    pthread_join(tsrc, NULL);
    if (0 != pthread_join(tid[0], NULL)) {
        printf("[vmf_h26xe] join thread failed.\n");
    }
    pthread_cond_destroy(&g_tVideoCond);
    pthread_mutex_destroy(&g_tVideoMutex);
    free(g_azH26xe_conf);
    free(g_szSsmReaderPin);
    free(g_szSrbWriterPin);
    if(g_ptVideoSrbHandle != NULL)
        SRB_Release(g_ptVideoSrbHandle);
    if(g_ptSsmReaderHandle != NULL)
        SSM_Release(g_ptSsmReaderHandle);
    if (g_ptResizeHandle)
        VMF_RS_Release(g_ptResizeHandle);
    printf("[vmf_h26xe] quit successfully!\n");
    return 0;
}
