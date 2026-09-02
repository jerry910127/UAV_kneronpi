#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

#include <opencv2/opencv.hpp>

extern "C" {
#include <mem_broker.h>
#include <msgbroker/msg_broker.h>
#include <video_source.h>
#include <video_bind.h>
#include <video_buf.h>
#include <sync_shared_memory.h>
#include <ssm_info.h>
#include <vmf_log.h>
#include <vector_dma.h>
#include <iniparser/iniparser.h>

#include "fec_api.h"
}

#include "example_shared_struct.h"
#include "kp_struct.h"

#if 1	//JDBG
#include "h26xenc_api.h"

#define CAM_WIDTH	640
#define CAM_HEIGHT	480

extern char *g_szSsmReaderPin;

extern SSM_WRITER_INIT_OPTION_T ssm_opt;
extern SSM_HANDLE_T* g_ptSsmWriterHandle;

void ssm_clear_header(unsigned char* virt_addr, unsigned int buf_size, void* pUserData);
#endif

#define VENC_VSRC_PIN       "vsrc_ssm"                  //! VMF_VSRC Output pin
#define VENC_VSRC_C_PIN     "vsrc_ssm_c_0"              //! VMF_VSRC Customer Output pin
#define VENC_VSRC_B_PIN     "vsrc_ssm_b_0"              //! VMF_VSRC Draw Box Customer Output pin
#define VENC_RESOURCE_DIR   "./Resource/"               //! directory contains ISP, AE, AWB, AutoScene sub directory
#define VENC_CMD_FIFO       "/tmp/venc/c0/command.fifo" //! communicate with rtsps, vrec, etc.
#define SRB_DEFAULT_PREFIX  "venc_srb"

extern FECDefValue gFecDefValue;

static VMF_BIND_CONTEXT_T* g_ptBind = NULL;

VMF_SRC_CONNECT_INFO_T connect_info;
VMF_LAYOUT_T g_tLayout;
VMF_VSRC_HANDLE_T* g_ptVsrcHandle = NULL;
ssm_handle_t *gptSsmHandle;


extern bool _blDispatchRunning;
extern bool _blFifoqManagerRunning;

extern bool _blSendInfRunning;
extern bool _blResultRunning;
extern bool _blDisplayRunning;

bool _blImageRunning = true;

NNM_SHARED_INPUT_T _input_data = {0};
pthread_mutex_t _mutex_image = PTHREAD_MUTEX_INITIALIZER;
//WEBCAM
NNM_SHARED_INPUT_T _input_data_webcam = {0};
pthread_mutex_t _mutex_image_webcam = PTHREAD_MUTEX_INITIALIZER;

/******************************* Local functions Implementation ************************************/
typedef struct
{
    VMF_DMA_HANDLE_T* ptDmaHandle; // 控制通道
    VMF_DMA_DESCRIPTOR_T* ptDmaDesc; // 描述子 (2D copy 參數)
} DMA_INFO_T;

DMA_INFO_T* dma2d_init()
{
    DMA_INFO_T* dma_info;
    //MemBroker 是 Kneron SDK 的記憶體池，確保分配到「連續實體位址」以供 DMA 使用。
    dma_info = (DMA_INFO_T*)MemBroker_GetMemory(sizeof(DMA_INFO_T), VMF_ALIGN_TYPE_DEFAULT);
    //檢查失敗：若沒拿到記憶體，列印錯誤並直接返回 NULL。
    if (NULL == dma_info) {
        printf("dma2d_init MemBroker_GetMemory failed\n");
        return NULL;
    }
    //初始化一條 2D DMA 通道，1：最多同時掛 1 個 DMA 任務（queue depth）。128：burst size / FIFO 深度（視 SDK 定義）    
    dma_info->ptDmaHandle = VMF_DMA_Init(1,128);
    if (NULL == dma_info->ptDmaHandle) {
        if(dma_info)
            MemBroker_FreeMemory(dma_info);
        printf("dma2d_init failed\n");
        return NULL;
    }
    //宣告並清零 2D Copy 描述子初始化結構 init。
    VMF_DMA_2DCF_INIT_T init;
    memset(&init, 0, sizeof(VMF_DMA_2DCF_INIT_T));

    init.dwProcessCbCr = 1;
    dma_info->ptDmaDesc = VMF_DMA_Descriptor_Create(DMA_2D, &init);
    if (NULL == dma_info->ptDmaDesc) {
        if(dma_info->ptDmaHandle)
            VMF_DMA_Release(dma_info->ptDmaHandle);
        if(dma_info)
            MemBroker_FreeMemory(dma_info);
        printf("dma_desc init failed\n");
    }
    return dma_info;
}

void dma2d_release(DMA_INFO_T* dma_info)
{
    VMF_DMA_Descriptor_Destroy(dma_info->ptDmaDesc);
    VMF_DMA_Release(dma_info->ptDmaHandle);
    MemBroker_FreeMemory(dma_info);
}

int dma2d_copy(VMF_DMA_HANDLE_T* dma_handle, VMF_DMA_DESCRIPTOR_T* dma_desc, void* dest, void* source, VMF_VSRC_SSM_OUTPUT_INFO_T* vsrc_ssm_info)
{
    int ret = 0;
    VMF_DMA_ADDR_T dma_addr;
    memset(&dma_addr, 0, sizeof(VMF_DMA_ADDR_T));
    //MemBroker_CacheCopyBack(source, vsrc_ssm_info->dwWidth * vsrc_ssm_info->dwHeight * 1.5);//Reading source data from ssm buffer doesn't need using CacheCopyBack.
    
    //指定 要搬運的影像寬、高（單位：像素）。
    dma_addr.dwCopyWidth = vsrc_ssm_info->dwWidth;
    dma_addr.dwCopyHeight = vsrc_ssm_info->dwHeight;
    //取得 來源 SSM buffer 的實體位址，並加入 VSRC 給的 Y / Cb / Cr 三平面 offset；
    dma_addr.pbySrcYPhysAddr = (unsigned char*)MemBroker_GetPhysAddr(source) + vsrc_ssm_info->dwOffset[0];
    dma_addr.pbySrcCbPhysAddr = (unsigned char*)MemBroker_GetPhysAddr(source) + vsrc_ssm_info->dwOffset[1];
    dma_addr.pbySrcCrPhysAddr = (unsigned char*)MemBroker_GetPhysAddr(source) + vsrc_ssm_info->dwOffset[2];
    dma_addr.dwSrcStride = vsrc_ssm_info->dwYStride;
    //→ 依 YUV420 佈局 計算目的緩衝區三平面位址：Y：起點，Cb：+ W × H，Cr：+ W × H / 4，dwDstStride 用畫面寬度（每行無額外 padding）。
    dma_addr.pbyDstYPhysAddr = (unsigned char*)MemBroker_GetPhysAddr(dest);
    dma_addr.pbyDstCbPhysAddr = dma_addr.pbyDstYPhysAddr + vsrc_ssm_info->dwWidth* vsrc_ssm_info->dwHeight;
    dma_addr.pbyDstCrPhysAddr = dma_addr.pbyDstCbPhysAddr + vsrc_ssm_info->dwWidth* vsrc_ssm_info->dwHeight/4;
    dma_addr.dwDstStride = vsrc_ssm_info->dwWidth;
    //在 DMA 寫入前 先清掉目的區域的 CPU cache，避免舊資料污染。（寫回/失效）
    MemBroker_CacheFlush(dest, vsrc_ssm_info->dwWidth * vsrc_ssm_info->dwHeight * 1.5);
    //把剛填好的位址/尺寸資訊 更新到描述子（僅改位址，不重建）。
    ret |= VMF_DMA_Descriptor_Update_Addr(dma_desc, &dma_addr);
    //把描述子陣列（長度 1）提交給 DMA 通道，排入待執行佇列。
    ret |= VMF_DMA_Setup(dma_handle, &dma_desc, 1);
    // 啟動 DMA；呼叫完成代表搬運結束（同步或輪詢完成）。
    ret |= VMF_DMA_Process(dma_handle);
    return ret;
}

/* ========================= sensor related ================================ */

static void release_video_source(VMF_VSRC_HANDLE_T* ptVsrcHandle)
{
    VMF_VSRC_Stop(ptVsrcHandle);
    VMF_VSRC_Release(ptVsrcHandle);
}

/* This function is called after video source is initialized and first video frame is captured in buffer */
//→ VSRC 在成功抓到第一張影像後回呼，把解析度 width × height 傳進來供後續排版 (layout) 使用。
//g_tLayout 初始化為「影像＝畫布、左上對齊」
static void vsrc_init_callback(unsigned int width, unsigned int height)
{
    memset(&g_tLayout, 0, sizeof(VMF_LAYOUT_T));
    g_tLayout.dwCanvasWidth = width;
    g_tLayout.dwCanvasHeight = height;
    g_tLayout.dwVideoPosX = 0;
    g_tLayout.dwVideoPosY = 0;
    g_tLayout.dwVideoWidth = width;
    g_tLayout.dwVideoHeight = height;
}

void set_eis(VMF_EIS_INIT_T* ptEisInit)
{
    // gFecDefValue 內含一組 FEC+EIS 預設值；取出其中的 tEis 區段作來源。
    EIS_T* ptEis = &(gFecDefValue.tEis);

    if (!ptEisInit) {
        printf("[%s] Err: eis init is null.\n", __func__);
        return;
    }
    //→ 鏡頭曲線節點檔 (Lens Curve) 路徑。
    ptEisInit->pszLensCurveNodesPath = ptEis->pszCurveNodesPath;
    //log 檔路徑設 NULL（此範例不啟用專屬 log）。
    ptEisInit->pszLogPath = NULL;
    // 陀螺儀資料放大倍率（校正用）。
    ptEisInit->fGyroDataGain = ptEis->fGyroDataGain;
    // EIS 變形網格切分數（基礎格點）。
    ptEisInit->dwGridSection = ptEis->dwGridSection;
    //確保 最大格點數 不小於基礎格點數。
    ptEisInit->dwMaxGridSection = (ptEis->dwMaxGridSection >= ptEis->dwGridSection)?ptEis->dwMaxGridSection:ptEis->dwGridSection;
    // → 裁切比例（EIS 為防抖預留邊界）。
    ptEisInit->fCropRatio = ptEis->fCropRatio;
    // → 影像格式類型 (YUV / RGB…) 及處理模式 (線性 / 滾動快門補償等)。
    ptEisInit->dwImageType = ptEis->dwImageType;
    ptEisInit->dwProcessMode = ptEis->dwProcessMode;
    // → 三組座標轉換係數，用於把陀螺儀座標系換算到影像座標系。
    ptEisInit->dwCoordinateTransform[0] = ptEis->dwCoordinateTransform[0];
    ptEisInit->dwCoordinateTransform[1] = ptEis->dwCoordinateTransform[1];
    ptEisInit->dwCoordinateTransform[2] = ptEis->dwCoordinateTransform[2];
    // → 時間戳校正（sensor vs IMU 時序差）——signed quad‐word。
    ptEisInit->sqwTimeOffset = ptEis->sqwTimeOffset;
    // → 影像是否預設旋轉 180° （部分鏡頭倒裝要開啟）。
    ptEisInit->bImageRotate180 = ptEis->bImageRotate180;
    // → Rolling Shutter 讀出時間差 & 比例，用於 RS 畫面校正。
    ptEisInit->sdwReadoutTimeOffset = ptEis->sdwReadoutTimeOffset;
    ptEisInit->fReadoutTimeRatio = ptEis->fReadoutTimeRatio;
    // → 是否強制使用原始 Rolling-Shutter 模式參數。
    ptEisInit->bForceOriRs = ptEis->bForceOriRs;
    // → 一次性複製 陀螺儀配置結構（I²C 位址、量測範圍、取樣率…）。
    memcpy(&ptEisInit->tGyroConfig, &ptEis->tGyroConfig, sizeof(VMF_VSRC_GYRO_CONFIG_T));
    // 呼叫者拿到填妥的 ptEisInit，即可傳給 VSRC 啟動 EIS。
}
// 建立並初始化 Video Source (VSRC)，把感測器、FEC、EIS、Fusion 等前端設定一次完成。成功回傳 0，失敗回 -1。
static int init_video_source(EXAMPLE_SENSOR_INIT_OPT_T* pExampleSensorInit)
{
    VMF_VSRC_INITOPT_T tVsrcInitOpt;
    VMF_VSRC_FRONTEND_CONFIG_T tVsrcFrontendConfig;

    memset(&tVsrcInitOpt, 0, sizeof(VMF_VSRC_INITOPT_T));
    memset(&tVsrcFrontendConfig, 0, sizeof(VMF_VSRC_FRONTEND_CONFIG_T));

    // 若 dwFecMode ≠ 0（代表使用魚眼校正）：1.先載入標定檔 (loadCalibrateConfig)2.再載 FEC 參數 (loadFECConfig)
    if (pExampleSensorInit->tSensorConf.dwFecMode != 0) {
        if (loadCalibrateConfig(&pExampleSensorInit->tSensorConf, &tVsrcFrontendConfig) == -1) {
            printf("[%s] Err: no calibrate config\n", __func__);
            return -1;
        }
        if (loadFECConfig(&pExampleSensorInit->tSensorConf, &tVsrcFrontendConfig) == -1) {
            printf("[%s] Err: no fec config\n", __func__);
            return -1;
        }
    } else {
        if (loadFECConfig(&pExampleSensorInit->tSensorConf, &tVsrcFrontendConfig) == -1) {
            printf("[%s] Err: no fec config\n", __func__);
            return -1;
        }
    }
    // 指定魚眼校正方法為 GTR（Kneron SDK 一種 GPU/Tensor 驅動之快速重映射算法）。
    tVsrcFrontendConfig.tFecInitConfig.eFecMethod = VMF_FEC_METHOD_GTR;
    
    //! Fusion or Normal mode
    //  如果 INI 裡有給 fusion_cfg（多感測器拼接設定）→ 走 Fusion 模式：
    if (pExampleSensorInit->tSensorConf.pszFusionConfigPath) {
        tVsrcFrontendConfig.dwSensorConfigCount = 2;
        tVsrcFrontendConfig.apszSensorConfig[0] = pExampleSensorInit->tSensorConf.pszSensorConfigPath;
        tVsrcFrontendConfig.apszSensorConfig[1] = pExampleSensorInit->tSensorConf.pszFusionConfigPath;
        tVsrcInitOpt.eAppMode = VMF_VSRC_APP_MODE_FUSION;
        printf("[%s] Fusion Mode\n", __func__);
    } else {
        tVsrcFrontendConfig.dwSensorConfigCount = 1;
        tVsrcFrontendConfig.apszSensorConfig[0] = pExampleSensorInit->tSensorConf.pszSensorConfigPath;
        tVsrcInitOpt.eAppMode = VMF_VSRC_APP_MODE_NORMAL;
        printf("[%s] Normal Mode\n", __func__);
    }

    // 此 VSRC 只有「一條前端 (Front-End) pipeline」——對應我們剛填好的 tVsrcFrontendConfig。
    tVsrcInitOpt.dwFrontConfigCount = 1;
    // 把那條前端的詳細設定（感測器 cfg / FEC / EIS 等）丟給 VSRC。
    tVsrcInitOpt.ptFrontConfig = &tVsrcFrontendConfig;
    // 定 AutoScene（AE/AWB 自動場景）INI 檔路徑，VSRC 啟動時會載入。
    tVsrcInitOpt.pszAutoSceneConfig = pExampleSensorInit->tSensorConf.pszAutoSceneConfigPath;
    // 規定 VSRC 輸出的 Pin 都用這個前綴：VENC_VSRC_PIN
    tVsrcInitOpt.pszOutPinPrefix = VENC_VSRC_PIN;
    // 影像第一幀出來時呼叫 
    tVsrcInitOpt.fnInitCallback = vsrc_init_callback;
    // "./Resource/"
    tVsrcInitOpt.pszResourceDir = VENC_RESOURCE_DIR;
    tVsrcInitOpt.tEngConfig.dwIfpOutDataType = 1;
    // 定義 IFP 與 ISP 的輸出資料格式——1 代表 YUV420，
    tVsrcInitOpt.tEngConfig.dwIspOutDataType = 1;

    //! Fill EIS configuration
    // 開了 eis_enable=1 → 配置一塊 VMF_EIS_INIT_T;
    // 用 set_eis() 把全域 default 值複製進去；
    // calloc 失敗就關閉 EIS 旗標。
    if (pExampleSensorInit->dwEisEnable == 1) {
        tVsrcFrontendConfig.tFecInitConfig.ptEisInit = (VMF_EIS_INIT_T *)calloc(1, sizeof(VMF_EIS_INIT_T));
        if (!tVsrcFrontendConfig.tFecInitConfig.ptEisInit) {
            printf("[%s] calloc VMF_EIS_INIT_T fail.\n", __func__);
            pExampleSensorInit->dwEisEnable = 0;
        } else {
            set_eis(tVsrcFrontendConfig.tFecInitConfig.ptEisInit);
        }
    }

    g_ptVsrcHandle = VMF_VSRC_Init(&tVsrcInitOpt);
    if (!g_ptVsrcHandle) {
        printf("[%s] VMF_VSRC_Init failed!\n", __func__);
        return -1;
    }

    if (VMF_VSRC_Start(g_ptVsrcHandle, NULL) != 0) {
        printf("[%s] VMF_VSRC_Start failed!\n", __func__);
        release_video_source(g_ptVsrcHandle);
        return -1;
    }

    return setup_fec_mode(g_ptVsrcHandle, &g_tLayout, (FEC_MODE)pExampleSensorInit->tSensorConf.dwFecMode, pExampleSensorInit->dwEisEnable);
}
// → 函式：利用 VSRC 的 handle (hVideoSource) 建立一個 Binder，
// 　　讓其他模組（SSM、ISP 動態設定…）能查詢 VSRC 資訊或修改 ISP 參數。
static VMF_BIND_CONTEXT_T* init_bind(void* hVideoSource)
{
    //! init video bind
    VMF_BIND_INITOPT_T tBindOpt;
    memset(&tBindOpt, 0, sizeof(VMF_BIND_INITOPT_T));
    tBindOpt.dwSrcOutputIndex = 0;
    tBindOpt.ptSrcHandle = hVideoSource;
    tBindOpt.pfnQueryFunc = (VMF_BIND_QUERY_FUNC) VMF_VSRC_GetInfo;
    tBindOpt.pfnIspFunc = (VMF_BIND_CONFIG_ISP_FUNC) VMF_VSRC_ConfigISP;
    // 全域變數 g_ptBind 會接住
    return VMF_BIND_Init(&tBindOpt);
}

static void release_bind(VMF_BIND_CONTEXT_T* pBind)
{
    VMF_BIND_Release(pBind);
}

void *example_sensor_image_thread(void *arg)
{
    EXAMPLE_SENSOR_INIT_OPT_T *pExampleSensorInit = (EXAMPLE_SENSOR_INIT_OPT_T*)arg;

    //Video source initialization
    unsigned int dwInferenceWidth = pExampleSensorInit->dwImageWidth;
    unsigned int dwInferenceHeight = pExampleSensorInit->dwImageHeight;
    ssm_handle_t *ptSsmHandle = NULL;
    ssm_buffer_t ssm_buf;
    const unsigned int getyuv_retry_cnt = 500;
    unsigned int wait_cnt = 0;
    char azReaderSsmName[64];
    char tmpName[30] = {0};

#ifdef H26XE_API_H
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

    // ➜ 初始化 2D DMA：<br>
    VMF_SSM_READER_SCHEME eImageBufMode = (VMF_SSM_READER_SCHEME)pExampleSensorInit->dwGetImageBufMode;
    DMA_INFO_T *pDmaInfo = dma2d_init();

    if (NULL == pDmaInfo) {
        printf("init dma failed\n");
        goto EXIT_SENSOR_IMAGE_THREAD;
    }

    // ➜ 初始化 VSRC（感測器＋ISP＋FEC/EIS／Fusion）
    if (init_video_source(pExampleSensorInit)) {
        printf("init_video_source failed!!\n");
        goto EXIT_SENSOR_IMAGE_THREAD;
    }

    /* initializing the binder associated with the video source */
    // ➜ 建立 Binder，讓後續能查 VSRC 資訊或動態調 ISP
    if ((g_ptBind = init_bind(g_ptVsrcHandle)) == NULL) {
        printf("init_bind failed!!\n");
        release_video_source(g_ptVsrcHandle);
        goto EXIT_SENSOR_IMAGE_THREAD;
    }

    // ➜ 透過 Binder 請求：產生一條解析度為 dwInferenceWidth×dwInferenceHeight 的 SSM 輸出流，參數會寫入 connect_info。
    VMF_BIND_Request(g_ptBind, dwInferenceWidth, dwInferenceHeight, dwInferenceWidth, 0, &connect_info);

    // ➜ 用 connect_info.szSrcPin （Pin 名稱）初始化一個 SSM Reader，從 VSRC 的 SSM buffer 取流。
    gptSsmHandle = ptSsmHandle = SSM_Reader_Init(connect_info.szSrcPin);

    // ➜ 向 MemBroker 申請一塊 YUV420 大小的連續記憶體，作為 DMA 的目標緩衝區。
    _input_data.input_buf_address = (uintptr_t)MemBroker_GetMemory(connect_info.dwSrcWidth * connect_info.dwSrcWidth * 1.5, VMF_ALIGN_TYPE_128_BYTE);

    if (!ptSsmHandle) {
        printf("%s() failed, SSM_Reader_Init failed!\n", __func__);
    }

    //! wait for video source being ready.
    // ➜ 等候第一張影像 ready：<br>
    memset(&ssm_buf, 0, sizeof(ssm_buffer_t));
    while (SSM_Reader_ReturnReceiveNewestBuff(ptSsmHandle, &ssm_buf, eImageBufMode) < 0) {
        if ((getyuv_retry_cnt < wait_cnt++) || (false == _blImageRunning)) {
            printf("%s err: get video yuv failed, exit\n", __func__);
            goto EXIT_SENSOR_IMAGE_THREAD;
        }
    }

    // run infinitely
    while (true == _blImageRunning) {
#ifdef H26XE_API_H
        SSM_Writer_SendGetBuff(g_ptSsmWriterHandle, &tWriterSsmBuffer);
#endif
        SSM_Reader_ReturnReceiveNewestBuff(ptSsmHandle, &ssm_buf, eImageBufMode);//VMF_SSM_READER_BLOCK / VMF_SSM_READER_NONBLOCK
        VMF_VSRC_SSM_OUTPUT_INFO_T vsrc_ssm_info;
        VMF_VSRC_SSM_GetInfo(ssm_buf.buffer, &vsrc_ssm_info);
        if (!ssm_buf.buffer) {
            printf("[%s] ssm_buf.buffer = %p, EXIT_SENSOR_IMAGE_THREAD \n", __func__, ssm_buf.buffer);
            goto EXIT_SENSOR_IMAGE_THREAD;
        }

        pthread_mutex_lock(&_mutex_image);

        // ➜ 用 DMA 把畫面從 SSM buffer 快速搬到 _input_data.input_buf_address。
        dma2d_copy(pDmaInfo->ptDmaHandle, pDmaInfo->ptDmaDesc, (void *)_input_data.input_buf_address, ssm_buf.buffer, &vsrc_ssm_info);

        _input_data.input_image_width = vsrc_ssm_info.dwWidth;
        _input_data.input_image_height = vsrc_ssm_info.dwHeight;
        _input_data.input_image_format = KP_IMAGE_FORMAT_YUV420;

        _input_data.input_buf_size = _input_data.input_image_width * _input_data.input_image_height * 1.5;
        // 標記 input_ready_inf = true，告訴推論執行緒「新畫面已就緒」
        _input_data.input_ready_inf = true;
#ifdef H26XE_API_H
        memcpy(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, (unsigned char *)_input_data.input_buf_address, _input_data.input_buf_size);
        MemBroker_CacheFlush(tWriterSsmBuffer.buffer + VMF_MAX_SSM_HEADER_SIZE, _input_data.input_buf_size);
#endif

        pthread_mutex_unlock(&_mutex_image);
        
    }

EXIT_SENSOR_IMAGE_THREAD:

    if (g_ptVsrcHandle)
        release_video_source(g_ptVsrcHandle);

    if (ptSsmHandle) {
        if (ssm_buf.buffer) SSM_Reader_ReturnBuff(ptSsmHandle, &ssm_buf);
        SSM_Release(ptSsmHandle);
    }
    gptSsmHandle = NULL;

    if (g_ptBind)
        release_bind(g_ptBind);

    if (0 != _input_data.input_buf_address)
        MemBroker_FreeMemory((void *)_input_data.input_buf_address);

    if (NULL != pDmaInfo)
        dma2d_release(pDmaInfo);

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

void *example_webcam_input_thread(void *arg)
{
    EXAMPLE_WEBCAM_INIT_OPT_T* pInitOpt=(EXAMPLE_WEBCAM_INIT_OPT_T*)arg;

    // 宣告 OpenCV 物件：
    // v_camera_cap：攝影機擷取介面。
    // cv_read_camera：存放原始 BGR 影格。
    // cv_img_to_be_sent：存放轉換後用於推論的 RGBA 影格。
    cv::VideoCapture cv_camera_cap;
    cv::Mat cv_read_camera, cv_img_to_be_sent;

    //嘗試以指定裝置開啟攝影機。若失敗，輸出錯誤訊息並跳到清理標籤結束執行緒 
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
        //從攝影機擷取一幀影像至 cv_read_camera（BGR 格式）。
        cv_camera_cap.read(cv_read_camera);

        pthread_mutex_lock(&_mutex_image_webcam);
        //  將 BGR 影像轉換為 RGBA 格式，符合 KP 推論函式庫的資料需求。
        cv::cvtColor(cv_read_camera, cv_img_to_be_sent, cv::COLOR_BGR2RGBA);

        // 將 RGBA 影像資料的記憶體位址存入共享結構，供推論介面直接讀取。
        _input_data_webcam.input_buf_address = (uintptr_t)cv_img_to_be_sent.data;
        _input_data_webcam.input_image_width = cv_img_to_be_sent.cols;
        _input_data_webcam.input_image_height = cv_img_to_be_sent.rows;
        _input_data_webcam.input_image_format = KP_IMAGE_FORMAT_RGBA8888;

        _input_data_webcam.input_buf_size = _input_data_webcam.input_image_width * _input_data_webcam.input_image_height * 4;
        _input_data_webcam.input_ready_inf = true;
        pthread_mutex_unlock(&_mutex_image_webcam);
    }

EXIT_FREAD_IMAGE_THREAD:

    _blSendInfRunning = false;
    _blResultRunning = false;
    _blDisplayRunning = false;

    _blDispatchRunning = false;
    _blFifoqManagerRunning = false;
    printf("[%s] bye!\r\n", __func__);
    return NULL;
}
