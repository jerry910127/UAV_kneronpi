/*
 *******************************************************************************
 *  Copyright (c) 2010-2022 VATICS(KNERON) Inc. All rights reserved.
 *
 *  +-----------------------------------------------------------------+
 *  | THIS SOFTWARE IS FURNISHED UNDER A LICENSE AND MAY ONLY BE USED |
 *  | AND COPIED IN ACCORDANCE WITH THE TERMS AND CONDITIONS OF SUCH  |
 *  | A LICENSE AND WITH THE INCLUSION OF THE THIS COPY RIGHT NOTICE. |
 *  | THIS SOFTWARE OR ANY OTHER COGCY OF THIS SOFTWARE MAY NOT BE    |
 *  | PROVIDED OR OTHERWISE MADE AVAILABLE TO ANY OTHER PERSON. THE   |
 *  | OWNERSHIP AND TITLE OF THIS SOFTWARE IS NOT TRANSFERRED.        |
 *  |                                                                 |
 *  | THE INFORMATION IN THIS SOFTWARE IS SUBJECT TO CHANGE WITHOUT   |
 *  | ANY PRIOR NOTICE AND SHOULD NOT BE CONSTRUED AS A COMMITMENT BY |
 *  | VATICS(KNERON) INC.                                             |
 *  +-----------------------------------------------------------------+
 *
 * *******************************************************************************
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include <srt/srt.h>
#include "mem_broker.h"
#include "mem_util.h"
#include "msg_broker.h"
#include "sync_shared_memory.h"
#include "video_encoder_output_srb.h"
#include "video_encoder_output_scm.h"
#include "video_source.h"
#include "video_encoder.h"
#include "video_decoder.h"
#include "video_h26xenc.h"
#include "video_bind.h"
#include "frame_info.h"
#include "vmf_log.h"

#define MODULE_NAME             "venc1"
#define STREAM_FEEDING_SIZE     (1*1024*1024)
#define FEEDING_SIZE            (256*1024)
#define ptEncBuff_SIZE          (1024*1024*8)

/* Adaptive Bitrate (ABR) System Parameters (Strict < 200 kbps Physical Limit) */
#define ABR_MIN_BITRATE         40000   // 40 kbps min video bitrate floor
#define ABR_INIT_BITRATE        100000  // 100 kbps initial video bitrate
#define ABR_MAX_BITRATE         130000  // 130 kbps max video bitrate ceiling (130k video + 45k overhead = 175k < 200k)
#define ABR_OVERHEAD_EST        45000   // 45 kbps estimated physical overhead (SRT/UDP/IP headers + AES + AI Telemetry)

static int g_bTerminate = 0;
static char* g_szInputPath = "/tmp/uav_test_720p.h264";
static char* g_szSrtIp = "192.168.168.17";
static int g_dwSrtPort = 9000;
static unsigned int g_dwWidth = 1280;
static unsigned int g_dwHeight = 720;
static unsigned int g_dwFps = 30;
static unsigned int g_dwBitrate = ABR_INIT_BITRATE; // Initialized to 140kbps for ABR
static unsigned int g_dwGop = 10;
static int g_bLiveMode = 0; // 0: File loop mode, 1: Live camera FIFO stream mode

SRTSOCKET g_srt_sock = SRT_INVALID_SOCK;
static struct timeval g_last_reconnect_time = {0, 0};

static void sig_handler(int signo)
{
    fprintf(stderr, "[%s] receive SIGNAL: %d\n", MODULE_NAME, signo);
    g_bTerminate = 1;
}

static void init_srt(void)
{
    srt_startup();
    g_srt_sock = srt_create_socket();
    if (g_srt_sock == SRT_INVALID_SOCK) {
        fprintf(stderr, "[SRT] Failed to create socket: %s\n", srt_getlasterror_str());
        return;
    }
    int yes = 1;
    int latency = 120;
    int message_api = 0;
    int transtype = 1; // SRTT_FILE (Matches Ground Station listener perfectly)
    int conn_timeout = 1000; // 1000 ms
    int snd_timeout = 300; // 300 ms send timeout (allows I-frames to transmit cleanly without drop)
    int snd_buf = 2000000; // 2MB sender buffer
    srt_setsockopt(g_srt_sock, 0, SRTO_SENDER, &yes, sizeof yes);
    srt_setsockopt(g_srt_sock, 0, SRTO_TSBPDMODE, &yes, sizeof yes);
    srt_setsockopt(g_srt_sock, 0, SRTO_LATENCY, &latency, sizeof latency);
    srt_setsockopt(g_srt_sock, 0, SRTO_MESSAGEAPI, &message_api, sizeof message_api);
    srt_setsockopt(g_srt_sock, 0, SRTO_TRANSTYPE, &transtype, sizeof transtype);
    srt_setsockopt(g_srt_sock, 0, SRTO_CONNTIMEO, &conn_timeout, sizeof conn_timeout);
    srt_setsockopt(g_srt_sock, 0, SRTO_SNDTIMEO, &snd_timeout, sizeof snd_timeout);
    srt_setsockopt(g_srt_sock, 0, SRTO_SNDBUF, &snd_buf, sizeof snd_buf);
    
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    sa.sin_port = htons(g_dwSrtPort);
    sa.sin_addr.s_addr = inet_addr(g_szSrtIp);
    
    fprintf(stderr, "[SRT] Connecting to %s:%d (caller mode)...\n", g_szSrtIp, g_dwSrtPort);
    gettimeofday(&g_last_reconnect_time, NULL);
    if (srt_connect(g_srt_sock, (struct sockaddr*)&sa, sizeof sa) == SRT_ERROR) {
        fprintf(stderr, "[SRT] Connection failed: %s.\n", srt_getlasterror_str());
        srt_close(g_srt_sock);
        g_srt_sock = SRT_INVALID_SOCK;
    } else {
        fprintf(stderr, "[SRT] Connected to %s:%d successfully!\n", g_szSrtIp, g_dwSrtPort);
    }
}

static void send_srt_headers(VMF_H26XENC_HANDLE_T* h26xe_handle)
{
    if (g_srt_sock == SRT_INVALID_SOCK) return;
    
    VMF_CODEC_NAL_PARAM_SETS_T tSpsPps;
    memset(&tSpsPps, 0, sizeof(VMF_CODEC_NAL_PARAM_SETS_T));
    
    unsigned char* sps = (unsigned char *)MemBroker_GetMemory(128, VMF_ALIGN_TYPE_DEFAULT);
    unsigned char* pps = (unsigned char *)MemBroker_GetMemory(128, VMF_ALIGN_TYPE_DEFAULT);
    unsigned char* vps = (unsigned char *)MemBroker_GetMemory(128, VMF_ALIGN_TYPE_DEFAULT);
    
    tSpsPps.pbySps = sps;
    tSpsPps.pbyPps = pps;
    tSpsPps.pbyVps = vps;
    tSpsPps.dwSpsSize = tSpsPps.dwPpsSize = tSpsPps.dwVpsSize = 128;
    
    VMF_CODEC_OPTION_T tOpt;
    memset(&tOpt, 0, sizeof(VMF_CODEC_OPTION_T));
    tOpt.eOptionFlag = VMF_CODEC_H26XE_GET_HEADER_INFO;
    tOpt.adwData[0]  = (unsigned long)&tSpsPps;
    
    if (0 == VMF_H26xEnc_SetOptions(h26xe_handle, &tOpt)) {
        fprintf(stderr, "[SRT] Sending VPS (%d bytes), SPS (%d bytes), PPS (%d bytes)...\n", 
               tSpsPps.dwVpsSize, tSpsPps.dwSpsSize, tSpsPps.dwPpsSize);
        if (tSpsPps.dwVpsSize > 0) {
            srt_send(g_srt_sock, (const char*)vps, tSpsPps.dwVpsSize);
        }
        if (tSpsPps.dwSpsSize > 0) {
            srt_send(g_srt_sock, (const char*)sps, tSpsPps.dwSpsSize);
        }
        if (tSpsPps.dwPpsSize > 0) {
            srt_send(g_srt_sock, (const char*)pps, tSpsPps.dwPpsSize);
        }
    } else {
        fprintf(stderr, "[SRT] Failed to get H.265 SPS/PPS/VPS headers!\n");
    }
    
    MemBroker_FreeMemory(sps);
    MemBroker_FreeMemory(pps);
    MemBroker_FreeMemory(vps);
}

static void send_srt_data(const void* data, int size, VMF_H26XENC_HANDLE_T* h26xe_handle)
{
    if (g_srt_sock == SRT_INVALID_SOCK) {
        struct timeval now;
        gettimeofday(&now, NULL);
        long long elapsed_ms = (now.tv_sec - g_last_reconnect_time.tv_sec) * 1000LL + (now.tv_usec - g_last_reconnect_time.tv_usec) / 1000;
        if (elapsed_ms < 5000) {
            // Cool down reconnects to avoid blocking video loop
            return;
        }
        g_last_reconnect_time = now;

        g_srt_sock = srt_create_socket();
        if (g_srt_sock != SRT_INVALID_SOCK) {
            int yes = 1;
            int latency = 120;
            int message_api = 0;
            int transtype = 1;
            int conn_timeout = 1000; // 1000 ms
            int snd_timeout = 300; // 300 ms send timeout
            int snd_buf = 2000000; // 2MB sender buffer
            srt_setsockopt(g_srt_sock, 0, SRTO_SENDER, &yes, sizeof yes);
            srt_setsockopt(g_srt_sock, 0, SRTO_TSBPDMODE, &yes, sizeof yes);
            srt_setsockopt(g_srt_sock, 0, SRTO_LATENCY, &latency, sizeof latency);
            srt_setsockopt(g_srt_sock, 0, SRTO_MESSAGEAPI, &message_api, sizeof message_api);
            srt_setsockopt(g_srt_sock, 0, SRTO_TRANSTYPE, &transtype, sizeof transtype);
            srt_setsockopt(g_srt_sock, 0, SRTO_CONNTIMEO, &conn_timeout, sizeof conn_timeout);
            srt_setsockopt(g_srt_sock, 0, SRTO_SNDTIMEO, &snd_timeout, sizeof snd_timeout);
            srt_setsockopt(g_srt_sock, 0, SRTO_SNDBUF, &snd_buf, sizeof snd_buf);
            
            struct sockaddr_in sa;
            memset(&sa, 0, sizeof sa);
            sa.sin_family = AF_INET;
            sa.sin_port = htons(g_dwSrtPort);
            sa.sin_addr.s_addr = inet_addr(g_szSrtIp);
            
            fprintf(stderr, "[SRT] Reconnecting to %s:%d...\n", g_szSrtIp, g_dwSrtPort);
            if (srt_connect(g_srt_sock, (struct sockaddr*)&sa, sizeof sa) == SRT_ERROR) {
                fprintf(stderr, "[SRT] Reconnect failed: %s. Will retry.\n", srt_getlasterror_str());
                srt_close(g_srt_sock);
                g_srt_sock = SRT_INVALID_SOCK;
                return;
            }
            fprintf(stderr, "[SRT] Reconnected to %s:%d successfully!\n", g_szSrtIp, g_dwSrtPort);
            send_srt_headers(h26xe_handle);
        } else {
            return;
        }
    }
    
    int ret = srt_send(g_srt_sock, (const char*)data, size);
    if (ret == SRT_ERROR) {
        int err = srt_getlasterror(NULL);
        if (err != 5002 && err != SRT_ETIMEOUT) { // Ignore would-block (SRT_EASYNCSND=5002) and send timeout (SRT_ETIMEOUT)
            fprintf(stderr, "[SRT] Send failed: %s (error %d)\n", srt_getlasterror_str(), err);
            if (err == SRT_ECONNLOST || err == SRT_ENOCONN) {
                srt_close(g_srt_sock);
                g_srt_sock = SRT_INVALID_SOCK;
            }
        }
    }
}

static void process_abr_control(VMF_H26XENC_HANDLE_T* h26xe_handle, unsigned int frame_cnt)
{
    if (g_srt_sock == SRT_INVALID_SOCK || srt_getsockstate(g_srt_sock) != SRTS_CONNECTED) return;
    if (frame_cnt % 5 != 0) return; // Evaluate ABR every 5 frames (~0.5s at 10 FPS)

    SRT_TRACEBSTATS perf;
    if (srt_bstats(g_srt_sock, &perf, 0) == SRT_ERROR) {
        return;
    }

    int snd_buf_bytes = perf.byteSndBuf;
    int rtt_ms = (int)perf.msRTT;
    int snd_drop = (int)perf.pktSndDrop;

    unsigned int current_bitrate = g_dwBitrate;
    unsigned int new_bitrate = current_bitrate;

    unsigned int est_physical_bps = current_bitrate + ABR_OVERHEAD_EST;

    /* Adaptive Bitrate Control Logic */
    if (snd_buf_bytes > 12000 || rtt_ms > 200 || snd_drop > 0) {
        // High Congestion: Aggressive reduction (75% of current)
        new_bitrate = (unsigned int)(current_bitrate * 0.75);
        if (new_bitrate < ABR_MIN_BITRATE) new_bitrate = ABR_MIN_BITRATE;
    } else if (snd_buf_bytes > 4000 || rtt_ms > 120) {
        // Moderate Congestion: Mild reduction (90% of current)
        new_bitrate = (unsigned int)(current_bitrate * 0.90);
        if (new_bitrate < ABR_MIN_BITRATE) new_bitrate = ABR_MIN_BITRATE;
    } else if (snd_buf_bytes < 2000 && rtt_ms < 80) {
        // Clean Network: Step up bitrate to maximize quality towards 175kbps (200kbps physical target)
        if (est_physical_bps < 175000) {
            new_bitrate = current_bitrate + 8000;
            if (new_bitrate > ABR_MAX_BITRATE) new_bitrate = ABR_MAX_BITRATE;
        }
    }

    if (new_bitrate != current_bitrate) {
        VMF_CODEC_OPTION_T option;
        memset(&option, 0, sizeof(option));
        option.eOptionFlag = VMF_CODEC_H26XE_CHANGE_BITRATE;
        option.adwData[0] = new_bitrate;

        int ret = VMF_H26xEnc_SetOptions(h26xe_handle, &option);
        if (ret == 0) {
            g_dwBitrate = new_bitrate;
            unsigned int est_phys = g_dwBitrate + ABR_OVERHEAD_EST;
            fprintf(stderr, "[ABR] RTT: %dms, SndBuf: %dB | Bitrate Adj: %u -> %u bps (Est Total Phys: ~%u bps / %.1f kbps)\n",
                    rtt_ms, snd_buf_bytes, current_bitrate, new_bitrate, est_phys, est_phys / 1000.0f);
        } else {
            fprintf(stderr, "[ABR] VMF_H26xEnc_SetOptions CHANGE_BITRATE failed: %d\n", ret);
        }
    } else if (frame_cnt % 30 == 0) {
        unsigned int est_phys = g_dwBitrate + ABR_OVERHEAD_EST;
        fprintf(stderr, "[ABR Status] Bitrate: %u bps | RTT: %dms | SndBuf: %dB | Est Total Phys: ~%.1f kbps (Target: 200kbps Limit)\n",
                g_dwBitrate, rtt_ms, snd_buf_bytes, est_phys / 1000.0f);
    }
}


static void print_usage(const char *name)
{
    fprintf(stderr, "Usage: %s [options]\n"
                    "Options:\n"
                    "  -i <path>       Input H.264 file (default: /tmp/uav_test_1080p.h264)\n"
                    "  -o <ip>         Target SRT receiver IP (default: 192.168.168.17)\n"
                    "  -p <port>       Target SRT receiver Port (default: 9000)\n"
                    "  -w <width>      Video width (default: 1920)\n"
                    "  -h <height>     Video height (default: 1080)\n"
                    "  -f <fps>        Frame rate (default: 10)\n"
                    "  -b <bitrate>    H.265 Bitrate in bps (default: 100000)\n"
                    "  -g <gop>        GOP size (default: 10)\n"
                    "  -l <0|1>        Live mode (0: File loop, 1: Live FIFO/pipe stream)\n"
                    "  -H              Show help\n", name);
}

int main(int argc, char* argv[])
{
    int ch;
    FILE* pfInput = NULL;
    VMF_VDEC_HANDLE_T* ptH26xDecoder = NULL;
    VMF_H26XDEC_STATE_T* ptH26xState = NULL;
    VMF_H26XENC_HANDLE_T* h26xe_handle = NULL;
    VMF_H26XENC_STATE_T* h26xe_state = NULL;
    unsigned char* pbyInBuf = NULL;
    unsigned char* ptEncBuff = NULL;
    
    VMF_VENC_INPUT_INFO_T input_info;
    VMF_VENC_OUTPUT_INFO_T output_info;
    
    /* register signal handler */
    signal(SIGTERM, sig_handler);
    signal(SIGINT, sig_handler);

    while ((ch = getopt(argc, argv, "i:o:p:w:h:f:b:g:l:H")) != -1) {
        switch(ch) {
        case 'i':
            g_szInputPath = strdup(optarg);
            break;
        case 'o':
            g_szSrtIp = strdup(optarg);
            break;
        case 'p':
            g_dwSrtPort = atoi(optarg);
            break;
        case 'w':
            g_dwWidth = atoi(optarg);
            break;
        case 'h':
            g_dwHeight = atoi(optarg);
            break;
        case 'f':
            g_dwFps = atoi(optarg);
            if (g_dwFps == 0) g_dwFps = 30;
            break;
        case 'b':
            g_dwBitrate = atoi(optarg);
            if (g_dwBitrate > ABR_MAX_BITRATE) {
                fprintf(stderr, "[venc1][ABR_LIMIT] Bitrate %u bps exceeds ABR ceiling %u bps. Clamped to %u bps.\n", 
                        g_dwBitrate, ABR_MAX_BITRATE, ABR_MAX_BITRATE);
                g_dwBitrate = ABR_MAX_BITRATE;
            }
            break;
        case 'g':
            g_dwGop = atoi(optarg);
            break;
        case 'l':
            g_bLiveMode = atoi(optarg);
            break;
        case 'H':
        default:
            print_usage(argv[0]);
            return 0;
        }
    }

    fprintf(stderr, "[%s] Input Path: %s\n", MODULE_NAME, g_szInputPath);
    fprintf(stderr, "[%s] Target SRT: %s:%d\n", MODULE_NAME, g_szSrtIp, g_dwSrtPort);
    fprintf(stderr, "[%s] Resolution: %dx%d @ %dfps\n", MODULE_NAME, g_dwWidth, g_dwHeight, g_dwFps);
    fprintf(stderr, "[%s] Bitrate: %d bps, GOP: %d\n", MODULE_NAME, g_dwBitrate, g_dwGop);

    /* Open input bitstream file */
    pfInput = fopen(g_szInputPath, "rb");
    if (!pfInput) {
        fprintf(stderr, "[%s] Failed to open input file: %s\n", MODULE_NAME, g_szInputPath);
        return -1;
    }

    /* Initialize SRT network streaming */
    init_srt();

    /* Initialize VPU Hardware H.264 Decoder */
    VMF_VDEC_INITOPT_T vdec_opt;
    memset(&vdec_opt, 0, sizeof(VMF_VDEC_INITOPT_T));
    vdec_opt.eCodecType = VMF_VDEC_CODEC_TYPE_H264;
    vdec_opt.dwStreamSize = STREAM_FEEDING_SIZE;
    
    ptH26xDecoder = VMF_VDEC_Init(&vdec_opt);
    if (!ptH26xDecoder) {
        fprintf(stderr, "[%s] Failed to initialize hardware decoder\n", MODULE_NAME);
        goto RELEASE;
    }

    ptH26xState = (VMF_H26XDEC_STATE_T*)VMF_VDEC_GetState(ptH26xDecoder);
    pbyInBuf = (unsigned char*)malloc(STREAM_FEEDING_SIZE);
    if (!pbyInBuf) {
        fprintf(stderr, "[%s] Failed to allocate input stream buffer\n", MODULE_NAME);
        goto RELEASE;
    }
    ptH26xState->tStreamBuf.ulVirtAddr = (unsigned long)pbyInBuf;

    /* Initialize VPU Hardware H.265 Encoder */
    VMF_CODEC_INITOPT_T codec_initopt;
    memset(&codec_initopt, 0, sizeof(VMF_CODEC_INITOPT_T));
    codec_initopt.eCodec = VMF_CODEC_ENC_HEVC; // H.265
    codec_initopt.dwEncWidth = g_dwWidth;
    codec_initopt.dwEncHeight = g_dwHeight;
    codec_initopt.dwSrcWidth = g_dwWidth;
    codec_initopt.dwSrcHeight = g_dwHeight;
    codec_initopt.dwSrcStride = ((g_dwWidth + 31) & (~31));
    codec_initopt.dwSrcChromaStride = ((g_dwWidth + 31) & (~31));
    codec_initopt.dwCropX = codec_initopt.dwCropY = 0;
    codec_initopt.dwCompressionRatio = 0;
    codec_initopt.bSubFrameSyncEn = 0;
    
    VMF_H26XENC_CONFIG_T h26xe_config;
    memset(&h26xe_config, 0, sizeof(VMF_H26XENC_CONFIG_T));
    h26xe_config.eProfile = VMF_H265ENC_PROFILE_MAIN;
    h26xe_config.dwQp = 26;
    
    // Ensure initial bitrate adheres to ABR max ceiling
    if (g_dwBitrate > ABR_MAX_BITRATE) g_dwBitrate = ABR_MAX_BITRATE;
    
    h26xe_config.dwBitrate = g_dwBitrate;
    h26xe_config.fFps = g_dwFps;
    h26xe_config.dwGop = g_dwGop;
    h26xe_config.dwMinIQp = 26; // Raised to 26 to permanently clamp I-frame size bursts from overflowing SRT buffers
    h26xe_config.dwMaxIQp = 50;
    h26xe_config.dwMinPQp = 24; // 24 for smooth P-frame quality
    h26xe_config.dwMaxPQp = 50;
    h26xe_config.dwDecodingRefreshType = 2; // IDR Frame (fixes invalid undecodable NALU and RPS/POC reference errors)

    h26xe_handle = VMF_H26xEnc_Init(&codec_initopt, &h26xe_config);
    if (!h26xe_handle) {
        fprintf(stderr, "[%s] Failed to initialize hardware encoder\n", MODULE_NAME);
        goto RELEASE;
    }

    h26xe_state = VMF_H26xEnc_GetState(h26xe_handle);
    memset(&input_info, 0, sizeof(VMF_VENC_INPUT_INFO_T));
    memset(&output_info, 0, sizeof(VMF_VENC_OUTPUT_INFO_T));
    h26xe_state->ptInputInfo = &input_info;
    h26xe_state->ptOutputInfo = &output_info;
    input_info.dwStride = ((g_dwWidth + 31) & (~31));

    /* Send SRT initial stream headers (VPS, SPS, PPS) */
    send_srt_headers(h26xe_handle);

    /* Allocate physical/virtual encoder output buffer */
    ptEncBuff = MemBroker_GetMemory(ptEncBuff_SIZE, VMF_ALIGN_TYPE_DEFAULT);
    if (!ptEncBuff) {
        fprintf(stderr, "[%s] Failed to allocate encoder output buffer\n", MODULE_NAME);
        goto RELEASE;
    }

    fprintf(stderr, "[%s] Streaming started successfully...\n", MODULE_NAME);

    unsigned int bFirstRead = 1;
    unsigned int frame_cnt = 0;
    unsigned int dec_frame_idx = 0;

    while (!g_bTerminate) {
        struct timeval start_time, end_time;
        gettimeofday(&start_time, NULL);

        /* Active Latency Sensing & Dynamic Lag Shrinking */
        if (g_bLiveMode) {
            int pipe_unread = 0;
            if (ioctl(fileno(pfInput), FIONREAD, &pipe_unread) == 0) {
                int lag_ms = (int)((long long)pipe_unread * 8000LL / (g_dwBitrate > 0 ? g_dwBitrate : 100000));
                
                // If backlog exceeds ~350ms (e.g. 4500 bytes at 100kbps), instantly flush stale backlog to real-time head
                if (pipe_unread > 4500) {
                    char flush_buf[4096];
                    int flushed = 0;
                    while (pipe_unread > 0) {
                        int to_read = pipe_unread > (int)sizeof(flush_buf) ? (int)sizeof(flush_buf) : pipe_unread;
                        int n = fread(flush_buf, 1, to_read, pfInput);
                        if (n <= 0) break;
                        flushed += n;
                        if (ioctl(fileno(pfInput), FIONREAD, &pipe_unread) != 0) break;
                    }
                    fprintf(stderr, "[LATENCY_SYNC] Backlog %d bytes (~%dms lag) cleared! Lag collapsed to < 50ms.\n", 
                            flushed, lag_ms);
                    ptH26xState->eResult = VMF_DEC_EMPTY;
                } else if (frame_cnt % 30 == 0 && frame_cnt > 0) {
                    fprintf(stderr, "[LATENCY_SYNC] Pipe Backlog: %d bytes (~%dms lag) | Status: %s\n",
                            pipe_unread, lag_ms, lag_ms < 150 ? "SYNCED" : "CATCHING_UP");
                }
            }
        }

        if (bFirstRead || VMF_DEC_EMPTY == ptH26xState->eResult) {
            bFirstRead = 0;
            unsigned int feeding_sz = g_bLiveMode ? (32 * 1024) : FEEDING_SIZE;
            unsigned int dwReadCount = fread(pbyInBuf, sizeof(unsigned char), feeding_sz, pfInput);
            if (dwReadCount == 0) { 
                if (g_bLiveMode) {
                    // In live camera mode (FIFO/pipe), do not rewind or reset frame index.
                    // Wait briefly for new incoming frames from camera pipe.
                    clearerr(pfInput);
                    usleep(1000);
                    continue;
                } else {
                    // Loop the video stream for file playback
                    fprintf(stderr, "[VDEC] Looping input video file...\n");
                    fseek(pfInput, 0, SEEK_SET);
                    dec_frame_idx = 0;
                    dwReadCount = fread(pbyInBuf, sizeof(unsigned char), FEEDING_SIZE, pfInput);
                    if (dwReadCount == 0) {
                        ptH26xState->bEndOfBitstream = 1;
                    }
                }
            }   
            ptH26xState->tStreamBuf.dwSize = dwReadCount;
        }

        int dwRet = VMF_VDEC_ProcessOneFrame(ptH26xDecoder);
        if (dwRet == 0) {
            if (ptH26xState->eResult == VMF_DEC_OK) {
                dec_frame_idx++;

                /* Frame Dropping / Downsampling:
                 * Only applied in file playback mode. In live camera mode (g_bLiveMode),
                 * capture upstream already delivers exact target FPS, so keep stride = 1.
                 */
                unsigned int stride = (g_bLiveMode || g_dwFps == 0 || g_dwFps >= 30) ? 1 : (30 / g_dwFps);
                if (stride > 1 && ((dec_frame_idx - 1) % stride != 0)) {
                    continue;
                }
                /* Hardware zero-copy YUV sharing from VDEC output to VENC input */
                input_info.tFrameBufPhys.apdwData[0] = (unsigned char*) ptH26xState->tFrameBuf.ulPhysYAddr;
                input_info.tFrameBufPhys.apdwData[1] = (unsigned char*) ptH26xState->tFrameBuf.ulPhysCbAddr;
                input_info.tFrameBufPhys.apdwData[2] = (unsigned char*) ptH26xState->tFrameBuf.ulPhysCrAddr;
                input_info.tFrameBufPhys.apdwData[3] = NULL;
                
                input_info.tFrameBuf.apdwData[0] = (unsigned char*) ptH26xState->tFrameBuf.ulVirtYAddr;
                input_info.tFrameBuf.apdwData[1] = (unsigned char*) ptH26xState->tFrameBuf.ulVirtCbAddr;
                input_info.tFrameBuf.apdwData[2] = (unsigned char*) ptH26xState->tFrameBuf.ulVirtCrAddr;
                input_info.tFrameBuf.apdwData[3] = NULL;
                
                /* Prepare encoder output destination buffer */
                output_info.pbyDstVirtBuf = (unsigned char*) ptEncBuff;
                output_info.pbyDstPhysBuf = (unsigned char*) MemBroker_GetPhysAddr(ptEncBuff);
                output_info.dwBufSize = ptEncBuff_SIZE;
                
                /* Process H.265 encoding */
                unsigned int enc_ret = VMF_H26xEnc_ProcessOneFrame(h26xe_handle);
                if (enc_ret == 0) {
                    unsigned int encoded_bytes = h26xe_state->dwEncBytes;
                    unsigned int is_key = h26xe_state->bIDR;
                    
                    /* Send encoded stream via SRT to ground station */
                    if (encoded_bytes > 0) {
                        send_srt_data(ptEncBuff, encoded_bytes, h26xe_handle);
                        frame_cnt++;

                        /* Execute Adaptive Bitrate (ABR) control loop */
                        process_abr_control(h26xe_handle, frame_cnt);

                        if (frame_cnt % 30 == 0) {
                            printf("[UAV Strm] Streamed %u frames (last %d bytes, key: %d) successfully\n", 
                                   frame_cnt, encoded_bytes, is_key);
                        }
                    }
                } else {
                    fprintf(stderr, "[Encoder] VMF_H26xEnc_ProcessOneFrame failed: %d\n", enc_ret);
                }

                if (!g_bLiveMode) {
                    gettimeofday(&end_time, NULL);
                    long long elapsed_usec = (end_time.tv_sec - start_time.tv_sec) * 1000000LL + (end_time.tv_usec - start_time.tv_usec);
                    long long frame_period_usec = 1000000LL / g_dwFps;
                    if (elapsed_usec < frame_period_usec) {
                        usleep(frame_period_usec - elapsed_usec);
                    }
                }
            }
        } else {
            if (ptH26xState->bEndOfBitstream) {
                fprintf(stderr, "[VDEC] End of bitstream reached.\n");
                break;
            } else {
                fprintf(stderr, "[VDEC] VMF_VDEC_ProcessOneFrame failed, error: %d\n", ptH26xState->eResult);
                break;
            }
        }
    }

RELEASE:
    if (ptEncBuff) {
        MemBroker_FreeMemory(ptEncBuff);
    }
    if (h26xe_handle) {
        VMF_H26xEnc_Release(h26xe_handle);
    }
    if (pbyInBuf) {
        free(pbyInBuf);
    }
    if (ptH26xDecoder) {
        VMF_VDEC_Release(ptH26xDecoder);
    }
    if (pfInput) {
        fclose(pfInput);
    }
    if (g_srt_sock != SRT_INVALID_SOCK) {
        srt_close(g_srt_sock);
    }
    srt_cleanup();
    fprintf(stderr, "[%s] Stopped and cleaned up.\n", MODULE_NAME);
    return 0;
}
