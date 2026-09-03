#ifndef __BUILDCFG_H__
#define __BUILDCFG_H__

#if 0	// for h26xe receiver
#define __MAIN__

#else	// for rgbir
#define VENC_EN
#define NNM_EN

#define LIVEVIEW_EN
#define IR_EN

#define TOF_EN
#define CTRL_BOARD
#define VENC_RECV_EN

#define RGB_CONV
#ifdef IR_EN
#   define IR_CONV
#   define IR_FEC
#   define RGBIR_OVERLAY
#endif
#define VIDEO_STREAM

#ifdef IR_EN
#   define RGBIR_CTL
#endif
#define VIDEO_SAVE
#endif

#endif
