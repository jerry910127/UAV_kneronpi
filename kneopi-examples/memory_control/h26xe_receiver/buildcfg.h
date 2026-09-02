#ifndef __BUILDCFG_H__
#define __BUILDCFG_H__

#if 1	// for h26xe receiver
#define __MAIN__

#else	// for rgbir
#define VENC_EN
#define NNM_EN

#define LIVEVIEW_EN
#define IR_EN

//#define VENC_RECV_EN

#define RGB_CONV
#define IR_CONV
#define IR_FEC
#define RGBIR_OVERLAY
#define VIDEO_STREAM
#define RGBIR_CTL
#endif

#endif
