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
#ifndef VENC_API_H
#define VENC_API_H

#include <video_encoder.h>
#include <video_encoder_output_srb.h>
#include <video_encoder_output_scm.h>
#include <video_encoder_adjust.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_STREAM 4

typedef struct
{
	VMF_VENC_HANDLE_T*		handle;
	VENC_ADJ_HANDLE_T* 		adj_handle;
	VMF_VENC_CONFIG_T		config;
	VMF_VENC_OUT_SRB_T*		srb_handle;
	VMF_VENC_OUT_SCM_T*     scm_handle;
	unsigned int 			aliveCount;
    unsigned int 			profile;
	unsigned int 			bitrate;
	unsigned int 			qp;
	unsigned int 			gop;
	unsigned int			minIQp;
	unsigned int			maxIQp;
	unsigned int			minPQp;
	unsigned int			maxPQp;    
	unsigned int 			encode_buffer_size;
	unsigned int 			encode_buffer_amount;
	unsigned int 			encode_check_size;
	unsigned int 			encode_block_writer;
	unsigned int 			encode_limit_frame;
	unsigned int 			block_mode;
	float 					fps;
    VMF_VENC_MODE           eProcessMode;    
    unsigned int            dwDecodingRefreshType;      //! I frame type  0: Non-IRAP 1: CRA 2: IDR 
}VEncoder;

typedef enum
{
	SRB,
	SCM
} MEM_TYPE;

typedef struct
{
	/* Common */
	unsigned int type;				// 0: H.264, 1: H.265, 2: MJPEG
	unsigned int w;					// encoding width
	unsigned int h;					// encoding height
	float        fps;			    // encoding fps
	unsigned int qp;				// qp

	unsigned int gop;				// gop
	unsigned int bitrate;			// bitrate
	unsigned int minIQp;
	unsigned int maxIQp;
	unsigned int minPQp;
	unsigned int maxPQp; 
} msg_video_t;

extern MEM_TYPE g_eType;

int venc_init( 
	VEncoder* encoder, const char* pszSrbName);

int venc_release(VEncoder* encoder);

int venc_start(VEncoder* encoder);

int venc_stop(VEncoder* encoder);

int venc_config(VEncoder* encoder);

int venc_init_codec(VMF_VENC_CONFIG_T* config);

int venc_set_piq(VMF_VENC_HANDLE_T*	handle, unsigned int enable, unsigned int wait);

int venc_load( VEncoder* encoder );
#ifdef __cplusplus
}
#endif

#endif
