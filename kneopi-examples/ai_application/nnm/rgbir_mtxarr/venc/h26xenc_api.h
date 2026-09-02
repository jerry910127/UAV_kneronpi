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
#ifndef H26XE_API_H
#define H26XE_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <video_h26xenc.h>
#include <video_encoder.h>
typedef struct
{
	unsigned int dwEncBytes;
	unsigned int bIdr;
} H26XE_OUTPUT_INFO_T;

int h26xe_init(VMF_CODEC_INITOPT_T* ptInitOpt);

int h26xe_get_header_info(VMF_CODEC_TYPE eCodec, VMF_VENC_H265_STREAM_HDR* ptHdr);

int h26xe_process(VMF_FRAME_BUF_T* virt_yuv_buf, VMF_FRAME_BUF_T* phys_yuv_buf, void* encoded_buf, unsigned int buf_size, H26XE_OUTPUT_INFO_T* info);

int h26xe_release();

#ifdef __cplusplus
}
#endif

#endif
