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
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include "encode.h"

#include <mem_broker.h>
#include <frame_info.h>
#include <video_buf.h>
#include <video_bind.h>


#define VENC_API_16_ALIGN(a)				((a+15)&(~15))
#define VENC_FFW_DEFAULT_MAX_FRAMES			30
#define VENC_SSM_SHARED						1

static  VMF_H4E_CONFIG_T vmf_h4e_default_config = {
	VMF_H264ENC_HIGH,        // eProfile
	25, 					// quantization parameter. 
	2000000, 				// bitrate control.  0 -> no bitrate constraint. 0 -> VBR, others -> CBR 
	30, 					// input frame rate. It is used for bitrate control 
	30,						// group of pictures 
    10,                     // dwMinIQp
    50,                     // dwMaxIQp
    10,                     // dwMinPQp
    50,                     // dwMaxPQp
    0,                          
};

static  VMF_H5E_CONFIG_T vmf_h5e_default_config = {
    VMF_H265ENC_MAIN,       // eProfile
	25, 					// quantization parameter. 
	2000000, 				// bitrate control.  0 -> no bitrate constraint. 0 -> VBR, others -> CBR 
	30, 					// input frame rate. It is used for bitrate control 
	30,						// group of pictures 
    10,                     // dwMinIQp
    50,                     // dwMaxIQp
    10,                     // dwMinPQp
    50,                     // dwMaxPQp
    1,                      // I frame type  0: Non-IRAP 1: CRA 2: IDR 
};

static VMF_JE_CONFIG_T vmf_je_default_config = {
	50,						// dwQp
	0,						// bEnableThumbnail
	0,						// dwThumbnailQp
	0,						// bJfifHdr
	1048576,				// dwBitrate
	25						// fFps
};

MEM_TYPE g_eType = SRB;

int venc_init_codec(VMF_VENC_CONFIG_T* config)
{
	if(config->pCodecConfig) {
		free(config->pCodecConfig);
		config->pCodecConfig = NULL;
	}
	
	switch (config->eCodecType) {
		case VMF_VENC_CODEC_TYPE_H264:
			config->pCodecConfig = malloc(sizeof(VMF_H4E_CONFIG_T));
			memcpy(config->pCodecConfig, &vmf_h4e_default_config, sizeof(VMF_H4E_CONFIG_T));
			break;
		case VMF_VENC_CODEC_TYPE_H265:
			config->pCodecConfig = malloc(sizeof(VMF_H5E_CONFIG_T));
			memcpy(config->pCodecConfig, &vmf_h5e_default_config, sizeof(VMF_H5E_CONFIG_T));
			break;
		case VMF_VENC_CODEC_TYPE_MJPG:
			config->pCodecConfig = malloc(sizeof(VMF_JE_CONFIG_T));
			memcpy(config->pCodecConfig, &vmf_je_default_config, sizeof(VMF_JE_CONFIG_T));
			break;
		default:
			return -1;
	}

	return 0;
}

int fill_codec(VEncoder* encoder)
{
	VMF_VENC_CONFIG_T* config = &encoder->config;
	switch (config->eCodecType) {
		case VMF_VENC_CODEC_TYPE_H264:
		{		
			VMF_H4E_CONFIG_T* h4e_config = (VMF_H4E_CONFIG_T*)(config->pCodecConfig);
            h4e_config->eProfile = encoder->profile;
			h4e_config->dwQp = encoder->qp;
			h4e_config->dwBitrate = encoder->bitrate;				
			h4e_config->fFps = encoder->fps;
			h4e_config->dwGop = encoder->gop;			
			h4e_config->dwMinIQp = encoder->minIQp;
			h4e_config->dwMaxIQp = encoder->maxIQp;	
            h4e_config->dwMinPQp = encoder->minPQp;
			h4e_config->dwMaxPQp = encoder->maxPQp;	
            h4e_config->reserved = encoder->dwDecodingRefreshType;	
			break;
		}
		case VMF_VENC_CODEC_TYPE_H265:
		{		
			VMF_H5E_CONFIG_T* h5e_config = (VMF_H5E_CONFIG_T*)(config->pCodecConfig);
            h5e_config->eProfile = encoder->profile;
			h5e_config->dwQp = encoder->qp;
			h5e_config->dwBitrate = encoder->bitrate;				
			h5e_config->fFps = encoder->fps;
			h5e_config->dwGop = encoder->gop;			
			h5e_config->dwMinIQp = encoder->minIQp;
			h5e_config->dwMaxIQp = encoder->maxIQp;	
            h5e_config->dwMinPQp = encoder->minPQp;
			h5e_config->dwMaxPQp = encoder->maxPQp;	
            h5e_config->dwDecodingRefreshType = encoder->dwDecodingRefreshType;	
			break;
		}
		case VMF_VENC_CODEC_TYPE_MJPG:
		{
			VMF_JE_CONFIG_T* je_config = (VMF_JE_CONFIG_T*)(config->pCodecConfig);
			je_config->fFps = encoder->fps;
			je_config->dwQp = encoder->qp;
			je_config->dwBitrate = encoder->bitrate;
			break;
		}
		default:
			return -1;
	}	
	return 0;
}

int venc_init(
	VEncoder* encoder, const char* szPinName )
{
	if(!encoder) {
		printf("[Encoder] %s() invalid handle \n", __func__);
		return -1;		
	}
	//! Init venc output mechanism: SRB
	if (SRB == g_eType) {
		VMF_VENC_OUT_SRB_INITOPT_T tInitOpt;
		memset(&tInitOpt, 0, sizeof(VMF_VENC_OUT_SRB_INITOPT_T));
		tInitOpt.pszSrbName = szPinName;
		tInitOpt.dwSrbNum  = encoder->encode_buffer_amount;
		tInitOpt.dwSrbSize = encoder->encode_buffer_size;
		tInitOpt.bBlockMode = encoder->block_mode;
		if (0 != VMF_VENC_OUT_SRB_Init(&encoder->srb_handle, &tInitOpt)) {
			goto VENC_INIT_ERROR;
		}
		printf("[Encoder] %s() output SRB name: %s\n", __func__, szPinName);
	} else if (SCM == g_eType) {
		VMF_VENC_OUT_SCM_INITOPT_T tInitOpt;
		memset(&tInitOpt, 0, sizeof tInitOpt);
		tInitOpt.szScmName = szPinName;
		tInitOpt.dwBufSize = encoder->encode_buffer_size;
		tInitOpt.dwChkSize = encoder->encode_check_size > 0 ? encoder->encode_check_size : (unsigned) (encoder->encode_buffer_size / 3);
		tInitOpt.bBlock = encoder->encode_block_writer;
		tInitOpt.bLimit = encoder->encode_limit_frame;
		if (VMF_VENC_OUT_SCM_Init(&encoder->scm_handle, &tInitOpt)) {
				goto VENC_INIT_ERROR;
		}
		printf("[Encoder] %s() output SCM name: %s\n", __func__, szPinName);
	}

	// Init encoder
	if( -1 == venc_init_codec(&encoder->config) )
		goto VENC_INIT_ERROR;
	if( -1 == fill_codec(encoder) )
		goto VENC_INIT_ERROR;
	//encoder->config.bConnectIfp = 1;
    encoder->config.eProcessMode = encoder->eProcessMode;
    

	// Setup encoder's output (SRB)
	if (SRB == g_eType)
		VMF_VENC_OUT_SRB_Setup_Config(&encoder->config, encoder->config.eCodecType, encoder->config.pCodecConfig, encoder->srb_handle);
	else if (SCM == g_eType)
		VMF_VENC_OUT_SCM_Setup_Config(&encoder->config, encoder->config.eCodecType, encoder->config.pCodecConfig, encoder->scm_handle);
	encoder->handle = VMF_VENC_Init(&encoder->config);
	if (!encoder->handle) {
		printf("[Encoder] %s() VMF_VENC_Init() failed\n", __func__);
		goto VENC_INIT_ERROR;
	}
	VMF_VENC_ProduceStreamHdr(encoder->handle);

	return 0;

VENC_INIT_ERROR:
	printf("[Encoder] %s() VENC_INIT_ERROR\n", __func__);
	venc_release(encoder);

	return -1;
}

int venc_start(VEncoder* encoder)
{	
	if (!encoder->handle) {
		return -1;
	}	

	if (++(encoder->aliveCount) == 1) {
		printf("[%s] VMF_VENC_Start() \n", __func__);
		return VMF_VENC_Start(encoder->handle);
	}		
	return 0;
}

int venc_stop(VEncoder* encoder)
{
	if (!encoder->handle || encoder->aliveCount == 0) {
		printf("[%s] venc_stop() encoder->aliveCount %d fail\n", __func__,encoder->aliveCount);
		return -1;
	}

	if (--(encoder->aliveCount) == 0) {
		printf("[%s] VMF_VENC_Stop() \n", __func__);
		return VMF_VENC_Stop(encoder->handle);
	}	
	return 0;
}

int venc_release(VEncoder* encoder)
{
	int ret = 0;
	if (!encoder || !encoder->handle) {
		return -1;
	}
	else {
		if(encoder->aliveCount) {
			ret = VMF_VENC_Stop(encoder->handle);
			if (ret) {
				printf("[Encoder] VMF_VENC_Stop faild, ret = %d \n", ret);
			}
		}
		if(encoder->adj_handle) {
			VMF_VENC_ADJ_Release(encoder->adj_handle);
			encoder->adj_handle = NULL;
		}
		ret = VMF_VENC_Release(encoder->handle);
		if (ret) {
			printf("[Encoder] VMF_VENC_Release faild, ret = %d \n", ret);
		}		
		encoder->handle = NULL;
	}

	if(encoder->config.pCodecConfig) {
		free((void*)(encoder->config.pCodecConfig));
		encoder->config.pCodecConfig = NULL;
	}	
	//! Release venc output mechanism: SRB
//	ret = VMF_VENC_OUT_SRB_Release(&encoder->outHandle);
	if (SRB == g_eType)
		ret = VMF_VENC_OUT_SRB_Release(&encoder->srb_handle);
	else if (SCM == g_eType)
		ret = VMF_VENC_OUT_SCM_Release(&encoder->scm_handle);
	if (ret) {
		printf("[Encoder] VMF_VENC_OUT_%s_Release faild, ret = %d \n", SRB == g_eType ? "SRB" : "SCM", ret);
	}	
	return ret;
}

int venc_config(VEncoder* encoder)
{
	if ( !encoder || !encoder->handle ) {
		printf("[Encoder] %s() invalid handle\n", __func__);
		return -1;
	}

	if(-1 == fill_codec(encoder)) {
		printf("[Encoder] %s() fill codec parameter failed!\n", __func__);
		return -1;
	}

	return VMF_VENC_Config(encoder->handle, &encoder->config);
}

int venc_load( VEncoder* encoder )
{
	VMF_VENC_CONFIG_T* config = &encoder->config;

	memset( encoder, 0, sizeof(VEncoder) );

	config->eCodecType = VMF_VENC_CODEC_TYPE_H264;
	encoder->profile = VMF_H264ENC_HIGH;

	config->dwEncWidth = 640;	//1280;	//1920;
	config->dwEncHeight = 480;	//720;	//1080;
	encoder->fps = 30;
	config->bConnectIfp = 0;
	encoder->bitrate = 1024 * 1024 * 5 / 2;	//2000000;
	encoder->qp = 25;
	encoder->minIQp = 10;
	encoder->maxIQp = 50;
	encoder->minPQp = 10;
	encoder->maxPQp = 50;

	encoder->gop = 30;
	encoder->encode_buffer_size = 6*1024*1024;
	encoder->encode_buffer_amount = 4;
	encoder->encode_check_size = 0;
	encoder->encode_block_writer = 1;
	encoder->encode_limit_frame = 1;
	config->dwNoSignalBackGroundColorY = 0;
	config->dwNoSignalBackGroundColorU = 128;
	config->dwNoSignalBackGroundColorV = 128;
	encoder->dwDecodingRefreshType = 1;
	encoder->block_mode = 0;
	encoder->eProcessMode = 0;

	return 0;
}

