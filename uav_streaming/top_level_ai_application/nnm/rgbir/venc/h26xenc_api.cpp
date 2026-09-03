
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
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sched.h>
#include <errno.h>

#include <mem_broker.h>
#include <mem_util.h>
#include <video_encoder.h>
#include "h26xenc_api.h"

using namespace std;
VMF_H26XENC_HANDLE_T* h26xe_handle = NULL;
VMF_H26XENC_STATE_T* h26xe_state = NULL;


VMF_VENC_INPUT_INFO_T input_info;
VMF_VENC_OUTPUT_INFO_T output_info;
VMF_CODEC_NAL_PARAM_SETS_T tSpsPps;

int h26xe_init(VMF_CODEC_INITOPT_T* ptInitOpt)
{
	VMF_H26XENC_CONFIG_T h26xe_config;
	memset(&h26xe_config, 0, sizeof(VMF_H26XENC_CONFIG_T));
    memset(&input_info, 0, sizeof(VMF_VENC_INPUT_INFO_T));        
    memset(&output_info, 0, sizeof(VMF_VENC_OUTPUT_INFO_T));

    h26xe_config.eProfile = ptInitOpt->eCodec?VMF_H265ENC_PROFILE_MAIN:VMF_H264ENC_PROFILE_HIGH;
	h26xe_config.dwQp = 25;
	h26xe_config.dwBitrate = 2000000;
	h26xe_config.fFps = 15;//30;
	h26xe_config.dwGop = h26xe_config.fFps;
    h26xe_config.dwMinIQp = 10;
	h26xe_config.dwMaxIQp = 50;
    h26xe_config.dwMinPQp = 10;
	h26xe_config.dwMaxPQp = 50;

	h26xe_handle = VMF_H26xEnc_Init(ptInitOpt, &h26xe_config);
	if (h26xe_handle) {
		h26xe_state = VMF_H26xEnc_GetState(h26xe_handle);
		h26xe_state->ptInputInfo = &input_info;
        h26xe_state->ptOutputInfo = &output_info;
        input_info.dwStride = ptInitOpt->dwSrcStride;
	} else {
		printf("[%s] H26XENC handle init failed!!\n", __func__);
		return -1;
	}
    
	return 0;
}

int h26xe_get_header_info(VMF_CODEC_TYPE eCodec, VMF_VENC_H265_STREAM_HDR * ptHdr)
{
	if (!h26xe_handle) {
		printf("[%s] H5E handle not exist! \n", __func__);
		return 0;
	}
	int ret = -1;
	unsigned char* sps = NULL;
	unsigned char* pps = NULL;
	unsigned char* vps = NULL;;

	VMF_CODEC_OPTION_T  tOpt;

	memset(&tSpsPps, 0, sizeof(VMF_CODEC_NAL_PARAM_SETS_T));

	sps = (unsigned char *)MemBroker_GetMemory(128, VMF_ALIGN_TYPE_DEFAULT);
	pps = (unsigned char *)MemBroker_GetMemory(128, VMF_ALIGN_TYPE_DEFAULT);
	if (eCodec == VMF_CODEC_ENC_HEVC)
		vps = (unsigned char *)MemBroker_GetMemory(128, VMF_ALIGN_TYPE_DEFAULT);

	tSpsPps.pbySps = sps;
	tSpsPps.pbyPps = pps;
    if (eCodec == VMF_CODEC_ENC_HEVC)
    	tSpsPps.pbyVps = vps;
    
    tSpsPps.dwSpsSize = tSpsPps.dwPpsSize = tSpsPps.dwVpsSize = 128;

	memset(&tOpt, 0, sizeof(VMF_CODEC_OPTION_T));
	tOpt.eOptionFlag = VMF_CODEC_H26XE_GET_HEADER_INFO;
	tOpt.adwData[0]  = addr2uint(&tSpsPps);
	ret = VMF_H26xEnc_SetOptions(h26xe_handle, &tOpt);

	if (eCodec == VMF_CODEC_ENC_HEVC)
		ptHdr->dwVpsSize = tSpsPps.dwVpsSize;
	else
		ptHdr->dwVpsSize = 0;
	ptHdr->dwSpsSize = tSpsPps.dwSpsSize;
	ptHdr->dwPpsSize = tSpsPps.dwPpsSize;

	if(ptHdr->dwVpsSize)
		memcpy(ptHdr->abyVpsData, vps, ptHdr->dwVpsSize);
	memcpy(ptHdr->abySpsData, sps, ptHdr->dwSpsSize);
	memcpy(ptHdr->abyPpsData, pps, ptHdr->dwPpsSize);

	MemBroker_FreeMemory(tSpsPps.pbySps);
	MemBroker_FreeMemory(tSpsPps.pbyPps);
    if (eCodec == VMF_CODEC_ENC_HEVC) {
    	MemBroker_FreeMemory(tSpsPps.pbyVps);
    }
	return ret;
}

int h26xe_process(VMF_FRAME_BUF_T* virt_yuv_buf, VMF_FRAME_BUF_T* phys_yuv_buf, void* encoded_buf, unsigned int buf_size, H26XE_OUTPUT_INFO_T* info)
{
    (void)virt_yuv_buf;
    
	if (!h26xe_handle) {
		printf("[%s] H26XEnc handle not exist! \n", __func__);
		return 0;
	}

	unsigned int ret = 0;

    input_info.tFrameBufPhys.apdwData[0] = phys_yuv_buf->apdwData[0];
    input_info.tFrameBufPhys.apdwData[1] = phys_yuv_buf->apdwData[1];
    input_info.tFrameBufPhys.apdwData[2] = phys_yuv_buf->apdwData[2]; 
    
	output_info.pbyDstVirtBuf = (unsigned char*) encoded_buf;
	output_info.pbyDstPhysBuf = (unsigned char*)MemBroker_GetPhysAddr(encoded_buf);
	output_info.dwBufSize = buf_size;
	
    if( (ret = VMF_H26xEnc_ProcessOneFrame(h26xe_handle)) == 0) {
        info->dwEncBytes = h26xe_state->dwEncBytes;
		info->bIdr = h26xe_state->bIDR;
    }
	return ret;
}

int h26xe_release()
{
	if (!h26xe_handle) {
		printf("[%s] H26XE handle not exist! \n", __func__);
		return 0;
	}

	return VMF_H26xEnc_Release(h26xe_handle);
}

