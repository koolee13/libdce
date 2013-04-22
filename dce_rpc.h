/*
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DCE_RPC_H__
#define __DCE_RPC_H__

/* RPC layer types.. these define the payload of messages between IPUMM
 * and MPU.  This should be kept in sync between firmware build and
 * driver.
 *
 * TODO: xxx_control(XDM_GETVERSION) is a bit awkward to deal with, because
 * this seems to be the one special case where status->data is used..
 * possibly we should define a special ioctl and msg to handle this case.
 */

#define DCE_DEVICE_NAME "rpmsg-dce"

/* Message-Ids:
 */
//#define DCE_RPC_CONNECT         (0x80000000 | 00) Connect not needed anymore.
typedef enum dce_rpc_call {
    DCE_RPC_ENGINE_OPEN = 0,
    DCE_RPC_ENGINE_CLOSE,
    DCE_RPC_CODEC_CREATE,
    DCE_RPC_CODEC_CONTROL,
    DCE_RPC_CODEC_GET_VERSION,
    DCE_RPC_CODEC_PROCESS,
    DCE_RPC_CODEC_DELETE
} dce_rpc_call;


typedef enum dce_codec_type {
    OMAP_DCE_VIDENC2 = 1,
    OMAP_DCE_VIDDEC3 = 2
} dce_codec_type;

/* Structures of RPC */
typedef struct dce_connect {
    uint32_t chipset_id;
    uint32_t debug;
} dce_connect;

typedef struct dce_engine_open {
    char          name[32]; /* engine name (in) */
    Engine_Error  error_code;   /* error code (out) */
    Engine_Handle eng_handle;   /* engine handle (out) */
} dce_engine_open;

typedef struct dce_engine_close {
    Engine_Handle eng_handle;   /* engine handle (in) */
} dce_engine_close;

typedef struct dce_codec_create {
    Engine_Handle  engine;
    char           codec_name[32];
    void          *static_params;
    dce_codec_type codec_id;
    void          *codec_handle;
} dce_codec_create;

typedef struct dce_codec_control {
    void          *codec_handle;
    uint32_t       cmd_id;
    void          *dyn_params;
    void          *status;
    dce_codec_type codec_id;
    int32_t        result;
} dce_codec_control;

typedef struct dce_codec_get_version {
    void          *codec_handle;
    void          *dyn_params;
    void          *status;
    void          *version;
    dce_codec_type codec_id;
    int32_t        result;
} dce_codec_get_version;

typedef struct dce_codec_delete {
    void          *codec_handle;
    dce_codec_type codec_id;
} dce_codec_delete;

/*  ---------------------- For GLP DCE -----------------------------*/
/* NOTE: CODEC_PROCESS does somewhat more than the other ioctls, in that it
 * handles buffer mapping/unmapping.  So the inBufs/outBufs are copied inline
 * (with translated addresses in the copy sent inline with codec_process_req).
 * Since we need the inputID from inArgs, and it is a small struct, it is also
 * copied inline.
 *
 * Therefore, the variable length data[] section has the format:
 *    uint8_t reloc[reloc_length * 4];
 *    uint8_t inargs[in_args_length * 4];
 *    uint8_t outbufs[in_bufs_length * 4];
 *    uint8_t inbufs[in_bufs_length * 4];
 */

#define MAX_INPUT_BUF 2 // Need to confirm for interlaced YUVs for Encoders
#define MAX_OUTPUT_BUF 2
#define MAX_TOTAl_BUF (MAX_INPUT_BUF + MAX_OUTPUT_BUF)

/* Struct to be used if approach [3] of Mmrpc call is followed */
typedef struct dce_codec_process {
    void          *codec_handle;
    void          *inBufs;
    void          *outBufs;
    void          *inArgs;
    void          *outArgs;
    int32_t        input_Buf[MAX_INPUT_BUF];
    int32_t        output_Buf[MAX_OUTPUT_BUF];
    dce_codec_type codec_id;
    int32_t        result;
} dce_codec_process;

#endif /* __DCE_RPC_H__ */

