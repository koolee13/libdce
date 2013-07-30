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

#ifndef __LIBDCE_H__
#define __LIBDCE_H__


#include <ti/sdo/ce/Engine.h>
#include <ti/sdo/ce/video3/viddec3.h>
#include <ti/sdo/ce/video2/videnc2.h>


/* DCE Error Types */
typedef enum dce_error_status {
    DCE_EOK = 0,
    DCE_EXDM_FAIL = -1,
    DCE_EOUT_OF_MEMORY = -2,
    DCE_EXDM_UNSUPPORTED = -3,
    DCE_EIPC_CREATE_FAIL = -4,
    DCE_EIPC_CALL_FAIL = -5,
    DCE_EINVALID_INPUT = -6
} dce_error_status;


/* other than the codec-engine API, you must use the following two functions
 * to allocate the data structures passed to codec-engine APIs (other than the
 * raw input/output buffers which should be passed as virtual addresses in
 * TILER space
 */
void *dce_alloc(int sz);
void dce_free(void *ptr);


#if defined(BUILDOS_LINUX)
void dce_set_fd(int fd);
int dce_get_fd();

/* avoid some messy stuff in xdc/std.h which leads to gcc issues */
#define xdc__ARGTOPTR
#define xdc__ARGTOFXN

#define XDM_MEMTYPE_BO 10
#define XDM_MEMTYPE_BO_OFFSET 11
#endif /* BUILDOS_LINUX */

#endif /* __LIBDCE_H__ */

