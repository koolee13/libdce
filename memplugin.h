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

#ifndef __MEMPLUGIN_H__
#define __MEMPLUGIN_H__

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#if defined(BUILDOS_QNX)
#include <memmgr.h>
#endif /* BUILDOS_QNX */

#if defined(BUILDOS_GLP)
#include <xf86drm.h>
#endif /* BUILDOS_GLP */

/* IPC Headers */
#include <tilermem.h>
#include <SharedMemoryAllocatorUsr.h>


#define P2H(p) (&(((MemHeader *)(p))[-1]))
#define H2P(h) ((void *)&(h)[1])

/* MemHeader is important because it is necessary to know the           */
/* size of the parameter buffers on IPU for Cache operations               */
/* The size can't be assumed as codec supports different inputs           */
/* For ex: static params can be VIDDEC3_Params, IVIDDEC3_Params */
/* or IH264DEC_Params                                                                   */
typedef struct MemHeader {
    int   size;
    void *ptr;
} MemHeader;

typedef enum mem_type {
    TILER_1D_BUFFER,
    TILER8_2D_BUFFER,
    TILER16_2D_BUFFER,
    SHARED_MEMORY_BUFFER,
    MEM_MAX
} mem_type;


/* DCE Error Types */
typedef enum mem_error_status {
    MEM_EOK = 0,
    MEM_EINVALID_INPUT = -1,
    MEM_EOUT_OF_TILER_MEMORY = -2,
    MEM_EOUT_OF_SHMEMORY = -3,
    MEM_EOUT_OF_SYSTEM_MEMORY = -4
} mem_error_status;

void *memplugin_alloc(int sz, int height, mem_type memory_type);

void memplugin_free(void *ptr, mem_type memory_type);

void *memplugin_share(void *ptr, mem_type memory_type);

#endif /* __MEMPLUGIN_H__ */

