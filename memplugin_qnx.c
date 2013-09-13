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

#include "memplugin.h"
#include "dce_priv.h"


/* For TILER 2D Buffers : sz       = width                              */
/*                                : height = nonzero                           */
/* For other memory_types : height = 0                               */
void *memplugin_alloc(int sz, int height, MemRegion region, int align, int flags)
{
    MemHeader          *h;
    shm_buf            *handle;
    MemAllocBlock       block;
    int                 num_block;
    mem_error_status    eError = MEM_EOK;

    _ASSERT(sz != 0, MEM_EINVALID_INPUT);
    _ASSERT((region < MEM_MAX) && (region >= MEM_TILER_1D), MEM_EINVALID_INPUT);

    /* Tiler buffer Allocations : Only Tiler 1D used for Parameter Buffers and RPC Message buffers */
    if( region == MEM_TILER_1D || region == MEM_TILER8_2D || region == MEM_TILER16_2D ) {
        num_block = 1;

        if( region == MEM_TILER_1D ) {
            /* Allocate in tiler paged mode (1d) container */
            block.pixelFormat = PIXEL_FMT_PAGE;
            block.dim.len = sz + sizeof(MemHeader);
#if 0 /* Data Tiler Buffers not to be allocated by DCE */
        } else if( region == MEM_TILER8_2D ) {
            /* Allocate in tiler 8 bit mode (2d) container */
            _ASSERT(height != 1, MEM_EINVALID_INPUT);
            block.pixelFormat = PIXEL_FMT_8BIT;
            block.dim.area.width  = sz;
            block.dim.area.height = height;
        } else {
            /* Allocate in tiler 16 bit mode (2d) container */
            _ASSERT(height != 1, MEM_EINVALID_INPUT);
            block.pixelFormat = PIXEL_FMT_16BIT;
            block.dim.area.width  = sz;
            block.dim.area.height = height;
#endif
        }
        block.stride = 0;

        h = MemMgr_Alloc(&block, num_block);

        _ASSERT(h != NULL, MEM_EOUT_OF_TILER_MEMORY);

        h->size = sz;
        h->region = region;
        memset(H2P(h), 0, sz);
        return (H2P(h));
    } else {

        handle = malloc(sizeof(shm_buf));
        _ASSERT(handle != NULL, MEM_EOUT_OF_SYSTEM_MEMORY);

        /* Parameter Buffers : Allocate from Shared Memory considering MemHeader*/
        SHM_alloc(sz + sizeof(MemHeader), handle);

        h = (MemHeader *)handle->vir_addr;
        _ASSERT_AND_EXECUTE(h != NULL, MEM_EOUT_OF_SHMEMORY, free(handle));

        h->size = sz;
        h->region = region;
        h->ptr = handle;
        memset(H2P(h), 0, sz);
        return (H2P(h));
    }
EXIT:
    return (NULL);
}

/* memory_type is not added if MemHeader is used for Tiler Buffers */
void memplugin_free(void *ptr)
{
    shm_buf            *handle;
    mem_error_status    eError = MEM_EOK;
    MemRegion region;

    _ASSERT(ptr != NULL, MEM_EINVALID_INPUT);
    region = (P2H(ptr))->region;
    _ASSERT((region < MEM_MAX) && (region >= MEM_TILER_1D), MEM_EINVALID_INPUT);

    if( region == MEM_TILER_1D ) {
        MemMgr_Free(P2H(ptr));
    } else if( region == MEM_SHARED ) {
        handle = (P2H(ptr))->ptr;
        SHM_release(handle);
        free(handle);
    } else {
        ERROR("Tiler 2D Allocation/Free not implemented");
    }
EXIT:;
}

inline int32_t memplugin_share(void *ptr)
{
    /* No Userspace Virtual pointers to DMA BUF Handles conversion required*/
    /* Do nothing */
    return ((int32_t)ptr);
}

