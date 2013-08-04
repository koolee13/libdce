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

extern struct omap_device   *dev;


/*  memplugin_alloc - allocates omap_bo buffer with a header above it.
 *  @sz: Size of the buffer requsted
 *  @height: this parameter is currently not used
 *  @memory_type : Currently dce_alloc is used on for parameter buffer
 *  Returns a virtual address pointer to omap_bo buffer or the param buffer
 */
void *memplugin_alloc(int sz, int height, mem_type memory_type)
{
    MemHeader        *h;
    struct omap_bo   *bo = omap_bo_new(dev, sz + sizeof(MemHeader), OMAP_BO_CACHED);

    if( !bo ) {
        return (NULL);
    }

    h = omap_bo_map(bo);
    memset(H2P(h), 0, sz);
    h->size = sz;
    h->ptr = (void *)bo;
    h->dma_buf_fd = 0;

    return (H2P(h));

}

/*
 * @ptr: pointer to omap_bo buffer, to be freed
 * @memory_type: Currently dce_free is called on parameter buffers only
 */
void memplugin_free(void *ptr, mem_type memory_type)
{
    if( ptr ) {
        MemHeader   *h = P2H(ptr);
        omap_bo_del((struct omap_bo *)h->ptr);
    }
}

/* memplugin_share - converts the omap_bo buffer into dmabuf
 * @ptr : pointer of omap_bo buffer, to be converted to fd
 * Returns a file discriptor for the omap_bo buffer
 */
int memplugin_share(void *ptr)
{
    if( ptr ) {
        MemHeader   *h = P2H(ptr);
        if( !h->dma_buf_fd ) {
            h->dma_buf_fd = omap_bo_dmabuf((struct omap_bo *)h->ptr);
        }
        return (h->dma_buf_fd);
    }
    return (-1);
}

