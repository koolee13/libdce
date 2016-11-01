/*
 * Copyright (c) 2014, Texas Instruments Incorporated
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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/eventfd.h>
#include <pthread.h>

#include "memplugin.h"
#include "libdce.h"

#include <xf86drm.h>
#include <omap_drm.h>
#include <omap_drmif.h>
#include <utils/Log.h>


#define LINUX_PAGE_SIZE 4096
#define INVALID_DRM_FD (-1)

int                    OmapDrm_FD  = INVALID_DRM_FD;
struct omap_device    *OmapDev     = NULL;
extern pthread_mutex_t    ipc_mutex;
extern int is_ipc_ready;

int memplugin_open()
{
    if (OmapDev)
        return MEM_EOK;

    /* Open omapdrm device */
    if( OmapDrm_FD == INVALID_DRM_FD ) {
	OmapDrm_FD = open("/dev/dri/renderD128", O_RDWR, 0);
        if(OmapDrm_FD <= 0) {
            ALOGE("omapdrm open failed\n");
            return MEM_EOPEN_FAILURE;
        }
    }
    OmapDev = omap_device_new(OmapDrm_FD);
    if(OmapDev == NULL) {
        close(OmapDrm_FD);
        return MEM_EOPEN_FAILURE;
    }

    pthread_mutexattr_t attr;
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&ipc_mutex, &attr);

    return MEM_EOK;
}

int memplugin_close()
{
    if (OmapDev) {
        omap_device_del(OmapDev);
        OmapDev = NULL;
    }

    if (OmapDrm_FD > 0) {
        close(OmapDrm_FD);
        OmapDrm_FD = INVALID_DRM_FD;
    }

    /* this ipc client is not associated with any codec engine
    * this channel is used only for GEM buffer register/unregister,
    * hence call deinit with -1 for the engine table index
    */

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    dce_ipc_deinit(IPU, -1);
    is_ipc_ready = 0;

    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);

    return MEM_EOK;
}


void *memplugin_alloc(int sz, int height, MemRegion region, int align, int flags)
{
    MemHeader        *h = NULL;
    struct omap_bo   *bo = omap_bo_new(OmapDev, sz + sizeof(MemHeader), OMAP_BO_WC);

    if( !bo ) {
        return (NULL);
    }

    h = omap_bo_map(bo);

    h->ptr = h + 1;
    memset(H2P(h), 0, sz);

    h->size = sz;
    /* get the fd from drm which needs to be closed by memplugin_free */
    h->dma_buf_fd = omap_bo_dmabuf(bo);
    h->region = region;
    h->flags = flags;/*Beware: This is a bit field.*/
    h->handle = (void*)bo;
    h->offset = 0;

    dce_buf_lock(1, (size_t *)&(h->dma_buf_fd));

    return (H2P(h));

}

void memplugin_free(void *ptr)
{
    if( ptr ) {
        MemHeader   *h = P2H(ptr);
        if( h->dma_buf_fd ) {
            dce_buf_unlock(1, (size_t *)&(h->dma_buf_fd));
            /* close the file descriptor */
            close(h->dma_buf_fd);
        }
        /*Finally, Delete the buffer object*/
        omap_bo_del((struct omap_bo *)h->handle);
        ptr = NULL;
    }

    return;
}

int32_t memplugin_share(void *ptr)
{
    if (!ptr)
        return MEM_EINVALID_INPUT;

    MemHeader *h = P2H(ptr);
    return h->dma_buf_fd;

}

void *memplugin_alloc_noheader(MemHeader *memHdr, int sz, int height, MemRegion region, int align, int flags)
{
    MemHeader *h = memHdr;
    if (!memHdr)
        return NULL;

    struct omap_bo   *bo = omap_bo_new(OmapDev, sz, OMAP_BO_WC);

    if( !bo ) {
        return (NULL);
    }

    h->ptr = omap_bo_map(bo);

    memset(h->ptr, 0, sz);

    h->size = sz;
    h->handle = (void *)bo;
    /* get the fd from drm which needs to be closed by memplugin_free */
    h->dma_buf_fd = omap_bo_dmabuf(bo);
    h->region = region;
    h->flags = flags;/*Beware: This is a bit field.*/
    h->offset = 0;

    dce_buf_lock(1, (size_t *)&(h->dma_buf_fd));

    return (h->ptr);

}

void memplugin_free_noheader(MemHeader *memHdr)
{
    if (!memHdr)
        return;

    MemHeader *h = memHdr;
    if( h->dma_buf_fd ) {
        dce_buf_unlock(1, (size_t *)&(h->dma_buf_fd));
        /* close the file descriptor */
        close(h->dma_buf_fd);
    }

    /*Finally, Delete the buffer object*/
    omap_bo_del((struct omap_bo *)h->handle);

    return;
}

