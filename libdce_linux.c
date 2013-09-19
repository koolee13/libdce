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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <xf86drm.h>
#include <omap_drm.h>
#include <omap_drmif.h>

#include <MmRpc.h>
#include "dce_priv.h"
#include "libdce.h"
#include "memplugin.h"

#define INVALID_DRM_FD (-1)

int                    OmapDrm_FD  = INVALID_DRM_FD;
struct omap_device    *OmapDev     = NULL;
extern MmRpc_Handle    MmRpcHandle;

void *dce_init(void)
{
    dce_error_status    eError = DCE_EOK;

    DEBUG(" >> dce_init");

    /* Open omapdrm device */
    if( OmapDrm_FD == INVALID_DRM_FD ) {
        DEBUG("Open omapdrm device");
        OmapDrm_FD = drmOpen("omapdrm", "platform:omapdrm:00");
        _ASSERT(OmapDrm_FD > 0, DCE_EOMAPDRM_FAIL);
    }
    OmapDev = omap_device_new(OmapDrm_FD);
    _ASSERT(OmapDev != NULL, DCE_EOMAPDRM_FAIL);

EXIT:
    return ((void *)OmapDev);
}

void dce_deinit(void *dev)
{
    omap_device_del(dev);
    dev = NULL;
    close(OmapDrm_FD);
    OmapDrm_FD = INVALID_DRM_FD;

    return;
}

int dce_buf_lock(int num, size_t *handle)
{
    int                 i;
    MmRpc_BufDesc      *desc = NULL;
    dce_error_status    eError = DCE_EOK;

    _ASSERT(num > 0, DCE_EINVALID_INPUT);

    desc = malloc(num * sizeof(MmRpc_BufDesc));
    _ASSERT(desc != NULL, DCE_EOUT_OF_MEMORY);

    for( i = 0; i < num; i++ ) {
        desc[i].handle = handle[i];
    }

    eError = MmRpc_use(MmRpcHandle, MmRpc_BufType_Handle, num, desc);

    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);
EXIT:
    if( desc ) {
        free(desc);
    }
    return (eError);
}

int dce_buf_unlock(int num, size_t *handle)
{
    int                 i;
    MmRpc_BufDesc      *desc = NULL;
    dce_error_status    eError = DCE_EOK;

    _ASSERT(num > 0, DCE_EINVALID_INPUT);

    desc = malloc(num * sizeof(MmRpc_BufDesc));
    _ASSERT(desc != NULL, DCE_EOUT_OF_MEMORY);

    for( i = 0; i < num; i++ ) {
        desc[i].handle = handle[i];
    }

    eError = MmRpc_release(MmRpcHandle, MmRpc_BufType_Handle, num, desc);

    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);
EXIT:
    if( desc ) {
        free(desc);
    }
    return (eError);
}

/* Incase of X11 or Wayland the fd can be shared to libdce using this call */
void dce_set_fd(int dce_fd)
{
    OmapDrm_FD = dce_fd;
}

int dce_get_fd(void)
{
    return (OmapDrm_FD);
}

