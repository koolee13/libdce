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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <ti/ipc/mm/MmRpc.h>
#include "dce_priv.h"
#include "libdce.h"
#include "dce_rpc.h"
#include "memplugin.h"


extern MmRpc_Handle    MmRpcHandle[];

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

    eError = MmRpc_use(MmRpcHandle[IPU], MmRpc_BufType_Handle, num, desc);

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

    eError = MmRpc_release(MmRpcHandle[IPU], MmRpc_BufType_Handle, num, desc);

    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);
EXIT:
    if( desc ) {
        free(desc);
    }
    return (eError);
}


