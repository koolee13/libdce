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
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/mman.h>

#include "libdce.h"

#include <tilermem.h>
#include <memmgr.h>
#include <ti/sdo/ce/Engine.h>
#include <ti/sdo/ce/video2/videnc2.h>
#include <ti/sdo/codecs/h264enc/ih264enc.h>
#include <ti/sdo/codecs/mpeg4enc/impeg4enc.h>


#define OMAP5

#define PRINT_DEBUG
//#define H264_DEBUG

#define ERROR(FMT, ...)  printf("%s:%d:\t%s\terror: " FMT "\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// enable below to print debug information
#ifdef PRINT_DEBUG
#define DEBUG(FMT, ...)  printf("%s:%d:\t%s\tdebug: " FMT "\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#else
#define DEBUG(FMT, ...)
#endif
#define INFO(FMT, ...)  printf("%s:%d:\t%s\tinfo: " FMT "\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define MIN(a, b)        (((a) < (b)) ? (a) : (b))

/* align x to next highest multiple of 2^n */
#define ALIGN2(x, n)   (((x) + ((1 << (n)) - 1)) & ~((1 << (n)) - 1))

// Profile the init and decode calls
//#define PROFILE_TIME

// NON-TILER OUTPUT configuration
//#define NON_TILER_OUTPUT

enum {
    IVAHD_H264_ENCODE,
    IVAHD_MPEG4_ENCODE,
    IVAHD_H263_ENCODE
};

/*
 * A very simple VIDENC2 client which will encode raw (unstrided) NV12 YUV frames
 * and write out to either h264, MPEG4, or H.263 format.
 */

// GLOBAL VARIABLES
int                      width, height, padded_width, padded_height, num_buffers, tiler;
Engine_Handle            engine    = NULL;
VIDENC2_Handle           codec     = NULL;
VIDENC2_Params          *params    = NULL;
VIDENC2_DynamicParams   *dynParams = NULL;
VIDENC2_Status          *status    = NULL;
VIDENC2_DynamicParams   *dynParams1 = NULL;
VIDENC2_Status          *status1    = NULL;
IVIDEO2_BufDesc         *inBufs    = NULL;
XDM2_BufDesc            *outBufs   = NULL;
VIDENC2_InArgs          *inArgs    = NULL;
VIDENC2_OutArgs         *outArgs   = NULL;

// H.264 specific
IH264ENC_InArgs          *h264enc_inArgs = NULL;
IH264ENC_OutArgs         *h264enc_outArgs = NULL;
IH264ENC_Params          *h264enc_params    = NULL;
IH264ENC_DynamicParams   *h264enc_dynParams = NULL;
IH264ENC_Status          *h264enc_status    = NULL;

// MPEG4/H.263 specific
IMPEG4ENC_InArgs          *mpeg4enc_inArgs    = NULL;
IMPEG4ENC_OutArgs         *mpeg4enc_outArgs   = NULL;
IMPEG4ENC_Params          *mpeg4enc_params    = NULL;
IMPEG4ENC_DynamicParams   *mpeg4enc_dynParams = NULL;
IMPEG4ENC_Status          *mpeg4enc_status    = NULL;

unsigned int    frameSize[64000]; /* Buffer for keeping frame sizes */
static int      input_offset = 0;

static void *tiler_alloc(int width, int height)
{
    int              dimensions;
    void            *bufPtr = NULL;
    MemAllocBlock    block;
    MemAllocBlock    blocks[2];

    memset(&block, 0, sizeof(MemAllocBlock));
    memset(blocks, 0, sizeof(MemAllocBlock) * 2);

    if( !height ) {
        DEBUG("tiler alloc 1D allocation width=%d", width);
        /* 1d allocation: */
        dimensions = 1;

        block.pixelFormat = PIXEL_FMT_PAGE;
        block.dim.len = width;
        block.stride = 0;

        bufPtr = MemMgr_Alloc(&block, dimensions);
    } else {
        DEBUG("tiler alloc 2D allocation width=%d height=%d", width, height);
        /* 2d allocation: */
        dimensions = 2;
        blocks[0].pixelFormat = PIXEL_FMT_8BIT;
        blocks[0].dim.area.width  = width;
        blocks[0].dim.area.height = height;
        blocks[0].stride = 0;

        blocks[1].pixelFormat = PIXEL_FMT_16BIT;
        blocks[1].dim.area.width  = width >> 1;
        blocks[1].dim.area.height = height >> 1;
        blocks[1].stride = 0;

        bufPtr = MemMgr_Alloc(blocks, dimensions);
    }
    DEBUG("tiler alloc return bufPtr %p", bufPtr);

    return (bufPtr);
}

#ifdef NON_TILER_OUTPUT
static XDAS_Int16
get_mem_type (SSPtr paddr)
{
    if((0x60000000 <= paddr) && (paddr < 0x68000000)) {
        return (XDM_MEMTYPE_TILED8);
    }
    if((0x68000000 <= paddr) && (paddr < 0x70000000)) {
        return (XDM_MEMTYPE_TILED16);
    }
    if((0x70000000 <= paddr) && (paddr < 0x78000000)) {
        return (XDM_MEMTYPE_TILED32);
    }
    if((0x78000000 <= paddr) && (paddr < 0x80000000)) {
        return (XDM_MEMTYPE_RAW);
    }
    return (-1);
}

/*! @brief Start address of DDR region for 1GB RAM */
#define DDR_1G_ADDRESS_START           0x80000000
/*! @brief End address of DDR region for 1GB RAM */
#define DDR_1G_ADDRESS_END             0xBFFFFFFF
#define DDR_1G_DUCATI_OFFSET           0x40000000

/*! @brief Start address of DDR region for 2GB RAM */
#define DDR_2G_ADDRESS_START           0xC0000000
/*! @brief End address of DDR region for 2GB RAM */
#define DDR_2G_ADDRESS_END             0xFFFFFFFF
#define DDR_2G_DUCATI_OFFSET           0xA0000000

static Int
SysLinkMemUtils_translateAddr (UInt32 physAddr)
{
    Int    ret = 0;

    if( physAddr >= DDR_1G_ADDRESS_START && physAddr <= DDR_1G_ADDRESS_END ) {
        ret = physAddr + DDR_1G_DUCATI_OFFSET;
    } else if( physAddr >= DDR_2G_ADDRESS_START && physAddr <= DDR_2G_ADDRESS_END ) {
        ret = physAddr - DDR_2G_DUCATI_OFFSET;
    }

    return (ret);
}

static void *output_allocate_nonTiler(int size)
{
    DEBUG(" ----------------- create nonTILER size %d --------------------", size);

    // Allocation through mmap
    uint64_t   *vaddr;
    int32_t     ret, len = 0;
    int64_t     paddr = 0;

    vaddr = mmap64(0, size, PROT_NOCACHE | PROT_READ | PROT_WRITE, MAP_ANON | MAP_PHYS | MAP_SHARED, NOFD, 0);
    if( vaddr == MAP_FAILED ) {
        ERROR("Failed to do memory mapping\n");
        return (NULL);
    }

    // Make sure the memory is contiguous
    ret = mem_offset64(vaddr, NOFD, (size_t) size, &paddr, (size_t *) &len);
    if( ret ) {
        ERROR("Failed to check memory contiguous ret %d errno %d\n", ret, errno);
        munmap(vaddr, size);
        return (NULL);
    } else if( len != (size)) {
        ERROR("Failed to check len %d != %d\n", len, size);
        munmap(vaddr, size);
        return (NULL);
    }

    DEBUG("nonTILER buf virtual address =%08x, paddr=%08x return %08x", vaddr, (unsigned int) TilerMem_VirtToPhys(vaddr), SysLinkMemUtils_translateAddr(TilerMem_VirtToPhys(vaddr)));

    // TODO: For IPC 3.x we need to return the virtual address instead of the physical address.
    // return (void*) vaddr;
    return ((void *) SysLinkMemUtils_translateAddr(TilerMem_VirtToPhys(vaddr)));

}

static void output_enc_free(void *vaddr, int size)
{
    munmap(vaddr, size);
}

#endif

/* ************************************************************************* */
/* utilities to allocate/manage 2d input buffers */

typedef struct InputBuffer InputBuffer;

struct InputBuffer {
    char        *buf; /* virtual address for local access, 4kb stride */
    SSPtr        y, uv; /* physical addresses of Y and UV for remote access */
    InputBuffer *next;      /* next free buffer */
    bool         tiler;
    uint32_t     len;
};

/* ************************************************************************* */

/* get file path.. return path is only valid until next time this is called */
static const char *get_path(const char *pattern, int cnt)
{
    static int     len = 0;
    static char   *path = NULL;

    /* It would be better to not assume the pattern doesn't expand to
         * less than 10 chars it's original length..
         */
    if((strlen(pattern) + 10) > len ) {
        len  = strlen(pattern) + 10;
        path = realloc(path, len);
    }

    snprintf(path, len - 1, pattern, cnt);

    return (path);
}

//#define DUMPINPUTDATA
//#define TMPDUMPDATA
//#define DUMPOUTPUTDATA

#ifdef DUMPINPUTDATA
FILE   *inputDump;
#endif

#ifdef TMPDUMPDATA
FILE   *tmpDump;
#endif

#ifdef DUMPOUTPUTDATA
FILE   *outputDump;
#endif


/* helper to read one frame NV12 of input */
int read_input(const char *pattern, int cnt, char *input)
{
    int    sz = 0, n = 0, num_planes, i, buf_height;
    int   *pTmpBuffer = NULL, *pTmpBufferPos = NULL;

    const char   *path = get_path(pattern, cnt);
    int           fd = open(path, O_RDONLY);

    //DEBUG("Open file fd %d errno %d", fd, errno);
    /* if we can't find the file, then at the end of stream */
    if( fd < 0 ) {
        DEBUG("open input file failed");
        return (0);
    }

    sz = width * height * 3 / 2;
    pTmpBuffer = calloc(sz, 1);
    pTmpBufferPos = pTmpBuffer;

    //Read 1 frame of NV12 YUV into input. Since it's TILER - stride = 4096
    lseek(fd, input_offset, SEEK_SET);
    n = read(fd, pTmpBuffer, sz); //reading 1 input frame width * height * 3/2
    DEBUG("reading input[%p] size of n = %d where input_offset is %d", input, n, input_offset);
    if( n ) {
        input_offset += n;
    } else {
        sz = -1; //cannot read more input
        free(pTmpBuffer);
        close(fd);

        DEBUG("sz=%d", sz);
        return (sz);
    }

#ifdef TMPDUMPDATA
    char          Buff[100];
    int static    GlobalCount = 0;
#endif

    for( num_planes = 0; num_planes < 2; num_planes++ ) {
        if( num_planes ) { //UV location
            buf_height = height / 2;
        } else { //Y location
            buf_height = height;
        }

        for( i = 0; i < buf_height; i++ ) {
            memcpy(input, pTmpBufferPos, width);

#ifdef TMPDUMPDATA

            if( GlobalCount < buf_height ) {
                DEBUG("[DCE_ENC_TEST] GlobalCount %d\n", GlobalCount);
                sprintf(Buff, "/sd/dce_enc_dump/dcefiledump%d.bin", GlobalCount);
                tmpDump = fopen(Buff, "wb+");
                if( tmpDump == NULL ) {
                    DEBUG("[DCE_ENC_TEST] Error opening /sd/dce_enc_dump/dcefiledump\n");
                } else {
                    GlobalCount++;
                    //DEBUG("Before Input [%p]\n", input);
                    fwrite(input, 1, width, tmpDump);

                    fflush(tmpDump);
                    fclose(tmpDump);
                    tmpDump = NULL;
                }
            }

#endif
            pTmpBufferPos = (int *) ((int)pTmpBufferPos + width);
            input = (char *) ((int)input + 4096);
            //DEBUG("After Input [%p]\n", input);

        }
    }

    free(pTmpBuffer);
    close(fd);

    DEBUG("sz=%d", sz);
    return (sz);
}

/* helper to write one frame of output */
int write_output(const char *pattern, int cnt, char *output, int bytesToWrite)
{
    int           sz = 0;
    const char   *path = get_path(pattern, cnt);
    int           fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);

    if( fd < 0 ) {
        ERROR("could open output file: %s (%d)", path, errno);
        return (0);
    }

    char   *p = output;
    sz = write(fd, p, bytesToWrite);

    DEBUG("Writing of bytesToWrite [%d] output sz %d \n", bytesToWrite, sz);

    close(fd);

    return (sz);
}

#ifdef PROFILE_TIME
/* for timing in microsecond */
uint64_t mark_microsecond(uint64_t *last)
{
    struct timespec    time;
    uint64_t           t1 = 0;

    clock_gettime(CLOCK_REALTIME, &time);
    t1 = timespec2nsec(&time);
    t1 = t1 / 1000;
    if( last ) {
        return (t1 - *last);
    }
    return (t1);
}

#endif

/* decoder body */
int main(int argc, char * *argv)
{
    Engine_Error    ec;
    XDAS_Int32      err;
    char           *output = NULL;
    int             output_size = 0;
    char           *in_pattern, *out_pattern;
    int             in_cnt = 0, out_cnt = 0;
    int             oned;
    InputBuffer    *buf = NULL;
    char            profile[10];
    int             profile_value;
    int             level;
    int             eof = 0;
    int             ivahd_encode_type;
    char            vid_codec[10];
    unsigned int    codec_switch = 0;
    int             bytesGenerated = 0;
    int             frames_to_write = 0;

#ifdef PROFILE_TIME
    uint64_t    init_start_time = 0;
    uint64_t    codec_process_time = 0;
    uint64_t    total_init_time = 0;
    uint64_t    input_alloc_time = 0;
#endif

#if 0
    int    loop = 0;

    while( loop == 0 ) {
        loop = 0;
    }

#endif

    if((argc >= 2) && !strcmp(argv[1], "-1")) {
        oned = TRUE;
        argc--;
        argv++;
    } else {
        oned = FALSE;
    }

    if( argc != 9 ) {
        printf("usage:   %s width height frames_to_write inpattern outpattern codec baseline/high level\n", argv[0]);
        printf("example: %s 1920 1088 300 in.yuv out.h264 h264 baseline 10\n", argv[0]);
        printf("example: %s 176 144 300 in.yuv out.m4v mpeg4 simple/baseline 0\n", argv[0]);
        printf("example: %s 176 144 300 in.yuv out.m4v h263 simple/baseline 0\n", argv[0]);
        printf("Currently supported codecs: h264 or mpeg4 or h263\n");
        printf("Run this command for help on the use case: use dce_enc_test\n");
        return (1);
    }

    width  = atoi(argv[1]);
    height = atoi(argv[2]);
    frames_to_write = atoi(argv[3]);
    in_pattern  = argv[4];
    out_pattern = argv[5];
    strcpy(vid_codec, argv[6]);
    strcpy(profile, argv[7]);
    level = atoi(argv[8]);

    printf("Selected codec: %s\n", vid_codec);

    enum {
        DCE_ENC_TEST_H264  = 1,
        DCE_ENC_TEST_MPEG4 = 2,
        DCE_ENC_TEST_H263 = 3
    };

    if((!(strcmp(vid_codec, "h264")))) {
        ivahd_encode_type = IVAHD_H264_ENCODE;
        codec_switch = DCE_ENC_TEST_H264;
        if((!(strcmp(profile, "baseline")))) {
            profile_value = IH264_BASELINE_PROFILE;
        } else if((!(strcmp(profile, "high")))) {
            profile_value = IH264_HIGH_PROFILE;
        } else {
            printf("Wrong profile value. Please use: baseline or high. See 'use dce_enc_test'.\n");
            return (1);
        }

        switch( level ) {
            case IH264_LEVEL_10 :
            case IH264_LEVEL_1b :
            case IH264_LEVEL_11 :
            case IH264_LEVEL_12 :
            case IH264_LEVEL_13 :
            case IH264_LEVEL_20 :
            case IH264_LEVEL_21 :
            case IH264_LEVEL_22 :
            case IH264_LEVEL_30 :
            case IH264_LEVEL_31 :
            case IH264_LEVEL_32 :
            case IH264_LEVEL_40 :
            case IH264_LEVEL_41 :
            case IH264_LEVEL_42 :
            case IH264_LEVEL_50 :
            case IH264_LEVEL_51 :
                printf("Acceptable H.264 level value = %d\n", level);
                break;
            default :
                printf("Wrong level value. Please use the correct level value for H.264. See 'use dce_enc_test'.\n");
                return (1);
        }
    } else if( !(strcmp(vid_codec, "mpeg4"))) {

        ivahd_encode_type = IVAHD_MPEG4_ENCODE;
        codec_switch = DCE_ENC_TEST_MPEG4;

        if((!(strcmp(profile, "simple")))) {
            profile_value = 3;
        } else {
            printf("Wrong profile value. Please use: simple. See 'use dce_enc_test'.\n");
            return (1);
        }

        switch( level ) {
            case IMPEG4ENC_SP_LEVEL_0 :
            case IMPEG4ENC_SP_LEVEL_0B :
            case IMPEG4ENC_SP_LEVEL_1 :
            case IMPEG4ENC_SP_LEVEL_2 :
            case IMPEG4ENC_SP_LEVEL_3 :
            case IMPEG4ENC_SP_LEVEL_4A :
            case IMPEG4ENC_SP_LEVEL_5 :
            case IMPEG4ENC_SP_LEVEL_6 :
                printf("Acceptable MPEG4 level value = %d\n", level);
                break;
            default :
                printf("Wrong level value. Please use the correct level value for MPEG4. See 'use dce_enc_test'.\n");
                return (1);
        }
    } else if( !(strcmp(vid_codec, "h263"))) {

        ivahd_encode_type = IVAHD_H263_ENCODE;
        codec_switch = DCE_ENC_TEST_H263;

        if((!(strcmp(profile, "simple")))) {
            profile_value = 3;
        } else {
            printf("Wrong profile value. Please use: simple. See 'use dce_enc_test'.\n");
            return (1);
        }

        switch( level ) {
            case IMPEG4ENC_H263_LEVEL_10 :
            case IMPEG4ENC_H263_LEVEL_20 :
            case IMPEG4ENC_H263_LEVEL_30 :
            case IMPEG4ENC_H263_LEVEL_40 :
            case IMPEG4ENC_H263_LEVEL_45 :
            case IMPEG4ENC_H263_LEVEL_50 :
            case IMPEG4ENC_H263_LEVEL_60 :
            case IMPEG4ENC_H263_LEVEL_70 :
                printf("Acceptable H263 level value = %d\n", level);
                break;
            default :
                printf("Wrong level value. Please use the correct level value for H263. See 'use dce_enc_test'.\n");
                return (1);
        }
    } else {
        printf("No valid codec entry. Please use: h264 or mpeg4 or h263\n");
        return (1);
    }

    DEBUG("width=%d, height=%d", width, height);

    /* output buffer parameters is TILER with width and height */
    width  = ALIGN2(width, 4);         /* round up to MB */
    height = ALIGN2(height, 1);        /* round up to MB */

    switch( codec_switch ) {
        case DCE_ENC_TEST_H264 :
        case DCE_ENC_TEST_MPEG4 :
        case DCE_ENC_TEST_H263 :
            num_buffers = 1;
            break;
        default :
            ERROR("Unrecognized codec to encode");
    }

    DEBUG("width=%d, height=%d, num_buffers=%d",
          width, height, num_buffers);

#ifdef PROFILE_TIME
    init_start_time = mark_microsecond(NULL);
#endif
    engine = Engine_open("ivahd_vidsvr", NULL, &ec);

    if( !engine ) {
        ERROR("fail");
        goto out;
    }

    DEBUG("Engine_open successful engine=%p", engine);

    DEBUG("input buffer configuration width %d height %d", width, height);
    inBufs = dce_alloc(sizeof(IVIDEO2_BufDesc));

    DEBUG("Input allocate through tiler");

#ifdef PROFILE_TIME
    uint64_t    alloc_time_start = mark_microsecond(NULL);
#endif

    inBufs->numPlanes = 2;
    inBufs->imageRegion.topLeft.x = 0;
    inBufs->imageRegion.topLeft.y = 0;
    inBufs->imageRegion.bottomRight.x = width;
    inBufs->imagePitch[0] = 4096;

    inBufs->topFieldFirstFlag = 0; //Only valid for interlace content.
    inBufs->contentType = IVIDEO_PROGRESSIVE;

    // NOTE: FOR INTERLACED.
    // For contentType = IVIDEO_INTERLACED
    // inBufs->contentType = IVIDEO_INTERLACED;
    // inBufs->dataLayout = IVIDEO_FIELD_SEPARATED;
    // if stereoInfoPreset == IH264_STEREOINFO_DISABLE then
    // inBufs->activeFrameRegion.bottomRight.y = height / 2;
    // inBufs->imageRegion.bottomRight.y = height / 2;

    inBufs->activeFrameRegion.topLeft.x = 0;
    inBufs->activeFrameRegion.topLeft.y = 0;
    inBufs->activeFrameRegion.bottomRight.x = width;
    inBufs->activeFrameRegion.bottomRight.y = height;

    inBufs->imageRegion.bottomRight.y = height;
    inBufs->chromaFormat = XDM_YUV_420SP;

    inBufs->secondFieldOffsetWidth[0] = 0;
    inBufs->secondFieldOffsetHeight[0] = 0;

    inBufs->planeDesc[0].memType = XDM_MEMTYPE_TILED8;
    inBufs->planeDesc[0].bufSize.tileMem.width  = width;
    inBufs->planeDesc[0].bufSize.tileMem.height = height;

    inBufs->imagePitch[1] = 4096;
    inBufs->secondFieldOffsetWidth[1] = 1;
    inBufs->secondFieldOffsetHeight[1] = 0;

    inBufs->planeDesc[1].memType = XDM_MEMTYPE_TILED16;
    inBufs->planeDesc[1].bufSize.tileMem.width  = width / 2; /* UV interleaved width is same a Y */
    inBufs->planeDesc[1].bufSize.tileMem.height = height / 2;

    buf = calloc(sizeof(InputBuffer), 1);
    DEBUG(" ----------------- create TILER buf 0x%x --------------------", (unsigned int)buf);
    buf->buf = tiler_alloc(width, height);
    if( buf->buf ) {
        //buf->y     = TilerMem_VirtToPhys(buf->buf);
        buf->y   = (SSPtr)buf->buf;
        //buf->uv  = TilerMem_VirtToPhys(buf->buf + (height * 4096));
        buf->uv  = (SSPtr)buf->buf + (height * 4096);

        DEBUG("buf=%p, buf->buf=%p y=%08x, uv=%08x", buf, buf->buf, buf->y, buf->uv);
    } else {
        ERROR(" ---------------- tiler_alloc failed --------------------");
        free(buf);
        goto shutdown;
    }

#ifdef PROFILE_TIME
    input_alloc_time = mark_microsecond(&alloc_time_start);
#endif

    DEBUG("input buffer configuration num_buffers %d width %d height %d", num_buffers, width, height);

    switch( codec_switch ) {
        case DCE_ENC_TEST_H264 :
            inArgs = dce_alloc(sizeof(IH264ENC_InArgs));
            inArgs->size = sizeof(IH264ENC_InArgs);

            outArgs = dce_alloc(sizeof(IH264ENC_OutArgs));
            outArgs->size = sizeof(IH264ENC_OutArgs);
            h264enc_outArgs = (IH264ENC_OutArgs *) outArgs;

            params = dce_alloc(sizeof(IH264ENC_Params));
            params->size = sizeof(IH264ENC_Params);
            break;
        case DCE_ENC_TEST_MPEG4 :
        case DCE_ENC_TEST_H263 :
            inArgs = dce_alloc(sizeof(IMPEG4ENC_InArgs));
            inArgs->size = sizeof(IMPEG4ENC_InArgs);

            outArgs = dce_alloc(sizeof(IMPEG4ENC_OutArgs));
            outArgs->size = sizeof (IMPEG4ENC_OutArgs);
            mpeg4enc_outArgs = (IMPEG4ENC_OutArgs *) outArgs;

            params = dce_alloc(sizeof(IMPEG4ENC_Params));
            params->size = sizeof(IMPEG4ENC_Params);
            break;
        default :
            ERROR("Unrecognized codec to encode");
    }

    params->encodingPreset = XDM_USER_DEFINED; //XDM_USER_DEFINED; //XDM_EncodingPreset
    params->rateControlPreset = IVIDEO_USER_DEFINED;
    params->maxHeight = height;
    params->maxWidth = width;
    params->dataEndianness = XDM_BYTE; //XDM_DataFormat
    params->maxBitRate = -1; //IGNORED
    params->minBitRate = 0;
    params->inputChromaFormat = XDM_YUV_420SP; //XDM_ChromaFormat
    params->inputContentType = IVIDEO_PROGRESSIVE; //IVIDEO_ContentType
    params->operatingMode = IVIDEO_ENCODE_ONLY; //IVIDEO_OperatingMode
    params->profile = profile_value;
    params->level = level;
    params->inputDataMode = IVIDEO_ENTIREFRAME; //IVIDEO_DataMode
    params->outputDataMode = IVIDEO_ENTIREFRAME; //IVIDEO_DataMode
    params->numInputDataUnits = 1;
    params->numOutputDataUnits = 1;
    params->metadataType[0] = IVIDEO_METADATAPLANE_NONE;
    params->metadataType[1] = IVIDEO_METADATAPLANE_NONE;
    params->metadataType[2] = IVIDEO_METADATAPLANE_NONE;

    DEBUG("dce_alloc VIDENC2_Params successful params=%p", params);

    switch( codec_switch ) {
        case DCE_ENC_TEST_H264 :
            DEBUG("H.264 Encoding with profile_value %d level %d", profile_value, level);
            params->maxInterFrameInterval = 1; //1,31 if IVIDEO_ContentType is IVIDEO_PROGRESSIVE

            //Miscellaneous
            h264enc_params = (IH264ENC_Params *) params;
            h264enc_params->interlaceCodingType = IH264_INTERLACE_DEFAULT;
            h264enc_params->bottomFieldIntra = 0;
            h264enc_params->gopStructure = IH264ENC_GOPSTRUCTURE_DEFAULT; // IH264ENC_GOPSTRUCTURE_NONUNIFORM
            h264enc_params->entropyCodingMode = IH264_ENTROPYCODING_DEFAULT; // IH264_ENTROPYCODING_CAVLC - BASE PROFILE
            h264enc_params->transformBlockSize = IH264_TRANSFORM_4x4; // BASE PROFILE
            h264enc_params->log2MaxFNumMinus4 = 10;
            h264enc_params->picOrderCountType = IH264_POC_TYPE_DEFAULT; // IH264_POC_TYPE_0
            h264enc_params->enableWatermark = 0;
            h264enc_params->IDRFrameInterval = 1;
            h264enc_params->pConstantMemory = NULL;
            h264enc_params->maxIntraFrameInterval = 0x7FFFFFFF;
            h264enc_params->debugTraceLevel = 0;
            h264enc_params->lastNFramesToLog = 0;
            h264enc_params->enableAnalyticinfo = 0;
            h264enc_params->enableGMVSei = 0;
            h264enc_params->constraintSetFlags = 20;
            h264enc_params->enableRCDO = 0;
            h264enc_params->enableLongTermRefFrame = IH264ENC_LTRP_NONE;
            h264enc_params->LTRPPeriod = 0;

            //H-P Coding Control Params
            h264enc_params->numTemporalLayer = IH264_TEMPORAL_LAYERS_1;
            h264enc_params->referencePicMarking = IH264_LONG_TERM_PICTURE;
            h264enc_params->reservedParams[0] = 0;
            h264enc_params->reservedParams[1] = 0;
            h264enc_params->reservedParams[2] = 0;

            //rate control params
            h264enc_params->rateControlParams.rateControlParamsPreset = IH264_RATECONTROLPARAMS_USERDEFINED;
            h264enc_params->rateControlParams.scalingMatrixPreset = IH264_SCALINGMATRIX_NONE;
            h264enc_params->rateControlParams.rcAlgo = IH264_RATECONTROL_DEFAULT; // 0
            h264enc_params->rateControlParams.qpI = 28;
            h264enc_params->rateControlParams.qpMaxI = 36;
            h264enc_params->rateControlParams.qpMinI = 10;
            h264enc_params->rateControlParams.qpP = 28;
            h264enc_params->rateControlParams.qpMaxP = 40;
            h264enc_params->rateControlParams.qpMinP = 10;
            h264enc_params->rateControlParams.qpOffsetB = 4;
            h264enc_params->rateControlParams.qpMaxB = 44;
            h264enc_params->rateControlParams.qpMinB = 10;
            h264enc_params->rateControlParams.allowFrameSkip = 0;
            h264enc_params->rateControlParams.removeExpensiveCoeff = 0;
            h264enc_params->rateControlParams.chromaQPIndexOffset = 0;
            h264enc_params->rateControlParams.IPQualityFactor = IH264_QUALITY_FACTOR_DEFAULT; // 0
            h264enc_params->rateControlParams.initialBufferLevel = 64000;
            h264enc_params->rateControlParams.HRDBufferSize = 64000;
            h264enc_params->rateControlParams.minPicSizeRatioI = 0;
            h264enc_params->rateControlParams.maxPicSizeRatioI = 20;
            h264enc_params->rateControlParams.minPicSizeRatioP = 0;
            h264enc_params->rateControlParams.maxPicSizeRatioP = 0;
            h264enc_params->rateControlParams.minPicSizeRatioB = 0;
            h264enc_params->rateControlParams.maxPicSizeRatioB = 0;
            h264enc_params->rateControlParams.enablePRC = 1;
            h264enc_params->rateControlParams.enablePartialFrameSkip = 0;
            h264enc_params->rateControlParams.discardSavedBits = 0;
            h264enc_params->rateControlParams.reserved = 0;
            h264enc_params->rateControlParams.VBRDuration = 8;
            h264enc_params->rateControlParams.VBRsensitivity = 0;
            h264enc_params->rateControlParams.skipDistributionWindowLength = 5;
            h264enc_params->rateControlParams.numSkipInDistributionWindow =1;
            h264enc_params->rateControlParams.enableHRDComplianceMode = 1;
            h264enc_params->rateControlParams.frameSkipThMulQ5 = 0;
            h264enc_params->rateControlParams.vbvUseLevelThQ5 = 0;
            h264enc_params->rateControlParams.reservedRC[0] = 0;
            h264enc_params->rateControlParams.reservedRC[1] = 0;
            h264enc_params->rateControlParams.reservedRC[2] = 0;

            //intercoding coding params
            h264enc_params->interCodingParams.interCodingPreset = IH264_INTERCODING_USERDEFINED;
            h264enc_params->interCodingParams.searchRangeHorP = 144;
            h264enc_params->interCodingParams.searchRangeVerP = 32;
            h264enc_params->interCodingParams.searchRangeHorB = 144;
            h264enc_params->interCodingParams.searchRangeVerB = 16;
            h264enc_params->interCodingParams.interCodingBias = IH264_BIASFACTOR_DEFAULT;
            h264enc_params->interCodingParams.skipMVCodingBias = IH264_BIASFACTOR_MILD;
            h264enc_params->interCodingParams.minBlockSizeP = IH264_BLOCKSIZE_8x8;
            h264enc_params->interCodingParams.minBlockSizeB = IH264_BLOCKSIZE_8x8;
            h264enc_params->interCodingParams.meAlgoMode = IH264ENC_MOTIONESTMODE_DEFAULT;

            //intra coding params.
            h264enc_params->intraCodingParams.intraCodingPreset = IH264_INTRACODING_DEFAULT;
            h264enc_params->intraCodingParams.lumaIntra4x4Enable = 0;
            h264enc_params->intraCodingParams.lumaIntra8x8Enable = 0x0FF;
            h264enc_params->intraCodingParams.lumaIntra16x16Enable = 0;  // BASE PROFILE
            h264enc_params->intraCodingParams.chromaIntra8x8Enable = 0;  // BASE PROFILE
            h264enc_params->intraCodingParams.chromaComponentEnable = IH264_CHROMA_COMPONENT_CB_CR_BOTH;  // BASE PROFILE
            h264enc_params->intraCodingParams.intraRefreshMethod = IH264_INTRAREFRESH_DEFAULT;
            h264enc_params->intraCodingParams.intraRefreshRate = 0;
            h264enc_params->intraCodingParams.gdrOverlapRowsBtwFrames = 0;
            h264enc_params->intraCodingParams.constrainedIntraPredEnable = 0;
            h264enc_params->intraCodingParams.intraCodingBias = IH264ENC_INTRACODINGBIAS_DEFAULT;

            //NALU Control Params.
            h264enc_params->nalUnitControlParams.naluControlPreset = IH264_NALU_CONTROL_USERDEFINED;
            h264enc_params->nalUnitControlParams.naluPresentMaskStartOfSequence = 0x01A0; // 416
            h264enc_params->nalUnitControlParams.naluPresentMaskIDRPicture = 0x0020; //32
            h264enc_params->nalUnitControlParams.naluPresentMaskIntraPicture = 2;
            h264enc_params->nalUnitControlParams.naluPresentMaskNonIntraPicture = 2;
            h264enc_params->nalUnitControlParams.naluPresentMaskEndOfSequence = 0x0C00; // 3072

            //Slice coding params
            h264enc_params->sliceCodingParams.sliceCodingPreset = IH264_SLICECODING_DEFAULT;
            h264enc_params->sliceCodingParams.sliceMode = IH264_SLICEMODE_DEFAULT;
            h264enc_params->sliceCodingParams.sliceUnitSize = 0;
            h264enc_params->sliceCodingParams.sliceStartOffset[0] = 0;
            h264enc_params->sliceCodingParams.sliceStartOffset[1] = 0;
            h264enc_params->sliceCodingParams.sliceStartOffset[2] = 0;
            h264enc_params->sliceCodingParams.streamFormat = IH264_STREAM_FORMAT_DEFAULT;

            //Loop Filter Params
            h264enc_params->loopFilterParams.loopfilterPreset = IH264_LOOPFILTER_DEFAULT;
            h264enc_params->loopFilterParams.loopfilterDisableIDC = IH264_DISABLE_FILTER_DEFAULT;
            h264enc_params->loopFilterParams.filterOffsetA = 0;
            h264enc_params->loopFilterParams.filterOffsetB = 0;

            //fmo coding params
            h264enc_params->fmoCodingParams.fmoCodingPreset = IH264_FMOCODING_DEFAULT;
            h264enc_params->fmoCodingParams.numSliceGroups = 1;
            h264enc_params->fmoCodingParams.sliceGroupMapType = IH264_SLICE_GRP_MAP_DEFAULT; // 4
            h264enc_params->fmoCodingParams.sliceGroupChangeDirectionFlag = IH264ENC_SLICEGROUP_CHANGE_DIRECTION_DEFAULT;
            h264enc_params->fmoCodingParams.sliceGroupChangeRate = 0;
            h264enc_params->fmoCodingParams.sliceGroupChangeCycle = 0;
            h264enc_params->fmoCodingParams.sliceGroupParams[0] = 0;
            h264enc_params->fmoCodingParams.sliceGroupParams[1] = 0;

            //VUI Control Params
            h264enc_params->vuiCodingParams.vuiCodingPreset = IH264_VUICODING_DEFAULT;
            h264enc_params->vuiCodingParams.aspectRatioInfoPresentFlag = 0;
            h264enc_params->vuiCodingParams.aspectRatioIdc = 0;
            h264enc_params->vuiCodingParams.videoSignalTypePresentFlag = 0;
            h264enc_params->vuiCodingParams.videoFormat = IH264ENC_VIDEOFORMAT_NTSC;
            h264enc_params->vuiCodingParams.videoFullRangeFlag = 0;
            h264enc_params->vuiCodingParams.timingInfoPresentFlag = 0;
            h264enc_params->vuiCodingParams.hrdParamsPresentFlag = 0;
            h264enc_params->vuiCodingParams.numUnitsInTicks= 1000;

            //Stereo Info Control Params
            h264enc_params->stereoInfoParams.stereoInfoPreset = IH264_STEREOINFO_DISABLE;
            h264enc_params->stereoInfoParams.topFieldIsLeftViewFlag = 1;
            h264enc_params->stereoInfoParams.viewSelfContainedFlag = 0;

            //Frame Packing SEI Params
            h264enc_params->framePackingSEIParams.framePackingPreset = IH264_FRAMEPACK_SEI_DISABLE;
            h264enc_params->framePackingSEIParams.framePackingType = IH264_FRAMEPACK_TYPE_DEFAULT;
            h264enc_params->framePackingSEIParams.frame0PositionX = 0;
            h264enc_params->framePackingSEIParams.frame0PositionY = 0;
            h264enc_params->framePackingSEIParams.frame1PositionX = 0;
            h264enc_params->framePackingSEIParams.frame1PositionY = 0;
            h264enc_params->framePackingSEIParams.reservedByte = 0;

            //SVC coding params
            h264enc_params->svcCodingParams.svcExtensionFlag = IH264_SVC_EXTENSION_FLAG_DISABLE;
            h264enc_params->svcCodingParams.dependencyID = 0;
            h264enc_params->svcCodingParams.qualityID = 0;
            h264enc_params->svcCodingParams.enhancementProfileID = 0;
            h264enc_params->svcCodingParams.layerIndex = 0;
            h264enc_params->svcCodingParams.refLayerDQId = 0;

            DEBUG("dce_alloc VIDENC2_Params successful h264enc_params=%p", h264enc_params);

            codec = VIDENC2_create(engine, "ivahd_h264enc", (VIDENC2_Params *)h264enc_params);
            break;

        case DCE_ENC_TEST_MPEG4 :
        case DCE_ENC_TEST_H263 :
            params->maxInterFrameInterval = 0;

            mpeg4enc_params = (IMPEG4ENC_Params *) params;

            mpeg4enc_params->useDataPartitioning = 0;
            mpeg4enc_params->useRvlc = 0;
            if( codec_switch == DCE_ENC_TEST_H263 ) {
                mpeg4enc_params->useShortVideoHeader = 1;
            } else {
                mpeg4enc_params->useShortVideoHeader = 0;
            }
            mpeg4enc_params->vopTimeIncrementResolution = 30;
            mpeg4enc_params->nonMultiple16RefPadMethod = IMPEG4_PAD_METHOD_MPEG4;
            mpeg4enc_params->pixelRange = IMPEG4ENC_PR_0_255;
            mpeg4enc_params->enableSceneChangeAlgo = IMPEG4ENC_SCDA_DISABLE;
            mpeg4enc_params->useVOS = 0;
            mpeg4enc_params->enableMONA = 0;
            mpeg4enc_params->enableAnalyticinfo = 0;
            mpeg4enc_params->debugTraceLevel = 0;
            mpeg4enc_params->lastNFramesToLog = 0;

            // IMPEG4ENC_RateControlParams
            mpeg4enc_params->rateControlParams.rateControlParamsPreset = IMPEG4_RATECONTROLPARAMS_DEFAULT;
            mpeg4enc_params->rateControlParams.rcAlgo = IMPEG4_RATECONTROLALGO_VBR;
            mpeg4enc_params->rateControlParams.qpI = 5;
            mpeg4enc_params->rateControlParams.qpP = 5;
            mpeg4enc_params->rateControlParams.seIntialQP = 5;
            mpeg4enc_params->rateControlParams.qpMax = 31;
            mpeg4enc_params->rateControlParams.qpMin = 1;
            mpeg4enc_params->rateControlParams.enablePerceptualQuantMode = 0;
            mpeg4enc_params->rateControlParams.allowFrameSkip = 0;
            mpeg4enc_params->rateControlParams.initialBufferLevel = 0;
            mpeg4enc_params->rateControlParams.vbvBufferSize = 0;
            mpeg4enc_params->rateControlParams.qpMinIntra = 0;

            // IMPEG4ENC_InterCodingParams
            mpeg4enc_params->interCodingParams.interCodingPreset = IMPEG4_INTERCODING_DEFAULT;
            mpeg4enc_params->interCodingParams.searchRangeHorP = 144;
            mpeg4enc_params->interCodingParams.searchRangeVerP = 32;
            mpeg4enc_params->interCodingParams.globalOffsetME = 1;
            mpeg4enc_params->interCodingParams.earlySkipThreshold = 200;
            mpeg4enc_params->interCodingParams.enableThresholdingMethod = 1;
            mpeg4enc_params->interCodingParams.minBlockSizeP = IMPEG4_BLOCKSIZE_8x8;
            mpeg4enc_params->interCodingParams.enableRoundingControl = 1;

            // IMPEG4ENC_IntraCodingParams
            mpeg4enc_params->intraCodingParams.intraCodingPreset = IMPEG4_INTRACODING_DEFAULT;
            mpeg4enc_params->intraCodingParams.intraRefreshMethod = 0;
            mpeg4enc_params->intraCodingParams.intraRefreshRate = 0;
            mpeg4enc_params->intraCodingParams.acpredEnable = 1;
            mpeg4enc_params->intraCodingParams.insertGOVHdrBeforeIframe = 0;
            mpeg4enc_params->intraCodingParams.enableDriftControl = 1;

            // IMPEG4ENC_sliceCodingParams
            mpeg4enc_params->sliceCodingParams.sliceCodingPreset = IMPEG4_SLICECODING_DEFAULT;
            mpeg4enc_params->sliceCodingParams.sliceMode = IMPEG4_SLICEMODE_NONE;
            mpeg4enc_params->sliceCodingParams.sliceUnitSize = 0;
            mpeg4enc_params->sliceCodingParams.gobInterval = 0;
            mpeg4enc_params->sliceCodingParams.useHec = 0;

            DEBUG("dce_alloc VIDENC2_Params successful mpeg4enc_params=%p", mpeg4enc_params);

            codec = VIDENC2_create(engine, "ivahd_mpeg4enc", (VIDENC2_Params *)mpeg4enc_params);
            break;
        default :
            ERROR("Unrecognized codec to encode");
    }

    if( !codec ) {
        ERROR("fail");
        goto out;
    }

    DEBUG("VIDENC2_create successful codec=%p", codec);

    switch( codec_switch ) {
        case DCE_ENC_TEST_H264 :
            dynParams = dce_alloc(sizeof(IH264ENC_DynamicParams));
            dynParams->size = sizeof(IH264ENC_DynamicParams);
            DEBUG("dce_alloc dynParams successful dynParams=%p size=%d", dynParams, dynParams->size);

            break;
        case DCE_ENC_TEST_MPEG4 :
        case DCE_ENC_TEST_H263 :
            dynParams = dce_alloc(sizeof(IMPEG4ENC_DynamicParams));
            dynParams->size = sizeof(IMPEG4ENC_DynamicParams);
            break;
        default :
            ERROR("Unrecognized codec to encode");
    }

    dynParams->inputHeight  = height;
    dynParams->inputWidth  = width;
    dynParams->refFrameRate = 15000; // refFrameRate in fps * 1000
    dynParams->targetFrameRate= 15000; // Target frame rate in fps * 1000
    dynParams->targetBitRate = 64000;
    dynParams->intraFrameInterval = 15; //Only 1st frame to be intra frame (I-frame)
    dynParams->generateHeader = XDM_ENCODE_AU;
    dynParams->captureWidth = width;
    dynParams->forceFrame = IVIDEO_NA_FRAME;
    dynParams->sampleAspectRatioHeight = 1;
    dynParams->sampleAspectRatioWidth = 1;
    dynParams->ignoreOutbufSizeFlag = XDAS_FALSE;  // If this is XDAS_TRUE then getBufferFxn and getBufferHandle needs to be set.
    dynParams->putDataFxn = NULL;
    dynParams->putDataHandle = NULL;
    dynParams->getDataFxn = NULL;
    dynParams->getDataHandle = NULL;
    dynParams->getBufferFxn = NULL;
    dynParams->getBufferHandle = NULL;
    dynParams->lateAcquireArg = -1;

    switch( codec_switch ) {
        case DCE_ENC_TEST_H264 :
            dynParams->interFrameInterval = 1; // 2 B frames
            dynParams->mvAccuracy = IVIDENC2_MOTIONVECTOR_QUARTERPEL; //IVIDENC2_MotionVectorAccuracy

            DEBUG("dce_alloc IH264ENC_DynamicParams successful size %d dynParams=%p", dynParams->size, dynParams);
            h264enc_dynParams = (IH264ENC_DynamicParams *) dynParams;

            h264enc_dynParams->sliceGroupChangeCycle = 0;
            h264enc_dynParams->searchCenter.x = 0x7FFF; // or 32767
            h264enc_dynParams->searchCenter.y = 0x7FFF; // or 32767
            h264enc_dynParams->enableStaticMBCount = 0;
            h264enc_dynParams->enableROI = 0;
            h264enc_dynParams->reservedDynParams[0] = 0;
            h264enc_dynParams->reservedDynParams[1] = 0;
            h264enc_dynParams->reservedDynParams[2] = 0;

            //Rate Control Params
            h264enc_dynParams->rateControlParams.rateControlParamsPreset = IH264_RATECONTROLPARAMS_EXISTING;
            h264enc_dynParams->rateControlParams.scalingMatrixPreset = IH264_SCALINGMATRIX_NONE;
            h264enc_dynParams->rateControlParams.rcAlgo = IH264_RATECONTROL_DEFAULT;
            h264enc_dynParams->rateControlParams.qpI = 28;
            h264enc_dynParams->rateControlParams.qpMaxI = 36;
            h264enc_dynParams->rateControlParams.qpMinI = 10;
            h264enc_dynParams->rateControlParams.qpP = 28;
            h264enc_dynParams->rateControlParams.qpMaxP = 40;
            h264enc_dynParams->rateControlParams.qpMinP = 10;
            h264enc_dynParams->rateControlParams.qpOffsetB = 4;
            h264enc_dynParams->rateControlParams.qpMaxB = 44;
            h264enc_dynParams->rateControlParams.qpMinB = 10;
            h264enc_dynParams->rateControlParams.allowFrameSkip = 0;
            h264enc_dynParams->rateControlParams.removeExpensiveCoeff = 0;
            h264enc_dynParams->rateControlParams.IPQualityFactor = IH264_QUALITY_FACTOR_DEFAULT;
            h264enc_dynParams->rateControlParams.chromaQPIndexOffset = 0;
            h264enc_dynParams->rateControlParams.initialBufferLevel = 64000;
            h264enc_dynParams->rateControlParams.HRDBufferSize = 64000;
            h264enc_dynParams->rateControlParams.enablePartialFrameSkip = 0;
            h264enc_dynParams->rateControlParams.minPicSizeRatioI = 0;
            h264enc_dynParams->rateControlParams.maxPicSizeRatioI = 20;
            h264enc_dynParams->rateControlParams.minPicSizeRatioP = 0;
            h264enc_dynParams->rateControlParams.maxPicSizeRatioP = 0;
            h264enc_dynParams->rateControlParams.minPicSizeRatioB = 0;
            h264enc_dynParams->rateControlParams.maxPicSizeRatioB = 0;
            h264enc_dynParams->rateControlParams.enablePRC = 1;
            h264enc_dynParams->rateControlParams.enableHRDComplianceMode = 0;
            h264enc_dynParams->rateControlParams.reserved = 0;
            h264enc_dynParams->rateControlParams.VBRDuration = 8;
            h264enc_dynParams->rateControlParams.VBRsensitivity = 0;
            h264enc_dynParams->rateControlParams.skipDistributionWindowLength = 5;
            h264enc_dynParams->rateControlParams.numSkipInDistributionWindow = 1;
            h264enc_dynParams->rateControlParams.enableHRDComplianceMode = 1;
            h264enc_dynParams->rateControlParams.frameSkipThMulQ5 = 0;
            h264enc_dynParams->rateControlParams.vbvUseLevelThQ5 = 0;
            h264enc_dynParams->rateControlParams.reservedRC[0] = 0;
            h264enc_dynParams->rateControlParams.reservedRC[1] = 0;
            h264enc_dynParams->rateControlParams.reservedRC[2] = 0;

            //Inter Coding Params
            h264enc_dynParams->interCodingParams.interCodingPreset = IH264_INTERCODING_EXISTING;
            h264enc_dynParams->interCodingParams.searchRangeHorP = 144;
            h264enc_dynParams->interCodingParams.searchRangeVerP = 32;
            h264enc_dynParams->interCodingParams.searchRangeHorB = 144;
            h264enc_dynParams->interCodingParams.searchRangeVerB = 16;
            h264enc_dynParams->interCodingParams.interCodingBias= IH264_BIASFACTOR_DEFAULT;
            h264enc_dynParams->interCodingParams.skipMVCodingBias = IH264_BIASFACTOR_MILD;
            h264enc_dynParams->interCodingParams.minBlockSizeP = IH264_BLOCKSIZE_8x8;
            h264enc_dynParams->interCodingParams.minBlockSizeB = IH264_BLOCKSIZE_8x8;
            h264enc_dynParams->interCodingParams.meAlgoMode = IH264ENC_MOTIONESTMODE_DEFAULT;

            //Intra Coding Params
            h264enc_dynParams->intraCodingParams.intraCodingPreset = IH264_INTRACODING_EXISTING;
            h264enc_dynParams->intraCodingParams.lumaIntra4x4Enable = 0xFF; // or 255 BASE PROFILE
            h264enc_dynParams->intraCodingParams.lumaIntra8x8Enable = 0; // BASE PROFILE
            h264enc_dynParams->intraCodingParams.lumaIntra16x16Enable = 0;
            h264enc_dynParams->intraCodingParams.chromaIntra8x8Enable = 0;
            h264enc_dynParams->intraCodingParams.chromaComponentEnable = IH264_CHROMA_COMPONENT_CB_CR_BOTH;
            h264enc_dynParams->intraCodingParams.intraRefreshMethod = IH264_INTRAREFRESH_DEFAULT;
            h264enc_dynParams->intraCodingParams.intraRefreshRate = 0;
            h264enc_dynParams->intraCodingParams.gdrOverlapRowsBtwFrames = 0;
            h264enc_dynParams->intraCodingParams.constrainedIntraPredEnable = 0;
            h264enc_dynParams->intraCodingParams.intraCodingBias = IH264ENC_INTRACODINGBIAS_DEFAULT;

            //Slice Coding Params
            h264enc_dynParams->sliceCodingParams.sliceCodingPreset = IH264_SLICECODING_EXISTING;
            h264enc_dynParams->sliceCodingParams.sliceMode = IH264_SLICEMODE_DEFAULT;
            h264enc_dynParams->sliceCodingParams.sliceUnitSize = 0;
            h264enc_dynParams->sliceCodingParams.sliceStartOffset[0] = 0;
            h264enc_dynParams->sliceCodingParams.sliceStartOffset[1] = 0;
            h264enc_dynParams->sliceCodingParams.sliceStartOffset[2] = 0;
            h264enc_dynParams->sliceCodingParams.streamFormat = IH264_STREAM_FORMAT_DEFAULT;

            status = dce_alloc(sizeof(IH264ENC_Status));
            status->size = sizeof(IH264ENC_Status);
            DEBUG("dce_alloc IH264ENC_Status successful status=%p", status);

            h264enc_status = (IH264ENC_Status *) status;
            err = VIDENC2_control(codec, XDM_SETPARAMS, (VIDENC2_DynamicParams *) h264enc_dynParams, (VIDENC2_Status *) h264enc_status);
            DEBUG("dce_alloc IH264ENC_Status successful h264enc_status=%p", h264enc_status);
            break;

        case DCE_ENC_TEST_MPEG4 :
        case DCE_ENC_TEST_H263 :
            dynParams->interFrameInterval = 0;
            dynParams->mvAccuracy = IVIDENC2_MOTIONVECTOR_HALFPEL; //IVIDENC2_MotionVectorAccuracy

            DEBUG("dce_alloc IMPEG4ENC_DynamicParams successful size %d dynParams=%p", dynParams->size, dynParams);
            mpeg4enc_dynParams = (IMPEG4ENC_DynamicParams *) dynParams;

            mpeg4enc_dynParams->aspectRatioIdc = IMPEG4ENC_ASPECTRATIO_SQUARE;

            // IMPEG4ENC_RateControlParams
            memcpy(&mpeg4enc_dynParams->rateControlParams, &mpeg4enc_params->rateControlParams, sizeof(IMPEG4ENC_RateControlParams));
            // IMPEG4ENC_InterCodingParams
            memcpy(&mpeg4enc_dynParams->interCodingParams, &mpeg4enc_params->interCodingParams, sizeof(IMPEG4ENC_InterCodingParams));
            // IMPEG4ENC_sliceCodingParams
            memcpy(&mpeg4enc_dynParams->sliceCodingParams, &mpeg4enc_params->sliceCodingParams, sizeof(IMPEG4ENC_sliceCodingParams));

            status = dce_alloc(sizeof(IMPEG4ENC_Status));
            status->size = sizeof(IMPEG4ENC_Status);
            DEBUG("dce_alloc IMPEG4ENC_Status successful status=%p", status);

            mpeg4enc_status = (IMPEG4ENC_Status *) status;
            DEBUG("dce_alloc IMPEG4ENC_Status successful mpeg4enc_status=%p", mpeg4enc_status);
            err = VIDENC2_control(codec, XDM_SETPARAMS, (VIDENC2_DynamicParams *) mpeg4enc_dynParams, (VIDENC2_Status *) mpeg4enc_status);
            ERROR("Codec_control returned err=%d, extendedError=%08x", err, mpeg4enc_status->videnc2Status.extendedError);
            break;
        default :
            ERROR("Unrecognized codec to encode");
    }

    if( err ) {
        ERROR("fail: %d", err);
        goto shutdown;
    }

    DEBUG("VIDENC2_control XDM_SETPARAMS successful");

    // XDM_GETBUFINFO
    // Send Control cmd XDM_GETBUFINFO to get min output and output size
    err = VIDENC2_control(codec, XDM_GETBUFINFO, dynParams, status);
    DEBUG("VIDENC2_control - XDM_GETBUFINFO err %d status numOutBuf %d OutBufSize %d", err, status->bufInfo.minNumOutBufs, status->bufInfo.minOutBufSize[0].bytes);

    outBufs = dce_alloc(sizeof(XDM2_BufDesc));
    output_size = status->bufInfo.minOutBufSize[0].bytes;

    DEBUG("Output allocate through tiler 1D");
    if( codec_switch == DCE_ENC_TEST_H264 ) {
        outBufs->numBufs = status->bufInfo.minNumOutBufs;          // this value is 1
    } else if( codec_switch == DCE_ENC_TEST_MPEG4 || codec_switch == DCE_ENC_TEST_H263 ) {
        outBufs->numBufs = 1;
    }
    output = tiler_alloc(output_size, 0);
#ifdef NON_TILER_OUTPUT
    output = output_allocate_nonTiler(output_size);
#endif
    //outBufs->descs[0].buf = (XDAS_Int8 *)TilerMem_VirtToPhys(output);
    outBufs->descs[0].buf = (XDAS_Int8 *)output;

    outBufs->descs[0].memType = XDM_MEMTYPE_RAW;
    outBufs->descs[0].bufSize.bytes = output_size;

    DEBUG("outBufs->descs[0].buf %p output %p", outBufs->descs[0].buf, output);

#ifdef DUMPINPUTDATA
    char          Buff1[100];
    int static    GlobalCount1 = 0;
#endif

#ifdef PROFILE_TIME
    total_init_time = (uint64_t)mark_microsecond(&init_start_time);
    INFO("total_init_time %llu output_alloc_time %llu actual init time in: %lld us", total_init_time, output_alloc_time, total_init_time  - output_alloc_time);
#endif

    // Handling codec_config
    while( inBufs->numPlanes && outBufs->numBufs ) {
        int    n;
        DEBUG("Looping on reading input inBufs->numPlanes %d outBufs->numBufs %d", inBufs->numPlanes, outBufs->numBufs);

        //Read the NV12 frame to input buffer to be encoded.
        n = read_input(in_pattern, in_cnt, buf->buf);

#ifdef DUMPINPUTDATA0
        DEBUG("input data buf->buf[%p]", buf->buf);

        //Dump the file
        if( inputDump == NULL ) {
            inputDump = fopen("/sd/dce_enc_dump/inputdump.yuv", "ab");
            //DEBUG("input data dump file open %p errno %d", inputDump, errno);
            if( inputDump == NULL ) {
                DEBUG("Opening input Dump /sd/dce_enc_dump/inputdump.yuv file FAILED");
            }
        }
        //DEBUG("input data dump file open %p Successful", inputDump);

        fwrite(buf->buf, sizeof(char), 4096 * height * 3 / 2, inputDump);
        DEBUG("Dumping input file of NV12 format with read data of %d inside buffersize = %d", n, 4096 * height * 3 / 2);
        fflush(inputDump);
        fclose(inputDump);
        inputDump = NULL;
#endif

        if( n && (n != -1)) {
            eof = 0;
            inBufs->planeDesc[0].buf = (XDAS_Int8 *)buf->y;
            inBufs->planeDesc[1].buf = (XDAS_Int8 *)buf->uv;
            DEBUG("inBufs->planeDesc[0].buf %p inBufs->planeDesc[1].buf %p", inBufs->planeDesc[0].buf, inBufs->planeDesc[1].buf);
            DEBUG("push: %d (plane[0]= %d + plane[1]= %d = %d bytes) (%p)", in_cnt, inBufs->planeDesc[0].bufSize.bytes, inBufs->planeDesc[1].bufSize.bytes, n, buf);
            in_cnt++;

            /*
                       * Input buffer has data to be encoded.
                       */
            inArgs->inputID = (XDAS_Int32)buf;
            if( codec_switch == DCE_ENC_TEST_H264 ) {
                h264enc_inArgs = (IH264ENC_InArgs *) inArgs;
                DEBUG("TEST inArgs->inputID %d h264enc_inArgs->videnc2InArgs.inputID %d", inArgs->inputID, h264enc_inArgs->videnc2InArgs.inputID);
            } else if( codec_switch == DCE_ENC_TEST_MPEG4 || codec_switch == DCE_ENC_TEST_H263 ) {
                mpeg4enc_inArgs = (IMPEG4ENC_InArgs *) inArgs;
                DEBUG("TEST inArgs->inputID %d mpeg4enc_inArgs->videnc2InArgs.inputID %d", inArgs->inputID, mpeg4enc_inArgs->videnc2InArgs.inputID);
            }
        } else if( n == -1 ) {

            // Set EOF as 1 to ensure flush completes
            eof = 1;
            in_cnt++;

            DEBUG("n == -1 - go to shutdown");

            goto shutdown;

            switch( codec_switch ) {
                case DCE_ENC_TEST_H264 :
                    DEBUG("Calling VIDENC2_control XDM_FLUSH h264enc_dynParams %p h264enc_status %p", h264enc_dynParams, h264enc_status);
                    err = VIDENC2_control(codec, XDM_FLUSH, (VIDENC2_DynamicParams *) h264enc_dynParams, (VIDENC2_Status *) h264enc_status);
                    break;
                case DCE_ENC_TEST_MPEG4 :
                case DCE_ENC_TEST_H263 :
                    DEBUG("Calling VIDENC2_control XDM_FLUSH mpeg4enc_dynParams %p mpeg4enc_status %p", mpeg4enc_dynParams, mpeg4enc_status);
                    err = VIDENC2_control(codec, XDM_FLUSH, (VIDENC2_DynamicParams *) mpeg4enc_dynParams, (VIDENC2_Status *) mpeg4enc_status);
                    break;
                default :
                    ERROR("Unrecognized codec to encode");
            }

            /* We have sent the XDM_FLUSH, call VIDENC2_process until we get
                       * an error of XDM_EFAIL which tells us there are no more buffers
                       * at codec level.
                       */

            inArgs->inputID = 0;
            if( codec_switch == DCE_ENC_TEST_H264 ) {
                h264enc_inArgs = (IH264ENC_InArgs *) inArgs;
            } else if( codec_switch == DCE_ENC_TEST_MPEG4 || codec_switch == DCE_ENC_TEST_H263 ) {
                mpeg4enc_inArgs = (IMPEG4ENC_InArgs *) inArgs;
            }
            inBufs->planeDesc[0].bufSize.bytes = 0;
            inBufs->planeDesc[1].bufSize.bytes = 0;
            outBufs->descs[0].buf = NULL;
            outBufs->descs[1].buf = NULL;
        } else {
            /* end of input..  (n == 0) */
            inBufs->numPlanes = 0;
            eof = 1;
            DEBUG("n == 0 - go to shutdown");

            goto shutdown;

            switch( codec_switch ) {
                case DCE_ENC_TEST_H264 :
                    DEBUG("Calling VIDENC2_control XDM_FLUSH h264enc_dynParams %p h264enc_status %p", h264enc_dynParams, h264enc_status);
                    err = VIDENC2_control(codec, XDM_FLUSH, (VIDENC2_DynamicParams *) h264enc_dynParams, (VIDENC2_Status *) h264enc_status);
                    break;
                case DCE_ENC_TEST_MPEG4 :
                case DCE_ENC_TEST_H263 :
                    DEBUG("Calling VIDENC2_control XDM_FLUSH mpeg4enc_dynParams %p mpeg4enc_status %p", mpeg4enc_dynParams, mpeg4enc_status);
                    err = VIDENC2_control(codec, XDM_FLUSH, (VIDENC2_DynamicParams *) mpeg4enc_dynParams, (VIDENC2_Status *) mpeg4enc_status);
                    break;
                default :
                    ERROR("Unrecognized codec to encode");
            }

            /* We have sent the XDM_FLUSH, call VIDENC2_process until we get
                       * an error of XDM_EFAIL which tells us there are no more buffers
                       * at codec level.
                       */

            inArgs->inputID = 0;
            if( codec_switch == DCE_ENC_TEST_H264 ) {
                h264enc_inArgs = (IH264ENC_InArgs *) inArgs;
            } else if( codec_switch == DCE_ENC_TEST_MPEG4 || codec_switch == DCE_ENC_TEST_H263 ) {
                mpeg4enc_inArgs = (IMPEG4ENC_InArgs *) inArgs;
            }
            inBufs->planeDesc[0].bufSize.bytes = 0;
            inBufs->planeDesc[1].bufSize.bytes = 0;
            outBufs->descs[0].buf = NULL;
            outBufs->descs[1].buf = NULL;
        }

#ifdef DUMPINPUTDATA
        DEBUG("input data buf->buf[%p]", buf->buf);

        //Dump the file
        if( inputDump == NULL ) {

            if( GlobalCount1 <= 50 ) {
                DEBUG("[DCE_ENC_TEST] GlobalCount1 %d\n", GlobalCount1);
                sprintf(Buff1, "/sd/dce_enc_dump/dceinputdump%d.bin", GlobalCount1);
                inputDump = fopen(Buff1, "wb+");
                //DEBUG("input data dump file open %p errno %d", inputDump, errno);
                if( inputDump == NULL ) {
                    DEBUG("Opening input Dump /sd/dce_enc_dump/dceinputdump.bin file FAILED");
                } else {
                    GlobalCount1++;
                    //DEBUG("Before Input [%p]\n", input);
                    fwrite(buf->buf, 1, 4096 * height * 3 / 2, inputDump);
                    DEBUG("Dumping input file of NV12 format with read data of %d inside buffersize = %d", n, 4096 * height * 3 / 2);

                    fflush(inputDump);
                    fclose(inputDump);
                    inputDump = NULL;
                }
            }
            //DEBUG("input data dump file open %p Successful", inputDump);
        }
#endif

        int    iters = 0;

        do {

#ifdef H264_DEBUG

            DEBUG(">> Going to call VIDENC2_process codec=%p, inBufs=%p, outBufs=%p, h264enc_inArgs=%p, h264enc_outArgs=%p",
                  codec, inBufs, outBufs, h264enc_inArgs, h264enc_outArgs);

            DEBUG("Calling VIDENC2_process inArgs->inputID=%x inBufs->descs[0].buf %p inBufs->descs[0].bufSize %d",
                  inArgs->inputID, inBufs->planeDesc[0].buf, (int) inBufs->planeDesc[0].bufSize.bytes);

            DEBUG("Calling VIDENC2_process inArgs->inputID=%x inBufs->descs[1].buf %p inBufs->descs[1].bufSize %d",
                  inArgs->inputID, inBufs->planeDesc[1].buf, (int) inBufs->planeDesc[1].bufSize.bytes);

            DEBUG("h264enc_inArgs->videnc2InArgs.size %d inputID %d control %d ", h264enc_inArgs->videnc2InArgs.size,
                  h264enc_inArgs->videnc2InArgs.inputID, h264enc_inArgs->videnc2InArgs.control);
#endif

#ifdef PROFILE_TIME
            codec_process_time = mark_microsecond(NULL);
#endif

#if 0
            err = VIDENC2_process(codec, inBufs, outBufs, inArgs, outArgs);
#endif

            if( codec_switch == DCE_ENC_TEST_H264 ) {
                err = VIDENC2_process(codec, inBufs, outBufs, (VIDENC2_InArgs *) h264enc_inArgs, (VIDENC2_OutArgs *) h264enc_outArgs);
                DEBUG("[DCE_ENC_TEST] VIDENC2_process - err %d", err);

                if( err < 0 ) {
                    int    i = 0;

                    for( i=0; i < IH264ENC_EXTERROR_NUM_MAXWORDS; i++ ) {
                        DEBUG("DETAIL EXTENDED ERROR h264enc_outArgs->extErrorCode[%d]=%08x", i, (uint)h264enc_outArgs->extErrorCode[i]);
                    }

                    err = VIDENC2_control(codec, XDM_GETSTATUS, (VIDENC2_DynamicParams *) h264enc_dynParams, (VIDENC2_Status *) h264enc_status);
                    DEBUG("[DCE_ENC_TEST] VIDENC2_control - XDM_GETSTATUS err %d", err);

                    for( i=0; i < IH264ENC_EXTERROR_NUM_MAXWORDS; i++ ) {
                        DEBUG("DETAIL EXTENDED ERROR h264enc_status->extErrorCode[%d]=%08x", i, (uint)h264enc_status->extErrorCode[i]);
                    }

                    if( XDM_ISFATALERROR(h264enc_outArgs->videnc2OutArgs.extendedError)) {
                        ERROR("process returned error: %d\n", err);
                        //ERROR("extendedError: %08x", outArgs->extendedError);
                        ERROR("extendedError: %08x", h264enc_outArgs->videnc2OutArgs.extendedError);
                        goto shutdown;
                    } else if( eof ) {
                        //ERROR("Codec_process returned err=%d, extendedError=%08x", err, outArgs->extendedError);
                        ERROR("Codec_process returned err=%d, extendedError=%08x", err, h264enc_outArgs->videnc2OutArgs.extendedError);
                        err = XDM_EFAIL;

                        if( err == XDM_EFAIL ) {
                            DEBUG("-------------------- Flush completed------------------------");
                        }
                    } else {
                        //DEBUG("Non-fatal err=%d, extendedError=%08x", err, outArgs->extendedError);
                        DEBUG("Non-fatal err=%d, h264enc_outArgs->videnc2OutArgs.extendedError=%08x ", err, h264enc_outArgs->videnc2OutArgs.extendedError);
                        err = XDM_EOK;
                    }
                }

                DEBUG("bytesGenerated %d", h264enc_outArgs->videnc2OutArgs.bytesGenerated);
                bytesGenerated = h264enc_outArgs->videnc2OutArgs.bytesGenerated;
            } else if( codec_switch == DCE_ENC_TEST_MPEG4 || codec_switch == DCE_ENC_TEST_H263 ) {
                DEBUG("[DCE_ENC_TEST] codec %p inBufs %p outBufs %p mpeg4enc_inArgs %p mpeg4enc_outArgs %p", codec, inBufs, outBufs, mpeg4enc_inArgs, mpeg4enc_outArgs);
                err = VIDENC2_process(codec, inBufs, outBufs, (VIDENC2_InArgs *) mpeg4enc_inArgs, (VIDENC2_OutArgs *) mpeg4enc_outArgs);
                DEBUG("[DCE_ENC_TEST] VIDENC2_process - err %d", err);
                if( err < 0 ) {
                    //TODO error handling on MPEG4/H.263
                    ERROR("Codec_process returned err=%d, extendedError=%08x", err, mpeg4enc_outArgs->videnc2OutArgs.extendedError);
                    goto shutdown;
                }
                DEBUG("\n bytesGenerated %d", mpeg4enc_outArgs->videnc2OutArgs.bytesGenerated);
                bytesGenerated = mpeg4enc_outArgs->videnc2OutArgs.bytesGenerated;
            }

#ifdef PROFILE_TIME
            INFO("processed returned in: %llu us", (uint64_t) mark_microsecond(&codec_process_time));
#endif

            /*
                  * Handling of output data from codec
                  */

            /* get the output buffer and write it to file */
            DEBUG("pop: %d (%p)", out_cnt, output);

            if( bytesGenerated ) {
                // write the frames to output file based on the value of frames_to_write on how many frames to write.
                if( out_cnt < frames_to_write ) {

#ifdef DUMPOUTPUTDATA
                    char          Buff2[100];
                    int static    GlobalCount2 = 0;
#endif

#ifdef DUMPOUTPUTDATA
                    DEBUG("Dumping output data output[%p]", output);

                    //Dump individual output file
                    if( outputDump == NULL ) {
                        if( GlobalCount2 <= 50 ) {
                            DEBUG("DCE_ENC_TEST GlobalCount2 %d\n", GlobalCount2);
                            sprintf(Buff2, "/sd/dce_enc_dump/dceoutputdump%d.bin", GlobalCount2);
                            outputDump = fopen(Buff2, "wb+");
                            //DEBUG("output data dump file open %p errno %d", outputDump, errno);
                            if( outputDump == NULL ) {
                                DEBUG("Opening output Dump /sd/dce_enc_dump/dceoutputdump.bin file FAILED");
                            } else {
                                GlobalCount2++;
                                fwrite(output, 1, bytesGenerated, outputDump);
                                DEBUG("Dumping output file of H.264 format of generated buffersize = %d", bytesGenerated);

                                fflush(outputDump);
                                fclose(outputDump);
                                outputDump = NULL;
                            }
                        }
                        //DEBUG("output data dump file open %p Successful", outputDump);
                    }
#endif

                    write_output(out_pattern, out_cnt++, output, bytesGenerated);
                } else {
                    out_cnt++;
                }
            }

            ++iters; // Guard for infinite VIDENC2_PROCESS loop when codec never return XDM_EFAIL
        } while( eof && (err != XDM_EFAIL) && (iters < 100));  // Multiple VIDENC2_process when eof until err == XDM_EFAIL

    }

shutdown:

    printf("\nDeleting encoder codec...\n");
    VIDENC2_delete(codec);

out:
    if( engine ) {
        Engine_close(engine);
    }
    if( params ) {
        dce_free(params);
    }
    if( dynParams ) {
        dce_free(dynParams);
    }
    if( status ) {
        dce_free(status);
    }
    if( inBufs ) {
        dce_free(inBufs);
    }
    if( outBufs ) {
        dce_free(outBufs);
    }
    if( inArgs ) {
        dce_free(inArgs);
    }
    if( outArgs ) {
        dce_free(outArgs);
    }

    printf("\nFreeing output %p...\n", output);
    if( output ) {
        MemMgr_Free(output);
    }
#ifdef NON_TILER_OUTPUT
    if( output ) {
        output_enc_free(output, output_size);
    }
#endif

    printf("\nFreeing buf %p...\n", buf);
    if( buf ) {
        if( buf->buf ) {
            MemMgr_Free(buf->buf);
        }
        free(buf);
    }
    printf("DCE ENC test completed...\n");

    return (0);
}

