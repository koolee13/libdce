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
#include <ti/sdo/ce/video3/viddec3.h>

// Need to define ti_sdo_fc_ires_NOPROTOCOLREV to supress warning because
// ijpegvdec.h, impeg2vdec.h, impeg4vdec.h, and ivc1vdec.h will define
// IRES_HDVICP2_PROTOCOLREVISION which is not used.
#define ti_sdo_fc_ires_NOPROTOCOLREV

#include <ti/sdo/codecs/h264vdec/ih264vdec.h>
#include <ti/sdo/codecs/mpeg4vdec/impeg4vdec.h>
#include <ti/sdo/codecs/vc1vdec/ivc1vdec.h>
#include <ti/sdo/codecs/jpegvdec/ijpegvdec.h>
#include <ti/sdo/codecs/mpeg2vdec/impeg2vdec.h>

#include "ti/shmemallocator/SharedMemoryAllocatorUsr.h"

#define PRINT_DEBUG
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

// Getting codec version through XDM_GETVERSION
#define GETVERSION

enum {
    IVAHD_AVC1_DECODE,
    IVAHD_H264_DECODE,
    IVAHD_MP4V_DECODE,
    IVAHD_S263_DECODE,
    IVAHD_VC1AP_DECODE,
    IVAHD_VC1SMP_DECODE,
    IVAHD_VP6V_DECODE,
    IVAHD_MP2V_DECODE,
    IVAHD_JPEGV_DECODE
};

/*
 * A very simple VIDDEC3 client which will decode h264 frames (one per file),
 * and write out raw (unstrided) nv12 frames (one per file).
 */

int                      width, height, frames_to_write, padded_width, padded_height, num_buffers, tiler;
Engine_Handle            engine    = NULL;
VIDDEC3_Handle           codec     = NULL;
VIDDEC3_Params          *params    = NULL;
VIDDEC3_DynamicParams   *dynParams = NULL;
VIDDEC3_Status          *status    = NULL;
XDM2_BufDesc            *inBufs    = NULL;
XDM2_BufDesc            *outBufs   = NULL;
VIDDEC3_InArgs          *inArgs    = NULL;
VIDDEC3_OutArgs         *outArgs   = NULL;

IH264VDEC_Params          *h264_params    = NULL;
IH264VDEC_DynamicParams   *h264_dynParams = NULL;
IH264VDEC_Status          *h264_status    = NULL;

IMPEG4VDEC_Params          *mpeg4_params       = NULL;
IMPEG4VDEC_DynamicParams   *mpeg4_dynParams    = NULL;
IMPEG4VDEC_Status          *mpeg4_status       = NULL;

IVC1VDEC_Params          *vc1_params     = NULL;
IVC1VDEC_DynamicParams   *vc1_dynParams  = NULL;
IVC1VDEC_Status          *vc1_status     = NULL;

IJPEGVDEC_Params          *mjpeg_params       = NULL;
IJPEGVDEC_DynamicParams   *mjpeg_dynParams    = NULL;
IJPEGVDEC_Status          *mjpeg_status       = NULL;

IMPEG2VDEC_Params          *mpeg2_params       = NULL;
IMPEG2VDEC_DynamicParams   *mpeg2_dynParams    = NULL;
IMPEG2VDEC_Status          *mpeg2_status       = NULL;

unsigned int    frameSize[64000]; /* Buffer for keeping frame sizes */
static int      input_offset = 0;

/*! Padding for width as per  Codec Requirement */
#define PADX_H264   32
#define PADX_MPEG4  32
#define PADX_VC1    32
/*! Padding for height as per Codec Requirement */
#define PADY_H264   24
#define PADY_MPEG4  32
#define PADY_VC1    40

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
    DEBUG("tiler alloc return bufPtr %p PA 0x%x", bufPtr, TilerMem_VirtToPhys(bufPtr));

    return (bufPtr);
}

/* ************************************************************************* */
/* utilities to allocate/manage 2d output buffers */

typedef struct OutputBuffer OutputBuffer;

struct OutputBuffer {
    char         *buf; /* virtual address for local access, 4kb stride */
    uint32_t      y, uv; /* virtual address of Y and UV for remote access */
    OutputBuffer *next;      /* next free buffer */
    bool          tiler;
    uint32_t      len;
    shm_buf       shmBuf;
};

static XDAS_Int16
get_mem_type (uint32_t paddr)
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

/* list of free buffers, not locked by codec! */
static OutputBuffer   *head = NULL;

#if 0
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

#endif

int output_allocate(XDM2_BufDesc *outBufs, int cnt,
                    int width, int height, int stride)
{
    int    tw, th;

    outBufs->numBufs = 2;

    if( stride != 4096 ) {
        /* non-2d allocation! */
        int    size_y = stride * height;
        int    size_uv = stride * height / 2;
        tw = size_y + size_uv;
        th = 0;
        outBufs->descs[0].memType = XDM_MEMTYPE_TILEDPAGE;
        outBufs->descs[0].bufSize.bytes = size_y;
        outBufs->descs[1].memType = XDM_MEMTYPE_TILEDPAGE;
        outBufs->descs[1].bufSize.bytes = size_uv;
    } else {
        tw = width;
        th = height;
        outBufs->descs[0].memType = XDM_MEMTYPE_TILED8;
        outBufs->descs[0].bufSize.tileMem.width  = width;
        outBufs->descs[0].bufSize.tileMem.height = height;
        outBufs->descs[1].memType = XDM_MEMTYPE_TILED16;
        outBufs->descs[1].bufSize.tileMem.width  = width; /* UV interleaved width is same a Y */
        outBufs->descs[1].bufSize.tileMem.height = height / 2;
    }

    while( cnt ) {
        OutputBuffer   *buf = calloc(sizeof(OutputBuffer), 1);

        DEBUG(" ----------------- create TILER buf 0x%x --------------------", (unsigned int)buf);

        buf->buf = tiler_alloc(tw, th);
        if( buf->buf ) {
            buf->y   = (uint32_t) buf->buf;
            buf->uv  = (uint32_t)(buf->buf + (height * stride));

            DEBUG("cnt=%d buf=%p, y=%08x, uv=%08x", cnt, buf, buf->y, buf->uv);

            buf->tiler = TRUE;
            buf->next = head;
            head = buf;
        } else {
            ERROR(" ---------------- tiler_alloc failed --------------------");
            free(buf);
            return (-EAGAIN);
        }
        cnt--;
    }

    return (0);
}

int output_allocate_nonTiler(XDM2_BufDesc *outBufs, int cnt,
                             int width, int height, int stride)
{
    int           tw;
    XDAS_Int16    y_type, uv_type;

    outBufs->numBufs = 2;

    while( cnt ) {
        OutputBuffer   *buf = calloc(sizeof(OutputBuffer), 1);

        DEBUG(" ----------------- create nonTILER buf 0x%x --------------------", (unsigned int)buf);
        int    size_y = width * height;
        int    size_uv = width * height * 1 / 2;
        tw = size_y + size_uv;

        // Allocation through mmap
        uint64_t   *vaddr;
        int32_t     ret, len = 0;
        int64_t     paddr = 0;
        uint32_t    uv_addr;

        //vaddr = mmap64(0, tw, PROT_NOCACHE | PROT_READ | PROT_WRITE, MAP_ANON | MAP_PHYS | MAP_SHARED, NOFD, 0);
        ret = SHM_alloc(tw, &buf->shmBuf);
        //if (vaddr == MAP_FAILED) {
        if( ret < 0 ) {
            //ERROR("Failed to do memory mapping\n");
            ERROR("Failed to alloc shmem buffer\n");
            free(buf);
            return (-ENOMEM);
        }
        vaddr = (uint64_t *)buf->shmBuf.vir_addr;

        // Make sure the memory is contiguous
        ret = mem_offset64(vaddr, NOFD, (size_t) tw, &paddr, (size_t *) &len);
        if( ret ) {
            ERROR("Failed to check memory contiguous ret %d errno %d\n", ret, errno);
            //munmap(vaddr, tw);
            SHM_release(&buf->shmBuf);
            free(buf);
            return (-ENOMEM);
        } else if( len != (tw)) {
            ERROR("Failed to check len %d != %d\n", len, tw);
            //munmap(vaddr, tw);
            SHM_release(&buf->shmBuf);
            free(buf);
            return (-ENOMEM);
        }

        buf->buf = (char *) vaddr;
        buf->y = (uint32_t)vaddr;
        uv_addr = (uint32_t) vaddr + (width * height);
        buf->uv = uv_addr;

        DEBUG("cnt=%d nonTILER buf=%p, y=%08x, uv=%08x paddr=%08x", cnt, buf, buf->y, buf->uv, (unsigned int) paddr);

        y_type = get_mem_type(buf->y);
        uv_type = get_mem_type(buf->uv);

        if((y_type < 0) || (uv_type < 0)) {
            DEBUG("non TILER buffer address translation buf->y %x buf->uv %x", buf->y, buf->uv);
            //buf->y = SysLinkMemUtils_translateAddr(buf->y);
            //buf->uv = SysLinkMemUtils_translateAddr(buf->uv);
            y_type = XDM_MEMTYPE_RAW;
            uv_type = XDM_MEMTYPE_RAW;
            DEBUG("buf->y %x buf->uv %x", buf->y, buf->uv);
            if( !buf->y || !buf->uv ) {
                //munmap(vaddr, tw);
                SHM_release(&buf->shmBuf);
                free(buf);
                return (-ENOMEM);
            }
        }

        buf->next = head;
        buf->tiler = FALSE;
        buf->len = tw;
        head = buf;

        cnt--;
    }

    if((y_type == XDM_MEMTYPE_RAW) && (uv_type == XDM_MEMTYPE_RAW)) {
        outBufs->descs[0].memType = y_type;
        outBufs->descs[0].bufSize.bytes = width * height;
        outBufs->descs[1].memType = uv_type;
        outBufs->descs[1].bufSize.bytes = width * height * 1 / 2;
    }

    return (0);
}

void output_free(void)
{
    OutputBuffer   *buf = head;

    while((buf=head)) {
        if( buf->tiler ) {
            MemMgr_Free(buf->buf);
        } else {
            //munmap(buf->buf, buf->len);
            SHM_release(&buf->shmBuf);
        }
        head = buf->next;
        free(buf);
    }
}

OutputBuffer *output_get(void)
{
    OutputBuffer   *buf = head;

    if( buf ) {
        head = buf->next;
    }
    DEBUG("output_get: %p", buf);
    return (buf);
}

void output_release(OutputBuffer *buf)
{
    DEBUG("output_release: %p", buf);
    buf->next = head;
    head = buf;
}

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

/* helper to read one frame of input */
int read_input(const char *pattern, int cnt, char *input)
{
    int           sz = 0, n = 0;
    const char   *path = get_path(pattern, cnt);
    int           fd = open(path, O_RDONLY);

    //DEBUG("Open file fd %d errno %d", fd, errno);
    /* if we can't find the file, then at the end of stream */
    if( fd < 0 ) {
        DEBUG("open input file failed");
        return (0);
    }

    if( frameSize[cnt] && (frameSize[cnt] != -1)) {
        lseek(fd, input_offset, SEEK_SET);
        n = read(fd, input, frameSize[cnt]);
        //DEBUG("reading input frameSize[%d] = n =%d", cnt, n);
        sz += n;
        input_offset += n;
    } else if((frameSize[cnt] == -1)) {
        // This happens to indicate flush command
        DEBUG("Flush requested from file size -1,frameSize[%d] is %d", cnt, frameSize[cnt]);
        sz = -1;
    }

    close(fd);

    DEBUG("sz=%d", sz);
    return (sz);
}

/* helper to write one frame of output */
int write_output(const char *pattern, int cnt, char *y, char *uv, int stride)
{
    int           sz = 0, n, i;
    const char   *path = get_path(pattern, cnt);
    int           fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);

    if( fd < 0 ) {
        ERROR("could open output file: %s (%d)", path, errno);
        return (0);
    }

    for( i = 0; i < height; i++ ) {
        char   *p = y;
        int     len = width;

        while( len && ((n = write(fd, p, len)) > 0)) {
            sz  += n;
            p   += n;
            len -= n;
        }

        if( n < 0 ) {
            ERROR("couldn't write to output file: (%d)", errno);
            break;
        }
        y += stride;
    }

    if( n >= 0 ) {
        for( i = 0; i < height / 2; i++ ) {
            char   *p = uv;
            int     len = width;

            while( len && ((n = write(fd, p, len)) > 0)) {
                sz  += n;
                p   += n;
                len -= n;
            }

            if( n < 0 ) {
                ERROR("couldn't write to output file: (%d)", errno);
                break;
            }
            uv += stride;
        }
    }

    close(fd);

    return (sz);
}

#ifdef PROFILE_TIME
/* for timing in microsecond */
uint64_t mark_microsecond(uint64_t *last)
{
#if 1
    struct timespec    time;
    uint64_t           t1 = 0;

    clock_gettime(CLOCK_REALTIME, &time);
    t1 = timespec2nsec(&time);
    t1 = t1 / 1000;
#else
    uint64_t    t1 = 0;

    t1 = ClockCycles();
    t1 = t1 * 1000000000 / SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    t1 = t1 / 1000;
#endif
    if( last ) {
        return (t1 - *last);
    }
    return (t1);
}

#endif
//#define DUMPINPUTDATA

#ifdef DUMPINPUTDATA
FILE   *inputDump;
#endif

#ifdef GETVERSION
#define VERSION_SIZE 128
#endif

/* decoder body */
int main(int argc, char * *argv)
{
    Engine_Error     ec;
    XDAS_Int32       err;
    char            *input = NULL;
    char            *in_pattern, *out_pattern, *frameData;
    int              in_cnt = 0, out_cnt = 0;
    int              oned, stride;
    unsigned int     frameCount = 0;
    FILE            *frameFile;
    unsigned char    frameinput[10] = { "\0" };
    int              eof = 0;
    int              ivahd_decode_type;
    char             vid_codec[10];
    char             tilerbuffer[10];
    unsigned int     codec_switch = 0;
    unsigned int     vc1_flush = 0;
    Bool             outBufsInUse = FALSE;

#ifdef PROFILE_TIME
    uint64_t    init_start_time = 0;
    uint64_t    codec_process_time = 0;
    uint64_t    total_init_time = 0;
    uint64_t    output_alloc_time = 0;
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
        printf("usage:   %s width height frames_to_write framefile inpattern outpattern codec tilerbuffer\n", argv[0]);
        printf("example: %s 320 240 30 frame.txt in.h264 out.yuv h264 tiler\n", argv[0]);
        printf("example: %s 640 480 30 frame.txt in.m4v out.yuv mpeg4 nontiler\n", argv[0]);
        printf("example: %s 720 480 30 frame.txt in.vc1 out.yuv vc1ap tiler\n", argv[0]);
        printf("example: %s 320 240 30 frame.txt in.vc1 out.yuv vc1smp nontiler\n", argv[0]);
        printf("example: %s 1280 720 30 frame.txt in.bin out.yuv mjpeg tiler\n", argv[0]);
        printf("example: %s 1920 1088 30 frame.txt in.bin out.yuv mpeg2 nontiler\n", argv[0]);
        printf("Currently supported codecs: h264, mpeg4, vc1ap, vc1smp, mjpeg, mpeg2\n");
        return (1);
    }

    /* error checking? */
    width  = atoi(argv[1]);
    height = atoi(argv[2]);
    frames_to_write = atoi(argv[3]);
    frameData   = argv[4];
    in_pattern  = argv[5];
    out_pattern = argv[6];
    strcpy(vid_codec, argv[7]);
    strcpy(tilerbuffer, argv[8]);

    printf("Selected codec: %s\n", vid_codec);
    printf("Selected buffer: %s\n", tilerbuffer);

    if( frames_to_write == -1 ) {
        /* Default : 30 frames to write into output file */
        frames_to_write = 30;
    }

    enum {
        DCE_TEST_H264   = 1,
        DCE_TEST_MPEG4  = 2,
        DCE_TEST_VC1SMP = 3,
        DCE_TEST_VC1AP  = 4,
        DCE_TEST_MJPEG  = 5,
        DCE_TEST_MPEG2  = 6
    };

    if((!(strcmp(vid_codec, "h264")))) {
        ivahd_decode_type = IVAHD_H264_DECODE;
        codec_switch = DCE_TEST_H264;

    } else if((!(strcmp(vid_codec, "mpeg4")))) {
        ivahd_decode_type = IVAHD_MP4V_DECODE;
        codec_switch = DCE_TEST_MPEG4;

    } else if((!(strcmp(vid_codec, "vc1smp")))) {
        ivahd_decode_type = IVAHD_VC1SMP_DECODE;
        codec_switch = DCE_TEST_VC1SMP;
        vc1_flush = 1;

    } else if((!(strcmp(vid_codec, "vc1ap")))) {
        ivahd_decode_type = IVAHD_VC1AP_DECODE;
        codec_switch = DCE_TEST_VC1AP;
        vc1_flush = 1;

    } else if((!(strcmp(vid_codec, "mjpeg")))) {
        ivahd_decode_type = IVAHD_JPEGV_DECODE;
        codec_switch = DCE_TEST_MJPEG;

    } else if((!(strcmp(vid_codec, "mpeg2")))) {
        ivahd_decode_type = IVAHD_MP2V_DECODE;
        codec_switch = DCE_TEST_MPEG2;

    } else {
        printf("No valid codec entry. Please use: h264, mpeg4, vc1ap, vc1smp, mjpeg or mpeg2\n");
        return (1);
    }

    DEBUG("Storing frame size data");
    frameFile = fopen(frameData, "rb");
    DEBUG("frameFile open %p errno %d", frameFile, errno);
    if( frameFile == NULL ) {
        DEBUG("Opening framesize file FAILED");
    }

    /* Read the frame Size from the frame size file */
    while( NULL != fgets((char *)frameinput, 10, frameFile)) {
        frameSize[frameCount] = atoi((char *)frameinput);
        //DEBUG("frameSize[%d] = %d \n", frameCount, frameSize[frameCount]);

        if( frameCount > 64000 ) {
            DEBUG("Num Frames %d exceeded MAX limit %d \n", frameCount, 64000);
            goto out;
        }
        frameCount++;
    }

    DEBUG("Num Frames is %d width=%d, height=%d", frameCount, width, height);

    /* calculate output buffer parameters: */
    width  = ALIGN2(width, 4);         /* round up to MB */
    height = ALIGN2(height, 4);        /* round up to MB */

    switch( codec_switch ) {
        case DCE_TEST_H264 :
            padded_width  = ALIGN2(width + (2 * PADX_H264), 7);
            padded_height = height + 4 * PADY_H264;
            // Some clips don't have enough buffers based on N+3 formula
            // Need sps->num_ref_frames data to match filter precisely
            //num_buffers   = 2 * (MIN(16, 32768 / ((width / 16) * (height / 16)))) + 1;
            num_buffers   = (MIN(16, 32768 / ((width / 16) * (height / 16)))) + 3;
            break;
        case DCE_TEST_MPEG4 :
            padded_width = ALIGN2(width + PADX_MPEG4, 7);
            padded_height = height + PADY_MPEG4;
            num_buffers = 8;
            break;
        case DCE_TEST_VC1SMP :
        case DCE_TEST_VC1AP :
            padded_width = ALIGN2(width + (2 * PADX_VC1), 7);
            padded_height = (ALIGN2(height / 2, 4) * 2) + 4 * PADY_VC1;
            num_buffers = 5;
            break;
        case DCE_TEST_MJPEG :
            padded_width = ALIGN2(width, 4);
            padded_height = ALIGN2(height, 4);
            num_buffers = 5;
            break;
        case DCE_TEST_MPEG2 :
            padded_width = ALIGN2(width, 7);
            padded_height = height;
            num_buffers = 4;
            break;
    }

    if( oned ) {
        stride = padded_width;
    } else {
        stride = 4096;
    }

    DEBUG("padded_width=%d, padded_height=%d, stride=%d, num_buffers=%d",
          padded_width, padded_height, stride, num_buffers);
#ifdef PROFILE_TIME
    init_start_time = mark_microsecond(NULL);
#endif
    engine = Engine_open("ivahd_vidsvr", NULL, &ec);

    if( !engine ) {
        ERROR("fail");
        goto out;
    }

    DEBUG("Engine_open successful engine=%p", engine);

    switch( codec_switch ) {
        case DCE_TEST_H264 :
            params = dce_alloc(sizeof(IH264VDEC_Params));
            if( !params ) {
                ERROR("dce_alloc fail");
                goto out;
            }
            params->size = sizeof(IH264VDEC_Params);
            params->maxBitRate      = 10000000;
            params->displayDelay    = IVIDDEC3_DISPLAY_DELAY_AUTO;
            params->numOutputDataUnits  = 0;
            params->maxWidth            = width;
            break;
        case DCE_TEST_MPEG4 :
            params = dce_alloc(sizeof(IMPEG4VDEC_Params));
            if( !params ) {
                ERROR("dce_alloc fail");
                goto out;
            }
            params->size = sizeof(IMPEG4VDEC_Params);
            params->maxBitRate      = 10000000;
            params->displayDelay    = IVIDDEC3_DISPLAY_DELAY_1;
            params->numOutputDataUnits  = 0;
            params->maxWidth            = width;
            break;
        case DCE_TEST_VC1SMP :
        case DCE_TEST_VC1AP :
            params = dce_alloc(sizeof(IVC1VDEC_Params));
            if( !params ) {
                ERROR("dce_alloc fail");
                goto out;
            }
            params->size = sizeof(IVC1VDEC_Params);
            params->maxBitRate      = 45000000;
            params->displayDelay    = IVIDDEC3_DISPLAY_DELAY_1;
            params->numOutputDataUnits  = 0;
            params->maxWidth            = width;
            break;
        case DCE_TEST_MJPEG :
            params = dce_alloc(sizeof(IJPEGVDEC_Params));
            if( !params ) {
                ERROR("dce_alloc fail");
                goto out;
            }
            params->size = sizeof(IJPEGVDEC_Params);
            params->maxBitRate      = 10000000;
            params->displayDelay    = IVIDDEC3_DISPLAY_DELAY_1;
            params->numOutputDataUnits  = 1;
            params->maxWidth            = width;
            break;

        case DCE_TEST_MPEG2 :
            params = dce_alloc(sizeof(IMPEG2VDEC_Params));
            if( !params ) {
                ERROR("dce_alloc fail");
                goto out;
            }
            params->size = sizeof(IMPEG2VDEC_Params);
            params->maxBitRate      = 10000000;
            params->displayDelay    = IVIDDEC3_DISPLAY_DELAY_1;
            params->numOutputDataUnits  = 0;
            params->maxWidth            = padded_width;
            break;

    }

    params->maxHeight           = height;
    params->maxFrameRate        = 30000;
    params->dataEndianness      = XDM_BYTE;
    params->forceChromaFormat   = XDM_YUV_420SP;
    params->operatingMode       = IVIDEO_DECODE_ONLY;
    //params->displayDelay        = IVIDDEC3_DECODE_ORDER;
    params->displayBufsMode     = IVIDDEC3_DISPLAYBUFS_EMBEDDED;
    params->inputDataMode       = IVIDEO_ENTIREFRAME;
    params->metadataType[0]     = IVIDEO_METADATAPLANE_NONE;
    params->metadataType[1]     = IVIDEO_METADATAPLANE_NONE;
    params->metadataType[2]     = IVIDEO_METADATAPLANE_NONE;
    params->outputDataMode      = IVIDEO_ENTIREFRAME;
    params->numInputDataUnits   = 0;
    params->errorInfoMode       = IVIDEO_ERRORINFO_OFF;

    DEBUG("dce_alloc VIDDEC3_Params successful params=%p", params);

    switch( codec_switch ) {
        case DCE_TEST_H264 :
            h264_params = (IH264VDEC_Params *) params;
            h264_params->dpbSizeInFrames = IH264VDEC_DPB_NUMFRAMES_AUTO;
            h264_params->pConstantMemory = 0;
            h264_params->presetLevelIdc = IH264VDEC_LEVEL41;
            h264_params->errConcealmentMode = IH264VDEC_APPLY_CONCEALMENT;
            h264_params->temporalDirModePred = TRUE;
            h264_params->detectCabacAlignErr = IH264VDEC_DISABLE_CABACALIGNERR_DETECTION;

            DEBUG("dce_alloc VIDDEC3_Params successful h264_params=%p", h264_params);

            err = msync((Ptr)h264_params, sizeof(IH264VDEC_Params), MS_CACHE_ONLY | MS_SYNC);

            codec = VIDDEC3_create(engine, "ivahd_h264dec", (VIDDEC3_Params *)h264_params);
            break;

        case DCE_TEST_MPEG4 :
            mpeg4_params = (IMPEG4VDEC_Params *) params;
            mpeg4_params->outloopDeBlocking = TRUE;
            mpeg4_params->sorensonSparkStream = FALSE;
            mpeg4_params->errorConcealmentEnable = FALSE;
            mpeg4_params->debugTraceLevel = 0;
            mpeg4_params->lastNFramesToLog = 0;
            mpeg4_params->paddingMode = IMPEG4VDEC_DEFAULT_MODE_PADDING;

            DEBUG("dce_alloc VIDDEC3_Params successful mpeg4_params=%p", mpeg4_params);

            err = msync((Ptr)mpeg4_params, sizeof(IMPEG4VDEC_Params), MS_CACHE_ONLY | MS_SYNC);

            codec = VIDDEC3_create(engine, "ivahd_mpeg4dec", (VIDDEC3_Params *)mpeg4_params);
            break;

        case DCE_TEST_VC1SMP :
        case DCE_TEST_VC1AP :
            vc1_params = (IVC1VDEC_Params *) params;

            DEBUG("dce_alloc VIDDEC3_Params successful vc1_params=%p", vc1_params);

            err = msync((Ptr)vc1_params, sizeof(IVC1VDEC_Params), MS_CACHE_ONLY | MS_SYNC);

            codec = VIDDEC3_create(engine, "ivahd_vc1vdec", (VIDDEC3_Params *)vc1_params);
            break;

        case DCE_TEST_MJPEG :
            mjpeg_params = (IJPEGVDEC_Params *) params;
            mjpeg_params->ErrorConcealmentON = TRUE;
            mjpeg_params->debugTraceLevel = 0;
            mjpeg_params->lastNFramesToLog = 0;
            mjpeg_params->sliceSwitchON = 0;
            mjpeg_params->numSwitchPerFrame = 0;
            mjpeg_params->numRestartMarkerPerSwitch = 0;

            DEBUG("dce_alloc VIDDEC3_Params successful mjpeg_params=%p", mjpeg_params);

            err = msync((Ptr)mjpeg_params, sizeof(IJPEGVDEC_Params), MS_CACHE_ONLY | MS_SYNC);

            codec = VIDDEC3_create(engine, "ivahd_jpegvdec", (VIDDEC3_Params *)mjpeg_params);
            break;

        case DCE_TEST_MPEG2 :
            mpeg2_params = (IMPEG2VDEC_Params *) params;
            mpeg2_params->outloopDeBlocking = TRUE;
            mpeg2_params->ErrorConcealmentON = FALSE;
            mpeg2_params->debugTraceLevel = 0;
            mpeg2_params->lastNFramesToLog = 0;

            DEBUG("dce_alloc VIDDEC3_Params successful mpeg2_params=%p", mpeg2_params);

            err = msync((Ptr)mpeg2_params, sizeof(IMPEG2VDEC_Params), MS_CACHE_ONLY | MS_SYNC);

            codec = VIDDEC3_create(engine, "ivahd_mpeg2vdec", (VIDDEC3_Params *)mpeg2_params);
            break;

    }

    if( !codec ) {
        ERROR("fail");
        goto out;
    }

    DEBUG("VIDDEC3_create successful codec=%p", codec);

    switch( codec_switch ) {
        case DCE_TEST_H264 :
            dynParams = dce_alloc(sizeof(IH264VDEC_DynamicParams));
            dynParams->size = sizeof(IH264VDEC_DynamicParams);
            break;
        case DCE_TEST_MPEG4 :
            dynParams = dce_alloc(sizeof(IMPEG4VDEC_DynamicParams));
            dynParams->size = sizeof(IMPEG4VDEC_DynamicParams);
            dynParams->lateAcquireArg = -1;
            break;
        case DCE_TEST_VC1SMP :
        case DCE_TEST_VC1AP :
            dynParams = dce_alloc(sizeof(IVC1VDEC_DynamicParams));
            dynParams->size = sizeof(IVC1VDEC_DynamicParams);
            dynParams->lateAcquireArg = -1;
            break;
        case DCE_TEST_MJPEG :
            dynParams = dce_alloc(sizeof(IJPEGVDEC_DynamicParams));
            dynParams->size = sizeof(IJPEGVDEC_DynamicParams);
            dynParams->lateAcquireArg = -1;
            break;

        case DCE_TEST_MPEG2 :
            dynParams = dce_alloc(sizeof(IMPEG2VDEC_DynamicParams));
            dynParams->size = sizeof(IMPEG2VDEC_DynamicParams);
            dynParams->lateAcquireArg = -1;
            break;

    }

    dynParams->decodeHeader  = XDM_DECODE_AU;
    /*Not Supported: Set default*/
    dynParams->displayWidth  = 0;
    dynParams->frameSkipMode = IVIDEO_NO_SKIP;
    dynParams->newFrameFlag  = XDAS_TRUE;

/*
 * VIDDEC3_control - XDM_GETVERSION & XDM_SETPARAMS
 */

#ifdef GETVERSION
    // Allocating TILER 1D to store the Codec version information from Codec.
    char *codec_version = NULL;
    codec_version = tiler_alloc(VERSION_SIZE, 0);
#endif

    switch( codec_switch ) {
        case DCE_TEST_H264 :
            h264_dynParams = (IH264VDEC_DynamicParams *) dynParams;
            DEBUG("Allocating IH264VDEC_DynamicParams successful h264_dynParams=%p", h264_dynParams);

            status = dce_alloc(sizeof(IH264VDEC_Status));
            status->size = sizeof(IH264VDEC_Status);
            DEBUG("dce_alloc IH264VDEC_Status successful status=%p", status);

#ifdef GETVERSION
            status->data.buf = (XDAS_Int8 *) codec_version;
            status->data.bufSize = VERSION_SIZE;
#endif

            h264_status = (IH264VDEC_Status *) status;
            DEBUG("IH264VDEC_Status successful h264_status=%p", h264_status);

#ifdef GETVERSION
            err = VIDDEC3_control(codec, XDM_GETVERSION, (VIDDEC3_DynamicParams *) h264_dynParams, (VIDDEC3_Status *) h264_status);
            DEBUG("VIDDEC3_control IH264VDEC_Status XDM_GETVERSION h264_status->data.buf = %s", (((VIDDEC3_Status *)h264_status)->data.buf));
#endif

            err = VIDDEC3_control(codec, XDM_SETPARAMS, (VIDDEC3_DynamicParams *) h264_dynParams, (VIDDEC3_Status *) h264_status);
            break;

        case DCE_TEST_MPEG4 :
            mpeg4_dynParams = (IMPEG4VDEC_DynamicParams *) dynParams;
            DEBUG("Allocating IMPEG4VDEC_DynamicParams successful mpeg4_dynParams=%p", mpeg4_dynParams);

            status = dce_alloc(sizeof(IMPEG4VDEC_Status));
            status->size = sizeof(IMPEG4VDEC_Status);
            DEBUG("dce_alloc IMPEG4VDEC_Status successful status=%p", status);

#ifdef GETVERSION
            status->data.buf = (XDAS_Int8 *) codec_version;
            status->data.bufSize = VERSION_SIZE;
#endif

            mpeg4_status = (IMPEG4VDEC_Status *) status;
            DEBUG("dce_alloc IMPEG4VDEC_Status successful mpeg4_status=%p", mpeg4_status);

#ifdef GETVERSION
            err = VIDDEC3_control(codec, XDM_GETVERSION, (VIDDEC3_DynamicParams *) mpeg4_dynParams, (VIDDEC3_Status *) mpeg4_status);
            DEBUG("VIDDEC3_control IMPEG4VDEC_Status XDM_GETVERSION mpeg4_status->data.buf = %s", (((VIDDEC3_Status *)mpeg4_status)->data.buf));
#endif

            err = VIDDEC3_control(codec, XDM_SETPARAMS, (VIDDEC3_DynamicParams *) mpeg4_dynParams, (VIDDEC3_Status *) mpeg4_status);
            break;

        case DCE_TEST_VC1SMP :
        case DCE_TEST_VC1AP :
            vc1_dynParams = (IVC1VDEC_DynamicParams *) dynParams;
            DEBUG("Allocating IVC1VDEC_DynamicParams successful vc1_dynParams=%p", vc1_dynParams);

            status = dce_alloc(sizeof(IVC1VDEC_Status));
            status->size = sizeof(IVC1VDEC_Status);
            DEBUG("dce_alloc IVC1VDEC_Status successful status=%p", status);

#ifdef GETVERSION
            status->data.buf = (XDAS_Int8 *) codec_version;
            status->data.bufSize = VERSION_SIZE;
#endif

            vc1_status = (IVC1VDEC_Status *) status;
            DEBUG("dce_alloc IVC1VDEC_Status successful vc1_status=%p", vc1_status);

#ifdef GETVERSION
            err = VIDDEC3_control(codec, XDM_GETVERSION, (VIDDEC3_DynamicParams *) vc1_dynParams, (VIDDEC3_Status *) vc1_status);
            DEBUG("VIDDEC3_control IVC1VDEC_Status XDM_GETVERSION vc1_status->data.buf = %s", (((VIDDEC3_Status *)vc1_status)->data.buf));
#endif

            err = VIDDEC3_control(codec, XDM_SETPARAMS, (VIDDEC3_DynamicParams *) vc1_dynParams, (VIDDEC3_Status *) vc1_status);
            break;

        case DCE_TEST_MJPEG :
            mjpeg_dynParams = (IJPEGVDEC_DynamicParams *) dynParams;
            mjpeg_dynParams->decodeThumbnail = 0;
            mjpeg_dynParams->thumbnailMode = IJPEGVDEC_THUMBNAIL_DOWNSAMPLE;
            mjpeg_dynParams->downsamplingFactor = IJPEGVDEC_NODOWNSAMPLE;
            mjpeg_dynParams->streamingCompliant = 1;
            DEBUG("Allocating IJPEGVDEC_DynamicParams successful mjpeg_dynParams=%p", mjpeg_dynParams);

            status = dce_alloc(sizeof(IJPEGVDEC_Status));
            status->size = sizeof(IJPEGVDEC_Status);
            DEBUG("dce_alloc IJPEGVDEC_Status successful status=%p", status);

#ifdef GETVERSION
            status->data.buf = (XDAS_Int8 *) codec_version;
            status->data.bufSize = VERSION_SIZE;
#endif

            mjpeg_status = (IJPEGVDEC_Status *) status;
            DEBUG("dce_alloc IJPEGVDEC_Status successful mjpeg_status=%p", mjpeg_status);

#ifdef GETVERSION
            err = VIDDEC3_control(codec, XDM_GETVERSION, (VIDDEC3_DynamicParams *) mjpeg_dynParams, (VIDDEC3_Status *) mjpeg_status);
            DEBUG("VIDDEC3_control IJPEGVDEC_Status XDM_GETVERSION mjpeg_status->data.buf = %s", (((VIDDEC3_Status *)mjpeg_status)->data.buf));
#endif

            err = VIDDEC3_control(codec, XDM_SETPARAMS, (VIDDEC3_DynamicParams *) mjpeg_dynParams, (VIDDEC3_Status *) mjpeg_status);
            break;

        case DCE_TEST_MPEG2 :
            mpeg2_dynParams = (IMPEG2VDEC_DynamicParams *) dynParams;
            //MPEG2 buffer width should be 128 byte aligned for non TILER (atleast)
            //If displayWidth=0 then MPEG2 codec does not have a way to calculate
            //the stride as compared to other codecs which can calculate it from
            //buffer size and image height.stride=buffersize/(height*1.5)
            mpeg2_dynParams->viddecDynamicParams.displayWidth = padded_width;

            DEBUG("dce_alloc IMPEG2VDEC_DynamicParams successful mpeg2_dynParams=%p", mpeg2_dynParams);

            status = dce_alloc(sizeof(IMPEG2VDEC_Status));
            status->size = sizeof(IMPEG2VDEC_Status);
            DEBUG("dce_alloc IMPEG2VDEC_Status successful status=%p", status);

#ifdef GETVERSION
            status->data.buf = (XDAS_Int8 *) codec_version;
            status->data.bufSize = VERSION_SIZE;
#endif

            mpeg2_status = (IMPEG2VDEC_Status *) status;
            DEBUG("mpeg2_status=%p", mpeg2_status);

#ifdef GETVERSION
            err = VIDDEC3_control(codec, XDM_GETVERSION, (VIDDEC3_DynamicParams *) mpeg2_dynParams, (VIDDEC3_Status *) mpeg2_status);
            DEBUG("VIDDEC3_control IMPEG2VDEC_Status XDM_GETVERSION mpeg2_status->data.buf = %s", (((VIDDEC3_Status *)mpeg2_status)->data.buf));
#endif

            err = VIDDEC3_control(codec, XDM_SETPARAMS, (VIDDEC3_DynamicParams *) mpeg2_dynParams, (VIDDEC3_Status *) mpeg2_status);
            break;

        default :
            DEBUG("Not implemented or supported codec_switch %d", codec_switch);
    }

#ifdef GETVERSION
    if( codec_version ) {
        MemMgr_Free(codec_version);
    }
#endif

    if( err ) {
        ERROR("fail: %d", err);
        goto shutdown;
    }

    DEBUG("VIDDEC3_control XDM_SETPARAMS successful");

    DEBUG("input buffer configuration width %d height %d", width, height);
    inBufs = dce_alloc(sizeof(XDM2_BufDesc));
    inBufs->numBufs = 1;
    input = tiler_alloc(width * height, 0);
    inBufs->descs[0].buf = (XDAS_Int8 *)input;
    inBufs->descs[0].memType = XDM_MEMTYPE_RAW;

    DEBUG("inBufs->descs[0].buf %p input %p", inBufs->descs[0].buf, input);

    outBufs = dce_alloc(sizeof(XDM2_BufDesc));

    DEBUG("output buffer configuration num_buffers %d padded_width %d padded_height %d", num_buffers, padded_width, padded_height);

#ifdef PROFILE_TIME
    uint64_t    alloc_time_start = mark_microsecond(NULL);
#endif

    if( !(strcmp(tilerbuffer, "tiler"))) {
        DEBUG("Output allocate through tiler");
        tiler = 1;
        err = output_allocate(outBufs, num_buffers,
                              padded_width, padded_height, stride);
    } else {
        DEBUG("Output allocate through non-tiler");
        tiler = 0;

        err = output_allocate_nonTiler(outBufs, num_buffers,
                                       padded_width, padded_height, stride);
    }

#ifdef PROFILE_TIME
    output_alloc_time = mark_microsecond(&alloc_time_start);
#endif

    if( err ) {
        ERROR("fail: %d", err);
        goto shutdown;
    }

    inArgs = dce_alloc(sizeof(IVIDDEC3_InArgs));
    inArgs->size = sizeof(IVIDDEC3_InArgs);

    outArgs = dce_alloc(sizeof(IVIDDEC3_OutArgs));
    outArgs->size = sizeof(IVIDDEC3_OutArgs);

#ifdef PROFILE_TIME
    total_init_time = (uint64_t)mark_microsecond(&init_start_time);
    INFO("total_init_time %llu output_alloc_time %llu actual init time in: %lld us", total_init_time, output_alloc_time, total_init_time  - output_alloc_time);
#endif

    while( inBufs->numBufs && outBufs->numBufs ) {
        OutputBuffer   *buf;
        int             n, i;

        if( !outBufsInUse ) {
            buf = output_get();
            if( !buf ) {
                ERROR("fail: out of buffers");
                goto shutdown;
            }
        } else {
            buf = 0;
        }

        n = read_input(in_pattern, in_cnt, input);
        if( n && (n != -1)) {
            eof = 0;
            inBufs->numBufs = 1;
            inBufs->descs[0].buf = (XDAS_Int8 *)input;
            inBufs->descs[0].bufSize.bytes = n;
            inArgs->numBytes = n;
            DEBUG("push: %d (%d bytes) (%p)", in_cnt, n, buf);
            in_cnt++;

            /*
             * Input buffer has data to be decoded.
             */
            inArgs->inputID = (XDAS_Int32)buf;
            if( !outBufsInUse ) {
                outBufs->numBufs = 2;
                outBufs->descs[0].buf = (XDAS_Int8 *)buf->y;
                outBufs->descs[1].buf = (XDAS_Int8 *)buf->uv;
            }
        } else if( n == -1 ) {

            // Set EOF as 1 to ensure flush completes
            eof = 1;
            in_cnt++;

            switch( codec_switch ) {
                case DCE_TEST_H264 :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH h264_dynParams %p h264_status %p", h264_dynParams, h264_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) h264_dynParams, (VIDDEC3_Status *) h264_status);
                    break;
                case DCE_TEST_MPEG4 :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH mpeg4_dynParams %p mpeg4_status %p", mpeg4_dynParams, mpeg4_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) mpeg4_dynParams, (VIDDEC3_Status *) mpeg4_status);
                    break;
                case DCE_TEST_VC1SMP :
                case DCE_TEST_VC1AP :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH vc1_dynParams %p vc1_status %p", vc1_dynParams, vc1_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) vc1_dynParams, (VIDDEC3_Status *) vc1_status);
                    break;
                case DCE_TEST_MJPEG :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH mjpeg_dynParams %p mjpeg_status %p", mjpeg_dynParams, mjpeg_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) mjpeg_dynParams, (VIDDEC3_Status *) mjpeg_status);
                    break;

                case DCE_TEST_MPEG2 :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH mpeg2_dynParams %p mpeg2_status %p", mpeg2_dynParams, mpeg2_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) mpeg2_dynParams, (VIDDEC3_Status *) mpeg2_status);
                    break;

            }

            /* We have sent the XDM_FLUSH, call VIDDEC3_process until we get
             * an error of XDM_EFAIL which tells us there are no more buffers
             * at codec level.
             */

            inArgs->inputID = 0;
            inArgs->numBytes = 0;
            inBufs->descs[0].buf = NULL;
            inBufs->descs[0].bufSize.bytes = 0;
            outBufs->numBufs = 0;
            outBufs->descs[0].buf = NULL;
            outBufs->descs[1].buf = NULL;
            if( buf ) {
                output_release(buf);
            }
            outBufsInUse = 0;


        } else {
            /* end of input.. */
            inBufs->numBufs = 0;
            eof = 1;

            switch( codec_switch ) {
                case DCE_TEST_H264 :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH h264_dynParams %p h264_status %p", h264_dynParams, h264_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) h264_dynParams, (VIDDEC3_Status *) h264_status);
                    break;
                case DCE_TEST_MPEG4 :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH mpeg4_dynParams %p mpeg4_status %p", mpeg4_dynParams, mpeg4_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) mpeg4_dynParams, (VIDDEC3_Status *) mpeg4_status);
                    break;
                case DCE_TEST_VC1SMP :
                case DCE_TEST_VC1AP :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH vc1_dynParams %p vc1_status %p", vc1_dynParams, vc1_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) vc1_dynParams, (VIDDEC3_Status *) vc1_status);
                    break;
                case DCE_TEST_MJPEG :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH mjpeg_dynParams %p mjpeg_status %p", mjpeg_dynParams, mjpeg_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) mjpeg_dynParams, (VIDDEC3_Status *) mjpeg_status);
                    break;

                case DCE_TEST_MPEG2 :
                    DEBUG("Calling VIDDEC3_control XDM_FLUSH mpeg2_dynParams %p mpeg2_status %p", mpeg2_dynParams, mpeg2_status);
                    err = VIDDEC3_control(codec, XDM_FLUSH, (VIDDEC3_DynamicParams *) mpeg2_dynParams, (VIDDEC3_Status *) mpeg2_status);
                    break;

            }

            /* We have sent the XDM_FLUSH, call VIDDEC3_process until we get
             * an error of XDM_EFAIL which tells us there are no more buffers
             * at codec level.
             */

            inArgs->inputID = 0;
            inArgs->numBytes = 0;
            inBufs->numBufs = 0;
            inBufs->descs[0].buf = NULL;
            inBufs->descs[0].bufSize.bytes = 0;
            outBufs->numBufs = 0;
            outBufs->descs[0].buf = NULL;
            outBufs->descs[1].buf = NULL;
            if( buf ) {
                output_release(buf);
            }
        }

#ifdef DUMPINPUTDATA
        DEBUG("input data dump inArgs->numBytes[%d] inputDump[%p]", inArgs->numBytes, inputDump);

        //Dump the file
        if( inputDump == NULL ) {
            inputDump = fopen("/tmp/inputdump.h264", "ab");
            DEBUG("input data dump file open %p errno %d", inputDump, errno);
            if( inputDump == NULL ) {
                DEBUG("Opening input Dump /tmp/inputdump.h264 file FAILED");
            }
        }
        DEBUG("input data dump file open %p Successful", inputDump);

        fwrite(input, sizeof(char), inArgs->numBytes, inputDump);
        DEBUG("Dumping input file with size = %d ", inArgs->numBytes);
        fflush(inputDump);
        fclose(inputDump);
        inputDump = NULL;
    #endif

        int    iters = 0;

        do {
            DEBUG("Calling VIDDEC3_process inArgs->inputID=%x inBufs->descs[0].buf %p inBufs->descs.bufSize %d input %p",
                  inArgs->inputID, inBufs->descs[0].buf, (int) inBufs->descs[0].bufSize.bytes, input);
#ifdef PROFILE_TIME
            codec_process_time = mark_microsecond(NULL);
#endif
            err = VIDDEC3_process(codec, inBufs, outBufs, inArgs, outArgs);
#ifdef PROFILE_TIME
            INFO("processed returned in: %llu us", (uint64_t) mark_microsecond(&codec_process_time));
#endif
            if( err ) {
                if( XDM_ISFATALERROR(outArgs->extendedError)) {
                    ERROR("process returned error: %d\n", err);
                    ERROR("extendedError: %08x", outArgs->extendedError);
                    goto shutdown;
                } else if( eof ) {
                    /*
                     * Flush has been ordered, processing the returned output from codec.
                     * For H.264, bit 18 - IH264VDEC_ERR_STREAM_END indicates flush is completed.
                     * For MPEG4, bit 24 - IMPEG4VDEC_ERR_STREAM_END indicates flush is completed.
                     * Only return XDM_EFAIL, when those bit are set.
                     */
                    ERROR("Codec_process returned err=%d, extendedError=%08x", err, outArgs->extendedError);
                    err = XDM_EFAIL;

                    if((!(((outArgs->extendedError) >> 24) & 0x1)) &&
                       ((ivahd_decode_type == 2 /*IVAHD_MP4V_DECODE*/) ||
                        (ivahd_decode_type == 3 /*IVAHD_S263_DECODE*/))) {
                        err = XDM_EOK;
                    }

                    if((!(((outArgs->extendedError) >> 18) & 0x1)) &&
                       ((ivahd_decode_type == 1 /*IVAHD_H264_DECODE*/) ||
                        (ivahd_decode_type == 0 /*IVAHD_AVC1_DECODE*/))) {
                        err = XDM_EOK;
                    }

                    if( err == XDM_EFAIL ) {
                        DEBUG("-------------------- Flush completed------------------------");
                    }
                } else {
                    DEBUG("Non-fatal err=%d, extendedError=%08x", err, outArgs->extendedError);
                    err = XDM_EOK;
                }
            }

            /*
             * Handling of output data from codec
             */
            if( tiler ) {
                for( i = 0; outArgs->outputID[i]; i++ ) {
                    /* calculate offset to region of interest */
                    XDM_Rect   *r = &(outArgs->displayBufs.bufDesc[0].activeFrameRegion);

                    int    yoff  = (r->topLeft.y * stride) + r->topLeft.x;
                    int    uvoff = (r->topLeft.y * stride / 2) + (stride * padded_height) + r->topLeft.x;

                    /* get the output buffer and write it to file */
                    buf = (OutputBuffer *)outArgs->outputID[i];
                    DEBUG("pop: %d (%p)", out_cnt, buf);

                    if( out_cnt < frames_to_write ) {  // write first 30 frames to output file out_cnt < 300
                        write_output(out_pattern, out_cnt++, buf->buf + yoff,
                                     buf->buf + uvoff, stride);
                    } else {
                        out_cnt++;
                    }
                }
            } else {
                for( i = 0; outArgs->outputID[i]; i++ ) {
                    /* calculate offset to region of interest */
                    XDM_Rect   *r = &(outArgs->displayBufs.bufDesc[0].activeFrameRegion);

                    int    yoff  = (r->topLeft.y * padded_width) + r->topLeft.x;
                    int    uvoff = (r->topLeft.y * padded_width / 2) + (padded_height * padded_width) + r->topLeft.x;

                    /* get the output buffer and write it to file */
                    buf = (OutputBuffer *)outArgs->outputID[i];
                    DEBUG("pop: %d (%p)", out_cnt, buf);

                    if( out_cnt < 300 ) {  // write first 300 frames to output file
                        write_output(out_pattern, out_cnt++, buf->buf + yoff,
                                     buf->buf + uvoff, padded_width);
                    } else {
                        out_cnt++;
                    }
                }
            }

            for( i = 0; outArgs->freeBufID[i]; i++ ) {
                DEBUG("freeBufID[%d] = %d", i, outArgs->freeBufID[i]);
                buf = (OutputBuffer *)outArgs->freeBufID[i];
                output_release(buf);
            }

            if( outArgs->outBufsInUseFlag ) {
                outBufsInUse = TRUE;
                DEBUG("outBufsInUseFlag is SET. Not sending a new output buffer to codec on the next Codec_process ");
            } else {
                outBufsInUse = FALSE;
            }

            ++iters; // Guard for infinite VIDDEC3_PROCESS loop when codec never return XDM_EFAIL
        } while( eof && (err != XDM_EFAIL) && (iters < 100));  // Multiple VIDDEC3_process when eof until err == XDM_EFAIL

    }

shutdown:

    printf("\nDeleting codec...\n");
    VIDDEC3_delete(codec);

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
    if( input ) {
        MemMgr_Free(input);
    }

    output_free();

    printf("DCE test completed...\n");

    return (0);
}

