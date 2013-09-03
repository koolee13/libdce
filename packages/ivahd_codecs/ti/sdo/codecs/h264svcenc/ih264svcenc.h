/*
********************************************************************************
 * HDVICP2.0 Based SVC Baseline Encoder
 *
 * "HDVICP2.0 Based SVC Baseline Encoder" is software module developed on TI's
 *  HDVICP2 based SOCs. This module is capable of compressing a 4:2:0 Raw
 *  video into a SVC baseline profile bit-stream. Based on ISO/IEC
 *  14496-10."
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
********************************************************************************
*/
/**
  ******************************************************************************
  *  @file     ih264svcenc.h
  *
  *  @brief    IH264SVCENC Interface Header
  *
  *  @author: Venugopala Krishna (venugopala@ti.com)
  *
  *  @version 0.0 (Apr 2010) : Initial version created
  *****************************************************************************
*/

/**
 *  @defgroup   HDVICP2SVC IH264SVCENC_TI (V7M)
 *  @ingroup    m3
 *
 *              The ISVCENC_TI interface enables encoding in SVC format
 *
 */

#ifndef _IH264SVCENC_H_  //--{

#define _IH264SVCENC_H_

#include <ti/xdais/ialg.h>
#include <ti/xdais/dm/ividenc2.h>

#include "ih264enc.h"

/** @ingroup    HDVICP2SVC */
/*@{*/


#ifdef __cplusplus
extern "C" {
#endif


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Definition of all the macros define by this interafce       */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 * Maximum number of instances of H.264 embedded inside SVC
*/
#define   IH264SVCENC_MAX_NUM_MULTI_INSTANCE   (9)

/**
  Length of the version string. The memory to get version
  number is owned by application
*/
#define IH264SVCENC_VERSION_LENGTH (64)


/**
  control method commands
*/
#define IH264SVCENC_GETSTATUS      XDM_GETSTATUS
#define IH264SVCENC_SETPARAMS      XDM_SETPARAMS
#define IH264SVCENC_RESET          XDM_RESET
#define IH264SVCENC_FLUSH          XDM_FLUSH
#define IH264SVCENC_SETDEFAULT     XDM_SETDEFAULT
#define IH264SVCENC_GETBUFINFO     XDM_GETBUFINFO

typedef IVIDENC2_Cmd IH264SVCENC_Cmd;


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Definition of all the Enumeration define by this interafce  */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 *  @enum       IH264SVCENC_ErrorBit
 *  @brief      error informations of IVAHD SVC encoder implementation by TI.
 *
 *  @remarks    When an internal error occurs, the algorithm will return
 *              an error return value (e.g. EFAIL, EUNSUPPORTED)
 *
 *  @remarks    The value of each enum is the bit which is set.
 *
 *  @remarks    Bits 8-15 are defined by XDM and hence not used by codec
 *              implementation.
 *              rest all bits are used. XDM defined error bits are also active.
 *
 *  @remarks    The algorithm can set multiple bits to 1 based on conditions.
 *              e.g. it will set bits #XDM_FATALERROR (fatal) and
 *              #XDM_UNSUPPORTEDPARAM (unsupported params) in case
 *              of unsupported run time parameters.
 *
 */

typedef enum {
    IH264SVCENC_LEVEL_INCOMPLAINT_PARAMETER = 0
                                              /**< Bit 0 - level incomplaint parameters.
                                              *   @remarks  This error is applicable when some parameters are set
                                              *   which are not meeting the limit defined by H.264 standard
                                              *   Table A-1  Level limits. It can be categorized under
                                              *   following category :
                                              *   IH264ENC_LEVEL_INCOMPLAINT_RESOLUTION : Invalid width/height
                                              *   IH264ENC_LEVEL_INCOMPLAINT_HRDBUFSZIE : Invalid HrdBufferSize
                                              *   IH264ENC_LEVEL_INCOMPLAINT_BITRATE    : Invalid Bit Rate
                                              *   IH264ENC_LEVEL_INCOMPLAINT_MBSPERSECOND : Invalid FrameRate/
                                              *                                             resolution
                                              *   IH264ENC_LEVEL_INCOMPLAINT_DPBSIZE      : Invalid DPB size
                                              *   For above 5 situations, only a signle bit (bit-0) is set as true
                                              */

} IH264SVCENC_ErrorBit;

/**
 *  @enum       ISVCENC_Level
 *  @brief      Level Identifier for H.264 Encoder
*/
typedef enum {
    IH264SVC_LEVEL_10 = 10 /**<  Level 1.0  */

} IH264SVCENC_Level;



/**<
 *  @struct IH264SVCENC_Params
 *  @brief This structure defines the Create time parameters for all
 *         H264SVCENC objects
 *  @param  h264Params           Create Parameters (Extended Class) of the H264 Encoder.
 *                               As defined in the interface file (ih264enc.h) of H264 Encoder.
 *
 *  @param  inBufIndex           Conveys how inBufs passed in the process is mapped to
 *                               different layers/instances of Encoder.
 *
 *  @param  numberOfLayers       Conveys number of layers to be encoded. This field is used
 *                               in reading the number of entries of h264Params, inBufIndex etc.
 *
 *  @param  avcMode              If numberOfLayers is set to be ONE, this flag determines whether
 *                               generated Bitstream be SVC or AVC. Non-Zero means AVC or if ZERO its
 *                               SVC. NOTE: This field is don't care if numberOfLayers is greater than ONE.
 *
*/

typedef struct {
    IH264ENC_Params h264Params[IH264SVCENC_MAX_NUM_MULTI_INSTANCE];
    XDAS_Int8       inBufIndex[IH264SVCENC_MAX_NUM_MULTI_INSTANCE];
    XDAS_UInt8      numberOfLayers;
    XDAS_UInt8      avcMode;

} IH264SVCENC_Params;


/**<
 *
 *  @struct IH264SVCENC_Status
 *  @brief This structure informs back the status of SVC encoder and tells the
 *         value of each control parameter
 *  @param  h264Status           Status Parameters (Extended Class) of the H264 Encoder.
 *                               As defined in the interface file (ih264enc.h) of H264 Encoder.
 *
 *  @param  videnc2Status        IVIDENC2 Class Status Parameter given out by SVC Encoder after collating,
 *                               information from different encoding layers/instances.
 *
 *  @param  numberOfLayers       Conveys the information for how many number of layers encoding the SVC
 *                               Encoder is configured. This field is used in reading the number of
 *                               entries of h264Status etc.
 *
 *  @param  avcMode              If numberOfLayers is set to be ONE, this flag determines whether
 *                               generated Bitstream be SVC or AVC. Non-Zero means AVC or if ZERO its
 *                               SVC. NOTE: This field is don't care if numberOfLayers is greater than ONE.
 *
*/

typedef struct {
    IVIDENC2_Status videnc2Status;
    IH264ENC_Status h264Status[IH264SVCENC_MAX_NUM_MULTI_INSTANCE];

    XDAS_UInt8 numberOfLayers;
    XDAS_UInt8 avcMode;

} IH264SVCENC_Status;

/**< This structure must be the first field of all SVCENC instance objects */
typedef struct IH264SVCENC_Obj {
    struct IH264SVCENC_Fxns *fxns;
} IH264SVCENC_Obj;

/**< This handle is used to reference all SVCENC instance objects */
typedef struct IH264SVCENC_Obj *IH264SVCENC_Handle;

/**<Default parameter values for SVCENC instance objects */
extern const IH264SVCENC_Params    H264SVCENC_TI_PARAMS;


/**<
 *
 *  @struct IH264SVCENC_DynamicParams
 *  @brief This structure defines the run time parameters for all SVCENC objects
 *  @param  h264DynamicParams    Dynamic Parameters (Extended Class) of the H264 Encoder.
 *                               As defined in the interface file (ih264enc.h) of H264 Encoder.
 *  @param  numberOfLayers       Conveys the information for how many number of layers encoding the SVC
 *                               Encoder is configured. This field is used in reading the number of
 *                               entries of h264DynamicParams etc.
 *
 *

*/

typedef struct IH264SVCENC_DynamicParams {
    IH264ENC_DynamicParams h264DynamicParams[IH264SVCENC_MAX_NUM_MULTI_INSTANCE];
    XDAS_UInt8             numberOfLayers;
} IH264SVCENC_DynamicParams;

extern const IH264SVCENC_DynamicParams    IH264SVCENC_DYNAMICPARAMS;

/**<

  @struct ISVCENC_InArgs
  @brief  This structure defines the input argument being passed to
          SVC encoder

  @params videnc2InArgs : It is instance of base class. It cntains all
          the necessary  info required run time parameters for all H264ENC
          objects

*/
typedef struct IH264SVCENC_InArgs {

    IVIDENC2_InArgs   videnc2InArgs;
    XDAS_Int32        processId;
    IH264ENC_RoiInput roiInputParams;
    XDAS_UInt32       inputKey;

} IH264SVCENC_InArgs;



/**<

  @struct IH264SVCENC_OutArgs
  @brief  This structure defines the outpur argument being generated from SVC
          encoder

  @params h264OutArgs : Thsi captures the outArgs of the each of the layer encoded.
  @params videnc2OutArgs : It is instance of base class which captures the outArgs
          as a consolidated whole for all the layers.
  @params numberOfLayers: Number of layers that the encoder has been configured to
           encode.This field is used in reading the number of entries of h264OutArgs etc.

*/
typedef struct IH264SVCENC_OutArgs {

    IVIDENC2_OutArgs videnc2OutArgs;
    IH264ENC_OutArgs h264OutArgs[IH264SVCENC_MAX_NUM_MULTI_INSTANCE];
    XDAS_UInt8       numberOfLayers;

} IH264SVCENC_OutArgs;

/**<

  @struct IH264SVCENC_Fxns
  @brief  This structure defines of the operations on SVCENC objects

    @params IVIDENC2_Fxns : It is instance of base class. It contains all
            function table

*/
typedef struct IH264SVCENC_Fxns {
    IVIDENC2_Fxns ividenc;

} IH264SVCENC_Fxns;

#ifdef __cplusplus
}
#endif

/*@}*/ /* ingroup HDVICP2SVC */

#endif  //_IH264SVCENC_H_ //--}

/* ========================================================================*/
/* End of file : ih264svcenc.h                                             */
/*-------------------------------------------------------------------------*/
/*            Copyright (c) 2009 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ========================================================================*/

