/*
 * Copyright (c) 2010, Texas Instruments Incorporated
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

/*
*******************************************************************************
 * HDVICP2.0 Based H.264SVC Decoder
 *
 * "HDVICP2.0 Based H.264SVC Decoder" is a software module developed on TI's
 *  HDVICP2 based SOCs. This module is capable of decoding a compressed
 *  baseline profile SVC bit-stream into a YUV 4:2:0 Raw video.
 *  Based on "Annex G ISO/IEC 14496-10".
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
*******************************************************************************
*/
/**
*******************************************************************************
 * @file   <ih264svcvdec.h>
 *
 * @brief  SVCVDEC Interface Header file.
 *
 *         This File contains the interface structures and Macro definition
 *         required for integrating the H264VDEC.
 *
 * @author:  Ashish Singh  <ashish.singh@ti.com>
 *
 * @version 0.1 (Apr 2010) : Base version borrowed from h264 universal decoder
 *
 * @version 0.2 (Apr 2010) : Implemented the interface structure for svc dec
 *
 * @version 0.3 (Sep 2010) : Added review comments and added one new default
 *                           dynemic parameter [Ashish  Singh ]
 *
 * @version 0.4 (Jan 2011) : Added error concealment mode exteded parameter
 *                           in create time parameter and defined the enum
 *                           for concealment modes [Ashish Singh]
 *
 * @version 0.5 (March 2011): Added spatial error concealment mode in enum
 *                            define for concealment modes[Ashish Singh]
 *
 * @version 0.6 (April 2011): Added some more extended parametrs and also
 *                            added some reserve parametrs in status/create
 *                            dynemic structures [Ashish Singh]
 * @version 0.7 (April 2011): Added one unregistred SSEI message for TI
 *                            SVC encoder encoded stream[Ashish Singh]
 *
 ******************************************************************************
*/

#ifndef IH264SVCVDEC_
#define IH264SVCVDEC_

#include <ti/xdais/ialg.h>
#include <ti/xdais/dm/ividdec3.h>
#include <ih264vdec.h>
#include <h264vdec_ti.h>


/**
  * Macro defined for maximum scalable layer in access unit supported by codec
  */
#define IH264SVCVDEC_MAX_NUM_LAYER 0x9
/**
  * Macro defined for getting out the version of the encoder written
  * Unregistred User SSEI messages
  */
#define VERSION_FIELD 0x4
/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_Obj
 *
 *  @brief  Modules object definition. This structure must be the first field
 *          of all H264SVCVDEC instance objects
 *
 *  @param  fxns : Structure type defining all functions necessary to
 *                 implement the IH264VDEC interface.
 ******************************************************************************
*/
typedef struct IH264SVCVDEC_Obj {
    struct IH264SVCVDEC_Fxns *fxns;
} IH264SVCVDEC_Obj;

/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_Status
 *
 *  @brief  This structure defines parameters that describe the status of the
 *          SVC Decoder and any other implementation specific parameters.
 *          The status parameters are defined in the XDM data structure,
 *          IVIDDEC3_Status
 *
 *  @param  h264SvcDecStatusParams : array to hold the status of each layer in
 *                    scalable stream
 *  @param  viddec3Status           : XDM Base class status structure
 *
 *  @param  errConcealmentMode      : Error Concealment applied by H.264SVC
 *                                    Decoder
 *  @param  numLayersPresent        : Number of scalable layers present in AU
 *
 *  @param  layerDID                : Array to store the spatial layer ID's
 *                                    present in AU
 *  @param  layerTID                : Array to store the Temporal layer ID
 *                                    present in AU
 *  @param  layerQID                : Array to store the Quality layer ID's
 *                                    present in AU
 *  @param  reserved                : Reserve for future uses
 *
 ******************************************************************************
*/
typedef struct IH264SVCVDEC_Status {
    IH264VDEC_Status h264SvcDecStatusParams;
    IVIDDEC3_Status  viddec3Status;
    XDAS_UInt32      errConcealmentMode;
    XDAS_UInt32      numLayersPresent;
    XDAS_Int32       layerDID[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_Int32       layerTID[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_Int32       layerQID[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt32      reserved[3];
} IH264SVCVDEC_Status;


/**
******************************************************************************
*  @struct IH264SVCVDEC_Params
*
*  @brief  This structure defines the creation parameters for all H264SVCVDEC
*          objects inside H264SVCVDEC object. This structure includes the
*      xdm baseclass creation parameters and any other implementation
*      specific parameters for SVC Decoder instance object.
*
*  @param  svcDecLayerParams   : array to hold the init time parameter of
*                                 each layer in scalable stream
*  @param  disablePreParsing   : Flag for disable or enable the pre-
*                                Parsing of the svc stream
*
*  @param  reserved            : Reserve for future uses
******************************************************************************
*/

typedef struct IH264SVCVDEC_Params {
    IH264VDEC_Params h264SvcDecLayerParams;
    XDAS_UInt32      disablePreParsing;
    XDAS_UInt32      reserved[3];
} IH264SVCVDEC_Params;

/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_DynamicParams
 *
 *  @brief  This structure defines the run-time parameters and any other
 *          implementation specific parameters for an SVC instance object.
 *          The base run-time parameters are defined in the XDM data structure,
 *          IVIDDEC3_DynamicParams.
 *
 *  @param  h264SvcDecDynamicParams : array to store the hold the dynemic para
 *                     meter for each layer of scalable stream
 *
 *  @param  reserved            : Reserve for future uses
 *
 ******************************************************************************
*/

typedef struct IH264SVCVDEC_DynamicParams {
    IH264VDEC_DynamicParams h264SvcDecDynamicParams;
    XDAS_UInt32             reserved[3];
} IH264SVCVDEC_DynamicParams;

/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_InArgs
 *
 *  @brief  This structure defines the run-time input arguments for an H264
 *          instance object (ISVCH264VDEC::process)
 *
 *  @param  viddec3InArgs : InArgs structure SVC Object
 ******************************************************************************
*/
typedef struct IH264SVCVDEC_InArgs {
    IH264VDEC_InArgs h264SvcDecLayerInArgs;
} IH264SVCVDEC_InArgs;

/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_OutArgs
 *
 *  @brief  This structure defines the run time output arguments for
 *          ISVCH264VDEC::process function
 *
 *  @param  svcDecLayerOutArgs : OutArgs structure SVC Object
 ******************************************************************************
*/

typedef struct IH264SVCVDEC_OutArgs {
    IH264VDEC_OutArgs h264SvcDecLayerOutArgs;
} IH264SVCVDEC_OutArgs;

/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_Fxns
 *
 *  @brief  This structure contains pointers to all the XDAIS and XDM interface
 *          functions
 *
 *  @param  ividdec3 : This structure contains pointers to all the XDAIS and
 *                     XDM interface functions
 ******************************************************************************
*/
typedef struct IH264SVCVDEC_Fxns {
    IVIDDEC3_Fxns ividdec3;
} IH264SVCVDEC_Fxns;

/*
 *  IH264SVCVDEC_Handle
 *  This handle is used to reference all H264SVCVDEC instance objects
 */
typedef struct IH264SVCVDEC_Obj *IH264SVCVDEC_Handle;

/*
 *  IH264SVCVDEC_Cmd
 *  This structure defines the control commands for the IMP4VENC module.
 */
typedef IVIDDEC3_Cmd IH264SVCVDEC_Cmd;

/*
 *  IH264SVCVDEC_PARAMS
 *  Default parameter values for H264SVCVDEC instance objects
 */
extern IH264SVCVDEC_Params    IH264SVCVDEC_PARAMS;
/*
 *  IH264SVCVDEC_TI_DYNAMICPARAMS
 *  Default dynamic parameter values for H264SVCVDEC instance objects
 */
extern IH264SVCVDEC_DynamicParams    IH264SVCVDEC_TI_DYNAMICPARAMS;

/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_Scalability_TI_Info
 *
 *  @brief   This structure contains svc dec supported SEI message syntax
 *           elements
 *
 *  @param  parsed_flag :1 - Indicates that in the current process call, c
 *                            contents of the structure is updated
 *                       0 - Indicates contents of the structure is not updated
 *
 *  @param  num_layers_minus1:(num_layers_minus1 + 1)specifies the number of
 *                scalable layers for which information is provided
 *                            in the scalability information SEI message
 *  @param  layer_id :        specifies the layer identifier of the i-th
 *                 scalable layer specified in the scalability information
 *                 SEI message
 *  @param  priority_id :  indicates an upper bound for the priority_id values
 *               of the current scalable layer representation
 *  @param  dependency_id : is equal to the values of dependency_id,of the VCL
 *              NAL units of the current scalable layer.
 *  @param  quality_id :is equal to the values of quality_id,of the VCL
 *              NAL units of the current scalable layer.
 *  @param  temporal_id :is equal to the values of temporal_id,of the VCL
 *              NAL units of the current scalable layer.
 *  @param  bitrate_info_present_flag: specifies that the bit rate
 *                     information for the current scalable
 *                     layer representation is present/
 *                     not present in the
 *                     scalability information SEI message
 *  @param  frm_rate_info_present_flag:specifies that the frame rate
 *                         information for the current scalable
 *                                     layer representation is present/not
 *                                     present in the scalability information
 *                                     SEI message
 *  @param  frm_size_info_present_flag: specifies that the frame size
 *                    information for the current scalable
 *                    layer representation is present/not
 *                      present in the scalability information
 *                    SEI message
 *  @param  avg_bitrate: indicates the average bit rate of the representation of
 *             the current scalable layer
 *  @param  max_bitrate_layer:indicates an upper bound for the bit rate of the
 *                current scalable layer in any fixed size time
 *                window
 *  @param  max_bitrate_layer_representation: indicates an upper bound for the
 *                          bit rate of the current scalable
 *                        layer representation in any fixed
 *                        size time window
 *  @param  max_bitrate_calc_window: specifies the size of the time window that
 *                     is used for calculating the upper bounds
 *                   for the bit rate of the current scalable
 *                     layer
 *  @param  constant_frm_rate_idc: indicates whether the frame rate of the
 *                   current scalable layer representation is
 *                     constant
 *  @param  avg_frm_rate:indicates the average frame rate, in units of frames
 *      per 256 seconds, of the representation of the current scalable
 *      layer
 *  @param  frm_width_in_mbs_minus1: indicate the width of the decoded pictures
 *                   for the current scalable layer
 *                     representation
 *  @param  frm_height_in_mbs_minus1:indicate the height of the decoded
 *                   pictures for the current scalable
 *                   layer representation
 *
 ******************************************************************************
*/

typedef struct IH264SVCVDEC_Scalability_TI_Info {
    XDAS_UInt32 parsed_flag;
    XDAS_UInt32 num_layers_minus1;
    XDAS_UInt32 layer_id[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  priority_id[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  dependency_id[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  quality_id[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  temporal_id[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  bitrate_info_present_flag[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  frm_rate_info_present_flag[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  frm_size_info_present_flag[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt16 avg_bitrate[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt16 max_bitrate_layer[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt16 max_bitrate_layer_representation[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt16 max_bitrate_calc_window[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt16 constant_frm_rate_idc[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt16 avg_frm_rate[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt32 frm_width_in_mbs_minus1[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt32 frm_height_in_mbs_minus1[IH264SVCVDEC_MAX_NUM_LAYER];
}IH264SVCVDEC_Scalability_TI_Info;

/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_UnRegSeiMessages_TI_Info
 *
 *  @brief   This structure contains Unregistred sei message encoded by the
 *           Texas Instruments svc netra encoder
 *
 *  @param  parsed_flag :1 - Indicates that in the current process call, c
 *                            contents of the structure is updated
 *                       0 - Indicates contents of the structure is not updated
 *
 *  @param  layerOffset: array to store offset of the position of the scalable
 *                       layer(SPS/SSPS/PPS/SLICE) encoded in
 *                       current access unit
 *
 *  @param  layerSize:   array to store the encoded each layer size
 *
 *  @param  num_layers:  Number of layers encoded in the current access unit
 *
 *  @param  dependency_id: array to store the spatial layer ID
 *
 *  @param  quality_id:    array to store the quality layer ID
 *
 *  @param  idr_flag : array to store the idr_flag info for each layer present
 *                     in access unit
 *
 *  @param  ti_specific_unregiSeiMessage_version  :
 *                                  version of the unregiustred sei message
 *                                  write by encoder for improving the decoder
 *                                  performance
 *
 *  @param  ti_specific_unregiMessagesFlag: Flag to tells that unregistred
 *                                          sei meesage is present in the stream
 *
 ******************************************************************************
*/

typedef struct IH264SVCVDEC_UnRegSeiMessages_TI_Info {
    XDAS_UInt32 parsed_flag;
    XDAS_UInt32 layerOffset[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt32 layerSize[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt16 num_layers;
    XDAS_UInt8  dependency_id[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  quality_id[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  idr_flag[IH264SVCVDEC_MAX_NUM_LAYER];
    XDAS_UInt8  ti_specific_unregiSeiMessage_version;
    XDAS_UInt8  ti_specific_unregiSeiMessagesFlag;
}IH264SVCVDEC_UnRegSeiMessages_TI_Info;

/**
 ******************************************************************************
 *  @struct IH264SVCVDEC_SSeiMessages
 *
 *  @brief   This structure contains all the supported SEI msg by svc decoder
 *
 *  @param  parsed_flag :1 - Indicates that in the current process call, c
 *                            contents of the structure is updated
 *                       0 - Indicates contents of the structure is not updated
 *
 *  @param  scalability_info : structure instance for scalabality info SEI
 *                 messages
 *
 ******************************************************************************
*/

typedef struct IH264SVCVDEC_SSeiMessages {
    XDAS_UInt32                           parsed_flag;
    IH264SVCVDEC_Scalability_TI_Info      scalability_info;
    IH264SVCVDEC_UnRegSeiMessages_TI_Info unregistersei_info;
}IH264SVCVDEC_SSeiMessages;


/**
 ******************************************************************************
 *  @enum       IH264VDEC_errConcealmentMode
 *  @brief      This enum indicates whether to apply error concealment or not
 *
 ******************************************************************************
*/
typedef enum {
    IH264SVCVDEC_NO_CONCEALMENT = 0,
    /**
    *  do not apply error concealment
    */
    IH264SVCVDEC_APPLY_H264_ERRCONCEALMENT,
    /**
    * apply error concealment of h264 decoder
    */
    IH264SVCVDEC_APPLY_SVC_ERRCONCEALMENT,
    /**
      * apply error concealment of svc decoder
      */
    IH264SVCVDEC_APPLY_SVC_SPATIAL_ERRCONCEALMENT
    /**
      * apply error concealment of svc decoder
      */

} IH264SVCVDEC_errConcealmentMode;

/**
 ******************************************************************************
 *  @enum       IH264SVCVDEC_preParsingMode
 *  @brief      This enum indicates whether to enable or disable preparsing
 *
 ******************************************************************************
*/
typedef enum {

    IH264SVCVDEC_ENABLE_PREPARSING = 0,
    /**
    *  Enable the preparsing, mostaly in error prone scenario
    */
    IH264SVCVDEC_DISABLE_PREPARSING = 1
                                      /**
                                      *  Disalbe  the preparsing, for video survilence scenarion, where
                                      *  drop of frame/slice don't happens
                                      */
}IH264SVCVDEC_preParsingMode;


#endif             /* IH264SVCVDEC_ */

