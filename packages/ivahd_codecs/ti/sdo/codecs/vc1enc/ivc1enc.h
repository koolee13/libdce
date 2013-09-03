/*
 *******************************************************************************
 *
 * HDVICP2.0 Based VC1 Encoder
 *
 * "HDVICP2.0 Based VC1 Encoder" is software module developed on TI's
 *  HDVICP2 based SOCs. This module is capable of compressing a 4:2:0 Raw
 *  video into a simple/main/advanced profile bit-stream. Based on SMPTE 421M."
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *******************************************************************************
*/

/*!
 *****************************************************************************
 * @file  ivc1enc.h
 *
 * @brief Interface header file
 *
 * @version 0.1 (Nov 2011) : Initial version [Prasad]
 *
 *****************************************************************************
 */

#ifndef _IVC1ENC_H_
#define _IVC1ENC_H_

#include <ti/xdais/ialg.h>
#include <ti/xdais/dm/ividenc2.h>


#ifdef __cplusplus
extern "C" {
#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Definition of all the macros define by this interafce       */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
  Maximum Number of slice start points
*/
#define IVC1ENC_MAX_NUM_SLICE_START_OFFSET  (3)
/**
  Length of the version string. The memory to get version
  number is owned by application
*/
#define IVC1ENC_VERSION_LENGTH (64)

/**
  control method commands
*/
#define IVC1ENC_GETSTATUS      XDM_GETSTATUS
#define IVC1ENC_SETPARAMS      XDM_SETPARAMS
#define IVC1ENC_RESET          XDM_RESET
#define IVC1ENC_FLUSH          XDM_FLUSH
#define IVC1ENC_SETDEFAULT     XDM_SETDEFAULT
#define IVC1ENC_GETBUFINFO     XDM_GETBUFINFO

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Definition of all the Enumeration define by this interafce  */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 *  @enum       IVC1ENC_ErrorBit
 *  @brief      error informations of IVAHD VC1 encoder implementation by TI.
*/
typedef enum {
    IVC1ENC_LEVEL_INCOMPLIANT_PARAMETER = 0,
    /**< Bit 0 - level incomplaint parameters.
    *   @remarks  This error is applicable when some parameters are set
    *   which are not meeting the limit defined by the VC1 standard.
    *   Level limits can be categorized under
    *   following category :
    *   IVC1ENC_LEVEL_INCOMPLIANT_RESOLUTION : Invalid width/height
    *   IVC1ENC_LEVEL_INCOMPLIANT_HRDBUFSZIE : Invalid HrdBufferSize
    *   IVC1ENC_LEVEL_INCOMPLIANT_BITRATE    : Invalid Bit Rate
    *   For above 3 situations, only a signle bit (bit-0) is set as true
    */
    IVC1ENC_PROFILE_INCOMPLIANT_CONTENTTYPE = 1,
    /**< Bit 1 - Profile incomplaint content type.
    *   @remarks  This error is applicable when
    *   IVIDENC2_Params::inputContentType is set as
    *   IVIDEO_INTERLACED but IVIDENC2_Params::profile is not set
    *   as IVC1_ADVANCED_PROFILE
    */
    IVC1ENC_PROFILE_INCOMPLIANT_INTERFRAMEINTERVAL = 4,
    /**< Bit 4 - Profile incomplaint interframeInterval.
    *   @remarks  This error is set when B frames are used with
    *   IVC1_SIMPLE_PROFILE
    */
    IVC1ENC_MAX_BIT_RATE_VIOLATION = 7,
    /**< Bit 7 - Max bits for one Unit Voilation
     * @remarks When max bit rate is enabled by user,
     * than it is possible that codec might not be able
     * honor max bit rate. This bit is set when bits consumed
     * in one unit ( 1 sec) is more than the allocated as per the
     * given max bit rate. If the frame rate is N , and if the
     * max bit rate is voilated in M th frame than this bit will
     * get set for frame M to N.
     */
    IVC1ENC_IMPROPER_HDVICP2_STATE = 16,
    /**< Bit 16 - HDVICP2 is not in proper state.
    */
    /*IVC1ENC_IMPROPER_STREAMFORMAT = 17, */
    /**< Bit 17 - stream format is not proper
    *   @remarks  This error is set when streamFormat is set
    *   other than  or
    */
    IVC1ENC_UNSUPPORTED_VIDENC2PARAMS = 20,
    /**< Bit 20 - Invalid videnc2 parameters
    *   @remarks  This error is set when any parameter of struct
    *   IVIDENC2_Params is not in allowed range
    */
    IVC1ENC_UNSUPPORTED_RATECONTROLPARAMS = 21,
    /**< Bit 21 - Invalid rate control parameters
    *   @remarks  This error is set when any parameter of struct
    *   IVC1ENC_RateControlParams is not in allowed range
    */
    IVC1ENC_UNSUPPORTED_INTERCODINGPARAMS = 22,
    /**< Bit 22 - Invalid inter coding parameters
    *   @remarks  This error is set when any parameter of struct
    *   IVC1ENC_InterCodingParams is not in allowed range
    */
    IVC1ENC_UNSUPPORTED_INTRACODINGPARAMS = 23,
    /**< Bit 23 - Invalid Intra coding parameters
    *   @remarks  This error is set when any parameter of struct
    *   IVC1ENC_IntraCodingParams is not in allowed range
    */
    IVC1ENC_UNSUPPORTED_SLICECODINGPARAMS = 25,
    /**< Bit 25 - Invalid slice coding parameters
    *   @remarks  This error is set when any parameter of struct
    *   IVC1ENC_SliceCodingParams is not in allowed range
    */
    IVC1ENC_UNSUPPORTED_LOOPFILTERPARAMS = 26,
    /**< Bit 26 - Invalid loop filter related parameters
    *   @remarks  This error is set when any parameter of struct
    *   IVC1ENC_LoopFilterParams is not in allowed range
    */
    IVC1ENC_UNSUPPORTED_VC1ENCPARAMS = 29,
    /**< Bit 29 - Invalid Create time extended parameters
    *   @remarks  This error is set when any parameter of struct
    *   IVC1ENC_Params is not in allowed range
    */
    IVC1ENC_UNSUPPORTED_VIDENC2DYNAMICPARAMS = 30,
    /**< Bit 30 - Invalid base class dyanmic paaremeters during control
    *   @remarks  This error is set when any parameter of struct
    *   IVIDENC2_DynamicParams is not in allowed range
    */
    IVC1ENC_UNSUPPORTED_VC1ENCDYNAMICPARAMS = 31
                                              /**< Bit 31 - Invalid exteded class dynamic parameters during control
                                              *   @remarks  This error is set when any parameter of struct
                                              *   IVC1ENC_DynamicParams (excluding embedded structures) is not in
                                              *    allowed range
                                              */
} IVC1ENC_ErrorBit;

/**
 *  @enum       IVC1ENC_Level
 *  @brief      Level Identifier for VC1 Encoder
*/
typedef enum {
    IVC1_SP_LOW  = 0, /**< Simple Profile Level Low     */
    IVC1_SP_MED  = 2, /**< Simple Profile Level Medium  */
    IVC1_MP_LOW  = 0, /**< Main Profile Level Low       */
    IVC1_MP_MED  = 2, /**< Main Profile Level Medium    */
    IVC1_MP_HIGH = 4, /**< Main Profile Level High      */
    IVC1_AP_L0   = 0, /**< Advanced Profile Level L0    */
    IVC1_AP_L1   = 1, /**< Advanced Profile Level L1    */
    IVC1_AP_L2   = 2, /**< Advanced Profile Level L2    */
    IVC1_AP_L3   = 3, /**< Advanced Profile Level L3    */
    IVC1_AP_L4   = 4 /**< Advanced Profile Level L4    */

} IVC1ENC_Level;


/**
 *  @enum       IVC1ENC_Profile
 *  @brief      Profile Identifier for VC1 Encoder
*/
typedef enum {
    IVC1_SIMPLE_PROFILE   = 0, /**< Simple Profile   */
    IVC1_MAIN_PROFILE     = 1, /**< Main Profile     */
    IVC1_ADVANCED_PROFILE = 3 /**< Advanced Profile */

} IVC1ENC_Profile;

/**

  @enum   IVC1ENC_RateControlParamsPreset
  @brief  These enumerations control the RateControl Params

*/

typedef enum {
    IVC1_RATECONTROLPARAMS_DEFAULT     = 0,
    /**< Default Rate Control params */
    IVC1_RATECONTROLPARAMS_USERDEFINED = 1,
    /**< User defined Rate Control params */
    IVC1_RATECONTROLPARAMS_EXISTING    = 2,
    /**< Keep the Rate Control params as existing. This is
    * useful because during control call if user doesn't want
    * to change the Rate Control Params
    */
    IVC1_RATECONTROLPARAMS_MAX

} IVC1ENC_RateControlParamsPreset;

/**

  @enum   IVC1ENC_RateControlAlgo
  @brief  These enumerations control the type of rateControl algo to be picked
          up by encoder. Only useful if IVIDENC2::rateControlPreset is set as
          IVIDEO_USER_DEFINED

*/
typedef enum {
    IVC1_RATECONTROL_PRC               = 0,       /**< Perceptual Rate Control,
                                                   * controls the QP @ MB level
                                                   */
    IVC1_RATECONTROL_PRC_LOW_DELAY     = 1,       /** Low Delay Rate Control */
    IVC1_RATECONTROL_DEFAULT = IVC1_RATECONTROL_PRC /** Default rcAlgo is PRC  */

} IVC1ENC_RateControlAlgo;

/**

  @enum   IVC1ENC_InterCodingPreset
  @brief  These enumerations control the type of inter coding

*/

typedef enum {
    IVC1_INTERCODING_DEFAULT     = 0, /**< Default Inter coding params      */
    IVC1_INTERCODING_USERDEFINED = 1, /**< User defined inter coding params */
    IVC1_INTERCODING_EXISTING    = 2, /**< Keep the inter coding params as
                                       *  existing. This is useful because
                                       *  during control call if user don't
                                       *  want to change the inter coding Params
                                       */
    IVC1_INTERCODING_MAX

} IVC1ENC_InterCodingPreset;

/**

  @enum   IVC1ENC_InterBlockSize
  @brief  These enumerations are defined for minimum Inter block size

*/

typedef enum {
    IVC1_BLOCKSIZE_16x16     = 0,                   /**< 16x16 Block size    */
    IVC1_BLOCKSIZE_DEFAULT   = IVC1_BLOCKSIZE_16x16, /**< Default block size  */
    IVC1_BLOCKSIZE_8x8       = 1,                   /**< 8x8 Block size      */
    IVC1_BLOCKSIZE_4x4       = 2,                   /**< 4x4 Block size      */
    IVC1_BLOCKSIZE_MAX

} IVC1ENC_InterBlockSize;

/**

  @enum   IVC1ENC_IntraRefreshMethods
  @brief  Refresh method Type Identifier for VC1 Encoder

*/

typedef enum {
    IVC1_INTRAREFRESH_NONE       = 0,  /**< Doesn't insert forcefully intra
                                            macro blocks */
    IVC1_INTRAREFRESH_DEFAULT    = IVC1_INTRAREFRESH_NONE,
    /**< Default intra refresh is OFF */
    IVC1_INTRAREFRESH_CYCLIC_MBS,      /**< Insters intra macro blocks in a
                                       * cyclic fashion cyclic interval is
                                       * equal to intraRefreshRate
                                       */
    IVC1_INTRAREFRESH_CYCLIC_SLICES,   /**< Insters Intra Slices(Row based) in
                                       * a cyclic fashion:
                                       * cyclic interval is equal to
                                       * intraRefreshRate
                                       */
    IVC1_INTRAREFRESH_RDOPT_MBS,       /**< position of intra macro blocks is
                                       * intelligently  chosen by encoder,
                                       * but the number of forcely coded
                                       * intra macro blocks in a frame is
                                       * guaranteed to be equal to
                                       * totalMbsInFrame/intraRefreshRate
                                       */
    IVC1_INTRAREFRESH_MAX

} IVC1ENC_IntraRefreshMethods;

/**

  @enum   IVC1ENC_IntraCodingPreset
  @brief  These enumerations control the type of intra coding

*/

typedef enum {
    IVC1_INTRACODING_DEFAULT     = 0, /**< Default intra coding params      */
    IVC1_INTRACODING_USERDEFINED = 1, /**< User defined intra coding params */
    IVC1_INTRACODING_MAX

} IVC1ENC_IntraCodingPreset;

/**
  @enum   IVC1ENC_SliceCodingPreset
  @brief  These enumerations control the type of slice coding
          Slice coding is allowed only in Advanced profile.
*/

typedef enum {
    IVC1_SLICECODING_DEFAULT     = 0,
    /**< Default slice coding params                        */
    IVC1_SLICECODING_USERDEFINED = 1,
    /**< User defined slicecoding params                    */
    IVC1_SLICECODING_EXISTING    = 2,
    /**< Keep the slice coding params as existing           */
    /**< This is useful because during control call         */
    /**< if user don't want to change the sliceCodingParams */
    IVC1_SLICECODING_MAX

} IVC1ENC_SliceCodingPreset;

/**
  @enum   IVC1ENC_SliceMode
  @brief  These enumerations control the type of slice coding
          Slice coding is allowed only in Advanced profile.
*/

typedef enum {
    IVC1_SLICEMODE_NONE    = 0,
    IVC1_SLICEMODE_DEFAULT = IVC1_SLICEMODE_NONE,
    /**< Default slice coding mode is disabled */
    IVC1_SLICEMODE_ROWS    = 1,
    /**< Slices are controlled based upon number of Rows */
    IVC1_SLICEMODE_OFFSET  = 2,
    /**< Slices are controlled based upon user defined offset in
    * unit of Rows
    */
    IVC1_SLICEMODE_MAX

} IVC1ENC_SliceMode;

/**

  @enum   IVC1ENC_StreamFormat
  @brief  These enumerations control the type of stream format
          Only applicable for simple and main profile streams.
*/
typedef enum {
    IVC1_RAW_FORMAT = 0,
    /**< bit-stream does not contain the RCV headers */
    IVC1_RCV1_FORMAT = 1,
    /**< bit-stream contain the RCV headers in Version 1 format */
    IVC1_RCV2_FORMAT = 2,
    /**< bit-stream contain the RCV headers in Version 2 format */
    IVC1_STREAM_FORMAT_DEFAULT = IVC1_RCV2_FORMAT,
    /**< Default stream format is RCV2 format */
    IVC1_STREAM_FORMAT_MAX
} IVC1ENC_StreamFormat;

/**
 * @enum   IVC1ENC_LoopFilterPreset
 * @brief  These enumerations control the loop filter params
*/

typedef enum {
    IVC1_LOOPFILTER_DEFAULT     = 0, /**< Default loop-filtering params      */
    IVC1_LOOPFILTER_USERDEFINED = 1, /**< User defined loop-filtering params */
    IVC1_LOOPFILTER_MAX
} IVC1ENC_LoopFilterPreset;

/**

  @enum   IVC1ENC_LoopFilterDisableIDC
  @brief  Control Parameter to enable or disable loop filter

*/
typedef enum {
    IVC1_DISABLE_FILTER_NONE = 0,
    /**< Enable filtering of all the edges */
    IVC1_DISABLE_FILTER_DEFAULT   = IVC1_DISABLE_FILTER_NONE,
    /**< Default is Loop filter enabled    */
    IVC1_DISABLE_FILTER_ALL_EDGES,
    /**< Disable filtering of all the edges */
    IVC1_DISABLE_FILTER_MAX
} IVC1ENC_LoopFilterDisableIDC;

/**

  @enum   IVC1ENC_GOPStructure
  @brief
  When B frames are used (InterFrameInterval > 1) then the arrangement of
  frames can be different

  GOP structure in display order as indicated below
  If contentType = Frame
  IVC1ENC_GOPSTRUCTURE_NONUNIFORM : IBBPBBP. .
  IVC1ENC_GOPSTRUCTURE_UNIFORM : BBIBBPBB. .
  If contentType = Field
  IVC1ENC_GOPSTRUCTURE_NONUNIFORM : IPBBBBPBBBB
  IVC1ENC_GOPSTRUCTURE_UNIFORM : BBBBIPBBBBPPBBBB

*/

typedef enum {
    IVC1ENC_GOPSTRUCTURE_NONUNIFORM  = 0,
    /**< Open Gop structure  : IBBPBBP */
    IVC1ENC_GOPSTRUCTURE_DEFAULT  = IVC1ENC_GOPSTRUCTURE_NONUNIFORM,
    /**< Default is open gop structure */
    IVC1ENC_GOPSTRUCTURE_UNIFORM = 1,
    /**< Close Gop structure : BBIBBPBB*/
    IVC1ENC_GOPSTRUCTURE_MAX

} IVC1ENC_GOPStructure;

/**

  @enum   IVC1ENC_BiasFactor
  @brief  Encoder uses bias b/w two possible choices for lot of decisions.
  It is to control the mild/strongness of the biasing

*/
typedef enum {
    IVC1_BIASFACTOR_LOW      = 0,
    /**< Low biasing                */
    IVC1_BIASFACTOR_MEDIUM   = 1,
    /**< Normal/Med biasing         */
    IVC1_BIASFACTOR_NORMAL   = IVC1_BIASFACTOR_MEDIUM,
    /**< Normal/Med biasing         */
    IVC1_BIASFACTOR_DEFAULT  = IVC1_BIASFACTOR_MEDIUM,
    /**< Default :Normal/Med biasing*/
    IVC1_BIASFACTOR_HIGH     = 2,
    /**< High biasing               */
    IVC1_BIASFACTOR_MILD       = 4, /**< Mild biasing             */
    IVC1_BIASFACTOR_ADAPTIVE   = 5, /**< Adaptive biasing     */
    IVC1_BIASFACTOR_MAX
} IVC1ENC_BiasFactor;


/**
 *  @enum       IVC1ENC_InterlaceCodingType
 *  @brief      Profile Identifier for VC1 Encoder
*/
typedef enum {
    IVC1_INTERLACE_PICAFF        = 0,
    /**< PicAFF type of interlace coding            */
    IVC1_INTERLACE_FIELDONLY     = 2,
    /**< Field only coding with fixed partiy scheme */
    IVC1_INTERLACE_FIELDONLY_MRF = IVC1_INTERLACE_FIELDONLY,
    /**< Use Most recent field for refernece        */
    IVC1_INTERLACE_FIELDONLY_ARF = 3,
    /**< codec decides the partiy of of the field to
      *  be used based upon content (adaptive)      */
    IVC1_INTERLACE_DEFAULT       = IVC1_INTERLACE_FIELDONLY_ARF,
    /**< Default : adaptive partiy for reference    */
    IVC1_INTERLACE_FIELDONLY_SPF = 4,
    /**< Use same parity field for refernece        */
    IVC1_INTERLACE_MAX

} IVC1ENC_InterlaceCodingType;

/**

  @enum   IVC1ENC_FrameQualityFactor
  @brief  These enumerations control the quality factor b/w two types of frames
          For example if user want I frame Quality to be given more importance
          than P frame, one can define it to be higher quality factor

*/
typedef enum {
    IVC1_QUALITY_FACTOR_1  = 0,  /**< Same Quality factor
                                  * b/w two types of frame
                                  */
    IVC1_QUALITY_FACTOR_DEFAULT  = IVC1_QUALITY_FACTOR_1,
    /**< Default Quality factor
    * to be used by encoder
    */
    IVC1_QUALITY_FACTOR_2  = 1,  /**< High Quality factor to
                                  * one frame type b/w two types
                                  of frame
                                  */
    IVC1_QUALITY_FACTOR_3  = 2,  /**< Higher Quality factor to
                                  one frame type b/w two types of frame
                                  */
    IVC1_QUALITY_FACTOR_MAX

} IVC1ENC_FrameQualityFactor;

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Definition of all the structures define by this interafce   */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**

  @struct IVC1ENC_RateControlParams
  @brief  This structure contains all the parameters which controls Rate
          Control behavior

  @param  rateControlParamsPreset :
          regarded @ IVC1ENC_DynamicParams::rateControlParams
          This Preset controls the USER_DEFINED vs DEFAULT mode. if User is
          not aware about following fields, it should be set as
          IVC1_RATECONTROLPARAMS_DEFAULT

  @param  rcAlgo  : ignored @ IVC1ENC_DynamicParams::rateControlParams
          This defines the rate control algorithm to be used. Only useful
          if IVIDENC2::rateControlPreset is set as IVIDEO_USER_DEFINED

  @param  qpI  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Initial Quantization Parameter for I frames.
          Valid Range is [-1,31]
          1 : Auto Initialization else other wise Initial QP.
          when rateControlPreset = IVIDEO_NONE, this quantization parameter is
          used by the whole video frame/field

  @param  qpMaxI  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Maximum Quantization Parameter for I frame(s). Range [-1,31]
          Useful to control a minimum quality level

  @param  qpMinI  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Minimum Quantization Parameter for I frame(s). Range [-1,31]
          Useful to control a maximum bit-rate level

  @param  qpP  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Initial Quantization Parameter for P frames. Valid Range is [-1,31]
          1 : Auto Initialization else other wise Initial QP.
          when rateControlPreset = IVIDEO_NONE, this quantization parameter is
          used by the whole video frame/field

  @param  qpMaxP  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Maximum Quantization Parameter for inter frame(s). Range [-1,31]
          Useful to control a minimum quality level

  @param  qpMinP  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Minimum Quantization Parameter for inter frame(s). Range [-1,31]
          Useful to control a maximum bit-rate level

  @param  qpOffsetB  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Offset of B frames Quantization Parameter from P frames.
          Valid Range is [-1,31]
          1 : Auto Initialization else other wise user provided offset
          if after adding the qpOffsetB into qp of P frame it exceeds 31 then
          it is clipped to 31
          when rateControlPreset = IVIDEO_NONE, this offset parameter is
          used by the whole video frame/field

  @param  qpMaxB  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Maximum Quantization Parameter for B frame(s). Range [-1,31]
          Useful to control a minimum quality level

  @param  qpMinB  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Minimum Quantization Parameter for B frame(s). Range [-1,31]
          Useful to control a maximum bit-rate level

  @param  IPQualityFactor  : ignored @ IVC1ENC_DynamicParams::rateControlParams
          This provides configurality to control I frame Quality wrt to P frame.
          Higher Quality factor means I frame quality is given higher
          improtance compared to P frame.
          Refer IVC1ENC_FrameQualityFactor for possible values

  @param  initialBufferLevel  :
          ignored @ IVC1ENC_DynamicParams::rateControlParams
          Initial Buffer level for HRD compliance. It informs that Hypothtical
          decoder can start after how much time. The value taken is the
          obsolute value of the HRD buffer size  For example if user want
          Hypothtical decoder to start taking out data from HRD buffer after
          half second then it should set initialBufferLevel = half of the
          HRD buffer size that is programmed.

  @param  HRDBufferSize  : regarded @ IVC1ENC_DynamicParams::rateControlParams
          Hypothetical Reference Decoder Buffer Size. This size controls the
          frame skip  logic of the encoder. for low delay applications this
          size should be small. Unit of this variable is bits

  @param  minPicSizeRatio : regarded @ IVC1ENC_DynamicParams::rateControlParams
                            This ratio is used to compute minimum picture size
                            in the following manner,
                            minPicSize = averagePicSize >> minPicSizeRatio
                            allowed values 1 to 4, Setting this to 0 will enable
                            encoder chosen ratio.
                            Note that this is guided value to rate control to
                            determine min picture size and encoder may not
                            strictly follow this
  @param  maxPicSizeRatio : regarded @ IVC1ENC_DynamicParams::rateControlParams
                            To determines ratio for max picture size
                            This ratio is used to compute maximum picture size
                            in the following manner,
                            maxPicSize = averagePicSize * maxPicSizeRatio
                            allowed values 2 to 30.Setting this to 0 and 1
                            will enable encoder chosen ratio.
                            Note that this is guided value to rate control
                            to determine max picture size and encoder may not
                            strictly follow this.

  @param  enablePRC       : regarded @ IVC1ENC_DynamicParams::rateControlParams
                          This flag is used to control allowing PRC in the frame

  @param  enablePartialFrameSkip : regarded @
                            IVC1ENC_DynamicParams::rateControlParams
                            This flag is used to control allowing partial frame
                            skip in the frame
  @param  reserved : 16 bit word, kept to not change the foot print

  @param  reservedRC
          Some part is kept reserved to add parameters later without
          changing the foot print of  interface memory

  @todo  More parameters to be added : delay (VBV), PRC related etc..

*/

typedef struct IVC1ENC_RateControlParams {
    XDAS_Int8  rateControlParamsPreset;
    XDAS_Int8  rcAlgo;
    XDAS_Int8  qpI;
    XDAS_Int8  qpMaxI;
    XDAS_Int8  qpMinI;
    XDAS_Int8  qpP;
    XDAS_Int8  qpMaxP;
    XDAS_Int8  qpMinP;
    XDAS_Int8  qpOffsetB;
    XDAS_Int8  qpMaxB;
    XDAS_Int8  qpMinB;
    XDAS_Int8  IPQualityFactor;
    XDAS_Int32 initialBufferLevel;
    XDAS_Int32 HRDBufferSize;
    XDAS_Int16 minPicSizeRatio;
    XDAS_Int16 maxPicSizeRatio;
    XDAS_Int8  enablePRC;
    XDAS_Int8  enablePartialFrameSkip;
    XDAS_Int8  discardSavedBits;
    XDAS_Int16 reserved;
    XDAS_Int32 reservedRC[3];
} IVC1ENC_RateControlParams;

/**

  @struct IVC1ENC_InterCodingParams
  @brief  This structure contains all the parameters which controls Inter MBs
          coding behavior
  @param  interCodingPreset
          This Preset controls the USER_DEFINED vs DEFAULT mode. if User is
          not aware about following fields, it should be set as
          IVC1_INTERCODING_DEFAULT
  @param  searchRangeHorP :regarded @ IVC1ENC_DynamicParams::interCodingParams
          Horizontal Search Range for P frames
  @param  searchRangeVerP :regarded @ IVC1ENC_DynamicParams::interCodingParams
          Vertical Search Range for P frames
  @param  searchRangeHorB :regarded @ IVC1ENC_DynamicParams::interCodingParams
          Horizontal Search Range for B frames
  @param  searchRangeVerB :regarded @ IVC1ENC_DynamicParams::interCodingParams
          Vertical Search Range for B frames
  @param  minBlockSize    :regarded @ IVC1ENC_DynamicParams::interCodingParams
          minimum block size for P and B frames. Refer IVC1ENC_InterBlockSize
          enumeration to see the valid values
  @param  skipMVCodingBias:regarded @ IVC1ENC_DynamicParams::interCodingParams
          Bias Control for having a macro block use skip MV vs regular MV
          refer IVC1ENC_BiasFactor for possible values
*/

typedef struct IVC1ENC_InterCodingParams {
    XDAS_Int8  interCodingPreset;
    XDAS_Int16 searchRangeHorP;
    XDAS_Int16 searchRangeVerP;
    XDAS_Int16 searchRangeHorB;
    XDAS_Int16 searchRangeVerB;
    XDAS_Int8  minBlockSize;
    XDAS_Int8  skipMVCodingBias;

} IVC1ENC_InterCodingParams;

/**
  @struct IVC1ENC_IntraCodingParams
  @brief  This structure contains all the parameters which controls Intra
          encoding

  @param  intraCodingPreset
          This Preset controls the USER_DEFINED vs DEFAULT mode. if User is
          not aware about following fields, it should be set as
          INTRA_CODING_DEFAULT other wise INTRA_CODING_USER_DEFINED
  @param  intraRefreshMethod
          Mechanism to do intra Refresh, see IVC1ENC_IntraRefreshMethods
          for valid values
  @param  disableACPrediction
          Flag to disable AC prediction
  @param  intraRefreshRate
          Rate at which intra Refresh is done, This rate is specified as
          One IntraMB per # MBs. For example if rate is 20 it means that
          there has to be  one intra MB(s) per 20 Mbs
*/

typedef struct IVC1ENC_IntraCodingParams {
    XDAS_Int8  intraCodingPreset;
    XDAS_Int8  intraRefreshMethod;
    XDAS_Int8  disableACPrediction;
    XDAS_Int16 intraRefreshRate;

} IVC1ENC_IntraCodingParams;

/**

  @struct IVC1ENC_SliceCodingParams
  @brief  This structure contains all the parameters which controls Slice
          encoding

  @param  sliceCodingPreset
          This Preset controls the USER_DEFINED vs DEFAULT mode. if User is
          not aware about following fields, it should be set as
          IVC1_SLICECODING_DEFAULT

  @param  sliceMode  : regarded @ IVC1ENC_DynamicParams::sliceCodingParams
          This defines the control mechanism to split a picture in slices.
          It can be either MB based or bytes based

  @param  sliceUnitSize  : regarded @ IVC1ENC_DynamicParams::sliceCodingParams
          parameter informs the number of offset information provided by user.
          Actual offset are provided with sliceStartOffset

  @param  insertPicParamsinSlice: Whether to insert picture header at the start
          of each slice.

  @param  sliceStartOffset[IVC1ENC_MAX_NUM_SLICE_START_OFFSET]  : regarded @
          IVC1ENC_DynamicParams::sliceCodingParams row numbering is assumed to
           start from 0. Entries in this array must have numbers in ascending
           order. First slice of the picture is always starting from 0th row
           of the picture so  0th entry is the offset of second slice in picture
          Ex 1 : sliceStartRowNum[0] = 25 ,
                 sliceStartRowNum[1] = 30, sliceStartRowNum[2] = 40
                 will result into 4 slices starting from row# 0, 25, 30 and 40
          Ex 2 : sliceStartRowNum[0] = 25 , sliceStartRowNum[1] = 70,
                 sliceStartRowNum[2] = 60  is invalid
          Ex 3 : sliceStartRowNum[0] = 25 , sliceStartRowNum[1] = 50,
                 sliceStartRowNum[2] = 100
                 will result into 3 slices starting from row# 0, 25 and 50
                 {if number of rows in picture < (100 + 1) }
*/

typedef struct IVC1ENC_SliceCodingParams {
    XDAS_Int8  sliceCodingPreset;
    XDAS_Int16 sliceMode;
    XDAS_Int16 sliceUnitSize;
    XDAS_Int8  insertPicParamsinSlice;
    XDAS_Int8  sliceStartOffset[IVC1ENC_MAX_NUM_SLICE_START_OFFSET];

} IVC1ENC_SliceCodingParams;

/**

  @struct IVC1ENC_LoopFilterParams
  @brief  This structure contains all the parameters which controls loop
          filtering operations

  @param  loopfilterPreset
          This Preset controls the USER_DEFINED vs DEFAULT mode. if User is
          not aware about following fields, it should be set as
          IVC1_LOOPFILTER_DEFAULT
  @param  loopfilterDisableIDC
          Controls VC-1 loop filter disabling options

*/
typedef struct IVC1ENC_LoopFilterParams {
    XDAS_Int8 loopfilterPreset;
    XDAS_Int8 loopfilterDisableIDC;

} IVC1ENC_LoopFilterParams;


/**< This structure must be the first field of all VC1ENC instance objects */
typedef struct IVC1ENC_Obj {
    struct IVC1ENC_Fxns *fxns;
} IVC1ENC_Obj;

/**<

  @struct IVC1ENC_Params
  @brief This structure defines the Create time parameters for all
         VC1ENC objects

  @param  videnc2Params        must be followed for all video encoders.
                               Base class create params
  @param  rateControlParams    Controls all Rate Control related parameters
  @param  interCodingParams    Controls all Inter coding related parameters
  @param  intraCodingParams    Controls all Intra coding related parameters
  @param  sliceCodingParams    Controls all Slice coding related parameters
  @param  loopFilterParams     Controls the in-loop filtering process

  @param  interlaceCodingType  Controls the type of interlaced coding, refer
                               IVC1ENC_InterlaceCodingType for more details
  @param  bottomFieldIntra     This field is valid only for interlaced sequences
                 0 = Bottom field of the first I frame in the GOP encoded as
                     P field.
          non-zero = Bottom field of the first I frame in the GOP encoded as I
                     field.

  @param  gopStructure         Defines the gop structure type:
                               uniform/non-uniform. For more information refer
                               IVC1ENC_GOPStructure
  @param  streamFormat         Controls the stream format to enable/disable
                               headers.
  @param  pConstantMemory      This pointer points to the the memory area where
                               constants are located. If this is set to NULL
                               then encoder assumes that all constants are
                               pointed by symbol VC1ENC_TI_ConstData

  @param  maxIntraFrameInterval
                               This parameter contains the maximum Intra Frame
                               interval. It is used to reduce the memory
                               requirement of refernce Buffers. Because for all
                               I frame/field configuration the reference frame
                               buffers are not required
 @param  reservedParams        Some part is kept reserved to add parameters
                               later without changing the foot print of
                               interface object memory
*/

typedef struct IVC1ENC_Params {
    IVIDENC2_Params           videnc2Params;
    IVC1ENC_RateControlParams rateControlParams;
    IVC1ENC_InterCodingParams interCodingParams;
    IVC1ENC_IntraCodingParams intraCodingParams;
    IVC1ENC_SliceCodingParams sliceCodingParams;
    IVC1ENC_LoopFilterParams  loopFilterParams;

    XDAS_Int8  interlaceCodingType;
    XDAS_Int8  bottomFieldIntra;
    XDAS_Int8  gopStructure;
    XDAS_Int8  streamFormat;
    XDAS_Int32 pConstantMemory;
    XDAS_Int32 maxIntraFrameInterval;
    XDAS_Int32 reservedParams[4];

} IVC1ENC_Params;


/**<

  @struct IVC1ENC_Status
  @brief  This structure informs back the status of VC1 encoder and tells the
          value of each control parameter

  @param  videnc2Status        : must be followed for all video encoders.
                               : Base class status
  @param  rateControlParams    : Controls all Rate Control related parameters
  @param  interCodingParams    : Controls all Inter coding related parameters
  @param  intraCodingParams    : Controls all Intra coding related parameters
  @param  sliceCodingParams    : Controls all Slice coding related parameters
  @param  loopFilterParams     : Controls the in-loop filtering process

  @param  interlaceCodingType  : Controls the type of interlaced coding
  @param  bottomFieldIntra     : This field valid only for interlaced sequences
                                 0 = Bottom field of the first I frame in the
                                 GOP encoded as P field.
                                 non-zero = Bottom field of the first I frame in
                                 the GOP encoded as I field.
  @param  gopStructure         : Defines the gop structure type: Open/Close.
  @param  maxIntraFrameInterval: This parameter contains the maximum Intra Frame
                                 interval. It is used to reduce the memory
                                 requirement of refernce Buffers. Because for
                                 all I frame/field configuration the reference
                                 frame buffers are not required
  @param  searchCenter         : seacrh Center for motion estimation
  @param  reservedStatus       : Some part is kept reserved to add parameters
                                 later without changing the foot print of
                                 interface memory
*/

typedef struct IVC1ENC_Status {
    IVIDENC2_Status           videnc2Status;
    IVC1ENC_RateControlParams rateControlParams;
    IVC1ENC_InterCodingParams interCodingParams;
    IVC1ENC_IntraCodingParams intraCodingParams;
    IVC1ENC_SliceCodingParams sliceCodingParams;
    IVC1ENC_LoopFilterParams  loopFilterParams;

    XDAS_Int8  interlaceCodingType;
    XDAS_Int8  bottomFieldIntra;
    XDAS_Int8  gopStructure;
    XDAS_Int32 maxIntraFrameInterval;
    XDM_Point  searchCenter;
    XDAS_Int32 reservedStatus[4];

} IVC1ENC_Status;

/**<

  @struct IVC1ENC_DynamicParams
  @brief  This structure defines the run time parameters for all VC1ENC objects

  @param  videnc2DynamicParams must be followed for all video encoders
  @param  rateControlParams    Controls all Rate Control related parameters.
                               only few are supported to be changed as
                               part @ Control call. Refer
                               IVC1ENC_RateControlParams to find out
  @param  interCodingParams    Controls all inter MB coding related parameters.
                               only few are supported to be changed as
                               part @ Control call. Refer interCodingParams to
                               find out
  @param  sliceCodingParams    Controls all Slice coding related parameters.
                               only few are supported to be changed as
                               part @ Control call.
                               Refer sliceCodingParams to find out
  @param  searchCenter         seacrh Center for motion estimation.
                               XDM_Point.x == 0xFFFF means ignore searchCenter
  @param  reservedDynParams    Some part is kept reserved to add parameters
                               later without changing the foot print of
                               interface memory
*/

typedef struct IVC1ENC_DynamicParams {

    IVIDENC2_DynamicParams    videnc2DynamicParams;
    IVC1ENC_RateControlParams rateControlParams;
    IVC1ENC_InterCodingParams interCodingParams;
    IVC1ENC_SliceCodingParams sliceCodingParams;
    XDM_Point                 searchCenter;
    XDAS_Int32                reservedDynParams[4];

} IVC1ENC_DynamicParams;

/**<

  @struct IVC1ENC_InArgs
  @brief  This structure defines the input argument being passed to
          VC1 encoder

  @param videnc2InArgs : It is instance of base class. It contains all
                          the necessary  info required run time parameters for
                          all VC1ENC objects

*/
typedef struct IVC1ENC_InArgs {

    IVIDENC2_InArgs videnc2InArgs;

} IVC1ENC_InArgs;

/**<

  @struct IVC1ENC_OutArgs
  @brief  This structure defines the outpur argument being generated from VC1
          encoder

  @param videnc2OutArgs : It is instance of base class. It contains all
                           the necessary info encoder should produce
  @param  vbvBufferLevel: This variable tells the buffer level at the end of
                          every picture from decoder perspective.the value
                          populated in this variable is latest for every
                          process call
*/
typedef struct IVC1ENC_OutArgs {

    IVIDENC2_OutArgs videnc2OutArgs;
    XDAS_Int32       vbvBufferLevel;
} IVC1ENC_OutArgs;

/**<

  IVC1ENC_Cmd:This structure defines of the control commands for VC1ENC objects
*/

typedef IVIDENC2_Cmd IVC1ENC_Cmd;

/**<

  @struct IVC1ENC_Fxns
  @brief  This structure defines of the operations on VC1ENC objects

  @param IVIDENC2_Fxns : It is instance of base class. It contains all
                          function table

*/
typedef struct IVC1ENC_Fxns {
    IVIDENC2_Fxns ividenc;

} IVC1ENC_Fxns;


/**<Default parameter values for VC1ENC instance objects */
extern const IVC1ENC_Params    VC1ENC_TI_PARAMS;


#ifdef __cplusplus
extern "C" {
#endif

#endif  /*_IVC1ENC_H_*/

/* ========================================================================== */
/*  End of file:   ivc1enc.h                                                  */
/* -------------------------------------------------------------------------- */
/*            Copyright (c) 2010 Texas Instruments, Incorporated.             */
/*                           All Rights Reserved.                             */
/* ========================================================================== */

