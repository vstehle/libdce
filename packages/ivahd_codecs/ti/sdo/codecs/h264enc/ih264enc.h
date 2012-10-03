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

#ifndef _IH264ENC_H_
#define _IH264ENC_H_

#include <ti/xdais/dm/ividenc2.h>

#define IH264ENC_MAX_NUM_SLICE_START_OFFSET  (3)

typedef enum
{
  IH264_RATECONTROLPARAMS_DEFAULT = 0,
      /**< Default Rate Control params */
  IH264_RATECONTROLPARAMS_USERDEFINED = 1,
      /**< User defined Rate Control params */
  IH264_RATECONTROLPARAMS_EXISTING = 2,
      /**< Keep the Rate Control params as existing. This is
      * useful because during control call if user don't want
      * to chnage the Rate Control Params
      */
  IH264_RATECONTROLPARAMS_MAX

} IH264ENC_RateControlParamsPreset;

typedef enum
{
  IH264_RATECONTROL_PRC = 0,
  /**< Perceptual Rate Control, controls the QP @ MB level */
  IH264_RATECONTROL_PRC_LOW_DELAY = 1,
  /** Low Delay Rate Control */
  IH264_RATECONTROL_DEFAULT = IH264_RATECONTROL_PRC
  /** Default rcAlgo is PRC  */
} IH264ENC_RateControlAlgo;

typedef enum
{ IH264_SLICEMODE_NONE = 0,
  IH264_SLICEMODE_DEFAULT = IH264_SLICEMODE_NONE,
  /**< Default slice coding mode is MB based */
  IH264_SLICEMODE_MBUNIT = 1,
  /**< Slices are controlled based upon number of Macroblocks */
  IH264_SLICEMODE_BYTES = 2,
  /**< Slices are controlled based upon number of bytes */
  IH264_SLICEMODE_OFFSET = 3,
  /**< Slices are controlled based upon user defined offset in
   * unit of Row */
  IH264_SLICEMODE_MAX
} IH264ENC_SliceMode;

typedef struct {
    IVIDENC2_Params videnc2Params;
    XDAS_Int8 reserved[144];
    XDAS_Int8 entropyCodingMode;
    XDAS_Int8 reserved0[3];
    XDAS_Int32 IDRFrameInterval;
    XDAS_Int8 reserved1[4];
    XDAS_Int32  maxIntraFrameInterval;
    XDAS_Int8 reserved2[16];
    XDAS_Int8 numTemporalLayer;
    XDAS_Int8 reserved3[13];
} IH264ENC_Params;

typedef enum {
  IH264_ENTROPYCODING_CAVLC = 0,
  /**< CAVLC coding type */
  IH264_ENTROPYCODING_DEFAULT = IH264_ENTROPYCODING_CAVLC,
  /**< Default is CAVLC coding type */
  IH264_ENTROPYCODING_CABAC = 1,
  /**< CABAC coding type */
  IH264_ENTROPYCODING_MAX
} IH264ENC_EntropyCodingMode;

typedef struct IH264ENC_RateControlParams {
  XDAS_Int8 rateControlParamsPreset;
  XDAS_Int8 scalingMatrixPreset;
  XDAS_Int8 rcAlgo;
  XDAS_Int8 qpI;
  XDAS_Int8 qpMaxI;
  XDAS_Int8 qpMinI;
  XDAS_Int8 qpP;
  XDAS_Int8 qpMaxP;
  XDAS_Int8 qpMinP;
  XDAS_Int8 qpOffsetB;
  XDAS_Int8 qpMaxB;
  XDAS_Int8 qpMinB;
  XDAS_Int8 allowFrameSkip;
  XDAS_Int8 removeExpensiveCoeff;
  XDAS_Int8 chromaQPIndexOffset;
  XDAS_Int8 IPQualityFactor;
  XDAS_Int32 initialBufferLevel;
  XDAS_Int32 HRDBufferSize;
  XDAS_Int16 minPicSizeRatio;
  XDAS_Int16 maxPicSizeRatio;
  XDAS_Int8 enablePRC;
  XDAS_Int8 enablePartialFrameSkip;
  XDAS_Int8 discardSavedBits;
  XDAS_Int8 reserved;
  XDAS_Int32 reservedRC[3];
} IH264ENC_RateControlParams ;

typedef struct IH264ENC_InterCodingParams {
  XDAS_Int8 interCodingPreset;
  XDAS_Int16 searchRangeHorP;
  XDAS_Int16 searchRangeVerP;
  XDAS_Int16 searchRangeHorB;
  XDAS_Int16 searchRangeVerB;
  XDAS_Int8 interCodingBias;
  XDAS_Int8 skipMVCodingBias;
  XDAS_Int8 minBlockSizeP;
  XDAS_Int8 minBlockSizeB;

} IH264ENC_InterCodingParams;

typedef struct IH264ENC_SliceCodingParams {
  XDAS_Int8 sliceCodingPreset;
  XDAS_Int16 sliceMode;
  XDAS_Int16 sliceUnitSize;
  XDAS_Int8 sliceStartOffset[IH264ENC_MAX_NUM_SLICE_START_OFFSET];
  XDAS_Int8 streamFormat;

} IH264ENC_SliceCodingParams ;

typedef struct IH264ENC_DynamicParams {
  IVIDENC2_DynamicParams videnc2DynamicParams;
  IH264ENC_RateControlParams rateControlParams;
  IH264ENC_InterCodingParams interCodingParams;
  IH264ENC_SliceCodingParams sliceCodingParams;
  XDAS_Int32 sliceGroupChangeCycle;
  XDM_Point searchCenter;
  XDAS_Int8 enableStaticMBCount;
  XDAS_Int32 reservedDynParams[4];
} IH264ENC_DynamicParams;

#endif
