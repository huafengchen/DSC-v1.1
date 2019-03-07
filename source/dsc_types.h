/***************************************************************************
*    Copyright (c) 2013, Broadcom Corporation
*    All rights reserved.
*
*  Statement regarding contribution of copyrighted materials to VESA:
*
*  This code is owned by Broadcom Corporation and is contributed to VESA
*  for inclusion and use in its VESA Display Stream Compression specification.
*  Accordingly, VESA is hereby granted a worldwide, perpetual, non-exclusive
*  license to revise, modify and create derivative works to this code and
*  VESA shall own all right, title and interest in and to any derivative 
*  works authored by VESA.
*
*  Terms and Conditions
*
*  Without limiting the foregoing, you agree that your use
*  of this software program does not convey any rights to you in any of
*  Broadcom’s patent and other intellectual property, and you
*  acknowledge that your use of this software may require that
*  you separately obtain patent or other intellectual property
*  rights from Broadcom or third parties.
*
*  Except as expressly set forth in a separate written license agreement
*  between you and Broadcom, if applicable:
*
*  1. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED
*  "AS IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES,
*  REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR
*  OTHERWISE, WITH RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY
*  DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY,
*  NONINFRINGEMENT, FITNESS FOR A PARTICULAR PURPOSE, LACK OF VIRUSES,
*  ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR
*  CORRESPONDENCE TO DESCRIPTION. YOU ASSUME THE ENTIRE RISK ARISING
*  OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
* 
*  2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL
*  BROADCOM OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL,
*  SPECIAL, INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR
*  IN ANY WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN
*  IF BROADCOM HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii)
*  ANY AMOUNT IN EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF
*  OR U.S. $1, WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY
*  NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
***************************************************************************/

#ifndef _DSC_TYPES_H_
#define _DSC_TYPES_H_

#include "fifo.h"

#define NUM_BUF_RANGES        15
#define NUM_COMPONENTS        3
#define MAX_UNITS_PER_GROUP   3
#define SAMPLES_PER_UNIT      3
#define PIXELS_PER_GROUP	  3
#define GROUPS_PER_SUPERGROUP 4
#define BP_RANGE              10
#define BP_SIZE				  3
#define PRED_BLK_SIZE		  3
#define ICH_BITS			  5
#define ICH_SIZE			  (1<<ICH_BITS)
#define ICH_PIXELS_ABOVE      7
#define OFFSET_FRACTIONAL_BITS  11
#define MAX_SE_SIZE           (4*dsc_cfg->bits_per_component+4)
#define PPS_SIZE			  128
#define BP_EDGE_COUNT		  3
#define BP_EDGE_STRENGTH      32
#define PADDING_LEFT          5  // Pixels to pad line arrays to the left
#define PADDING_RIGHT         5  // Pixels to pad line arrays to the right
#define RC_SCALE_BINARY_POINT   3
#define SOMEWHAT_FLAT_QP_THRESH (7+(2*(dsc_cfg->bits_per_component-8)))
#define OVERFLOW_AVOID_THRESHOLD  (-172)

typedef enum { PT_MAP=0, PT_LEFT, PT_BLOCK } PRED_TYPE;

#define NUM_PRED_TYPES        (PT_BLOCK+BP_RANGE)

/// Configuration for a single RC model range
typedef struct dsc_range_cfg_s {
	int  range_min_qp;			///< Min QP allowed for this range
	int  range_max_qp;			///< Max QP allowed for this range
	int  range_bpg_offset;		///< Bits/group offset to apply to target for this group
} dsc_range_cfg_t;

/// Codec configuration
typedef struct dsc_cfg_s {
	int  linebuf_depth;		///< Bits / component for previous reconstructed line buffer
	int  rcb_bits;			///< Rate control buffer size (in bits); not in PPS, used only in C model for checking overflow
	int  bits_per_component; ///< Bits / component to code (must be 8, 10, or 12)
	int  convert_rgb;		///< Flag indicating to do RGB - YCoCg conversion and back (should be 1 for RGB input)
	int  slice_width;		///< Slice width
	int  slice_height;		///< Slice height
	int  enable_422;        ///< 4:2:2 enable mode (from PPS, 4:2:2 conversion happens outside of DSC encode/decode algorithm)
	int  pic_width;			///< Picture width
	int  pic_height;		///< Picture height
	int  rc_tgt_offset_hi;		///< Offset to bits/group used by RC to determine QP adjustment
	int  rc_tgt_offset_lo;		///< Offset to bits/group used by RC to determine QP adjustment
	int  bits_per_pixel;	///< Bits/pixel target << 4 (ie., 4 fractional bits)
	int  rc_edge_factor;	///< Factor to determine if an edge is present based on the bits produced
	int  rc_quant_incr_limit1; ///< Slow down incrementing once the range reaches this value
	int  rc_quant_incr_limit0; ///< Slow down incrementing once the range reaches this value
	int  initial_xmit_delay;	///< Number of pixels to delay the initial transmission
	int  initial_dec_delay;		///< Number of pixels to delay the VLD on the decoder, not including SSM
	int  block_pred_enable;	///< Block prediction range (in pixels)
	int  first_line_bpg_ofs; ///< Bits/group offset to use for first line of the slice
	int  initial_offset;    ///< Value to use for RC model offset at slice start
	int  xstart;			///< X position in the picture of top-left corner of slice
	int  ystart;			///< Y position in the picture of top-left corner of slice
	int  rc_buf_thresh[NUM_BUF_RANGES-1];   ///< Thresholds defining each of the buffer ranges
	dsc_range_cfg_t rc_range_parameters[NUM_BUF_RANGES];  ///< Parameters for each of the RC ranges
	int  rc_model_size;     ///< Total size of RC model
	int  flatness_min_qp;	///< Minimum QP where flatness information is sent
	int  flatness_max_qp;   ///< Maximum QP where flatness information is sent
	int  flatness_det_thresh;  ///< MAX-MIN for all components is required to be <= this value for flatness to be used
	int  initial_scale_value;  ///< Initial value for scale factor
	int  scale_decrement_interval;   ///< Decrement scale factor every scale_decrement_interval groups
	int  scale_increment_interval;   ///< Increment scale factor every scale_increment_interval groups
	int  nfl_bpg_offset;		///< Non-first line BPG offset to use
	int  slice_bpg_offset;		///< BPG offset used to enforce slice bit constraint
	int  final_offset;          ///< Final RC linear transformation offset value
	int  vbr_enable;			///< Enable on-off VBR (ie., disable stuffing bits)
	int  muxing_mode;           ///< 0 = standard DSU, 1 = substream muxing, 2 = concatenated (not yet supported)
	int  mux_word_size;         ///< Mux word size (in bits) for SSM mode
	int  chunk_size;            ///< The (max) size in bytes of the "chunks" that are used in slice multiplexing
	int  pps_identifier;		///< Placeholder for PPS identifier
} dsc_cfg_t;

/// The ICH state
typedef struct dsc_history_s {
	unsigned int *pixels[NUM_COMPONENTS];  ///< Array of pixels that corresponds to the shift register (position 0 is MRU)
	int *valid;				  ///< Valid bits for each pixel in the history (position 0 is MRU)
} dsc_history_t;

/// The DSC state
typedef struct dsc_state_s {
	int isEncoder;			///< 1=encode, 0=decode
	int numBits;			///< Number of bits read/written
	int prevNumBits;		///< Number of bits at end of previous group
	int postMuxNumBits;     ///< Number of bits read/written post-mux
	int bufferSize;			///< Buffer size in bits
	int bufferFullness;		///< Current buffer fullness
	int bpgFracAccum;		///< Accumulator for fractional bits/group
	int rcXformOffset;		///< Offset used for RC model (to get rcXformOffset from spec, subtract rc_model_size from this value)
	int throttleInt;		///< Integer portion of RC offset
	int throttleFrac;		///< Fractional portion of RC offset
	int nonFirstLineBpgTarget;  ///< Bits/group target for non-first-lines
	int currentScale;		///< Current scale factor used for RC model
	int scaleAdjustCounter;  ///< Counter used to compute when to adjust scale factor
	int predErr[NUM_COMPONENTS][BP_RANGE];   ///< Errors for all pred types for each component
	int stQp;				///< QP from RC
	int prevQp;		  		///< QP for previous group from RC
	int quantizedResidual[MAX_UNITS_PER_GROUP][SAMPLES_PER_UNIT];  ///< Quantized residuals for current group
	int quantizedResidualMid[MAX_UNITS_PER_GROUP][SAMPLES_PER_UNIT];  ///< Quantized residuals assuming midpoint prediction for current group
	int rcSizeUnit[MAX_UNITS_PER_GROUP];  ///< Size for each unit assuming size was perfectly predicted
	int rcSizeGroup;		///< Sum of RcSizeUnits for previous group
	int codedGroupSize;		///< Size of previous group in bits
	int predictedSize[NUM_COMPONENTS];  ///< Predicted sizes for next DSU code 
	PRED_TYPE *prevLinePred;  ///< BP selection decsion buffer (since model calculates BP offset one line ahead of time)
	int bpCount;			///< Number of times in a row block prediction had the lowest cost
	int lastEdgeCount;		///< How long ago we saw the last edge (for BP)
	int edgeDetected;       ///< Was an edge detected for BP
	int lastErr[NUM_COMPONENTS][BP_SIZE][BP_RANGE];  ///< 3-pixel SAD's for each of the past 3 3-pixel-wide prediction blocks for each BP offset
	dsc_history_t history;	///< The ICH
	int hPos;				///< Current horizontal position within the slice
	int vPos;				///< Current vertical position within the slice
	int masterQp;			///< QP used for the current group
	int prevMasterQp;			///< QP used for the previous group
	int midpointRecon[MAX_UNITS_PER_GROUP][SAMPLES_PER_UNIT];  ///< Reconstructed samples assuming midpoint prediction is selected
	int midpointSelected[MAX_UNITS_PER_GROUP];  ///< Encoder flags indicating for each unit whether midpoint prediction is selected
	int useMidpoint[MAX_UNITS_PER_GROUP];  ///< Decoder flags indicating for each unit whether midpoint prediction should be used 
	int ichSelected;		///< Flag indicating ICH is used for current group
	int prevIchSelected;	///< Flag indicating ICH is used for previous group
	int ichLookup[PIXELS_PER_GROUP];		///< ICH indices for current group
	int *prevLine[NUM_COMPONENTS];  ///< Previous line reconstructed samples 
	int *currLine[NUM_COMPONENTS];  ///< Current line reconstructed samples
	int *origLine[NUM_COMPONENTS];  ///< Current line original samples (for encoder)
	int origWithinQerr[PIXELS_PER_GROUP];   ///< Encoder flags indicating that original pixels are within the quantization error
	unsigned int ichPixels[PIXELS_PER_GROUP][NUM_COMPONENTS];  ///< ICH pixel samples selected for current group (for encoder)
	int leftRecon[NUM_COMPONENTS];		///< Previous group's rightmost reconstructed samples (used for midpoint prediction)
	int cpntBitDepth[NUM_COMPONENTS];   ///< Bit depth for each component
	int origIsFlat;			///< Flag indicating that original pixels are flat for this group
	int groupCount;			///< Current group count
	int firstFlat;			///< If -1, none of the 4 group set are flat, otherwise indicates which group is the one where flatness starts
	int prevFirstFlat;			///< If -1, none of the 4 group set are flat, otherwise indicates which group is the one where flatness starts
	int prevIsFlat;			///< Flag indicating the previous group is flat
	int flatnessType;		///< 0 = somewhat flat; 1 = very flat
	int prevFlatnessType;   ///< The flatnessType coded with the previous group
	int maxError[NUM_COMPONENTS];  ///< Max error for each component using DPCM
	int maxMidError[NUM_COMPONENTS];  ///< Max error for each component using MPP
	int maxIchError[NUM_COMPONENTS];  ///< Max error for each component using ICH
	int groupCountLine;    ///< Group count for current line
	int prevRange;         ///< RC range for previous group
	fifo_t shifter[NUM_COMPONENTS];		///< Decoder funnel shifter
	fifo_t encBalanceFifo[NUM_COMPONENTS];	///< Encoder balance FIFO's
	fifo_t seSizeFifo[NUM_COMPONENTS];	///< Syntax element sizes
	int forceMpp;			///< Flag to force MPP mode to prevent underflow
	int *quantTableLuma;	///< Quantization table for luma
	int *quantTableChroma;	///< Quantization table for chroma
	int numBitsChunk;		///< Number of bits output for the current chunk
	int chunkCount;         ///< Chunk number currently being coded
	int bitsClamped;        ///< Number of bits clamped by buffer level tracker in VBR mode
	int *chunkSizes;        ///< For encoders, stores the chunk sizes for each chunk in slice multiplexing
	int pixelCount;         ///< Number of pixels that have been processed
	int prevPixelCount;		///< Number of pixels that have been processed as of the previous group
	int chunkPixelTimes;    ///< Number of pixels that have been generated for the current slice line
	int rcOffsetClampEnable; ///< Set to true after rcXformOffset falls below final_offset - rc_model_size
	int scaleIncrementStart; ///< Flag indicating that the scale increment has started
} dsc_state_t;

#endif // __DSC_TYPES_H_
