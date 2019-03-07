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

/*! \file dsc_codec.c
 *    DSC codec
 *  \author Frederick Walls (fwalls@broadcom.com)
 *  \author Sandy MacInnis (macinnis@broadcom.com)
 *  \author and others at Broadcom
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <assert.h>
#include "dsc_utils.h"
#include "dsc_codec.h"
#include "dsc_types.h"
#include "multiplex.h"

//#define PRINTDEBUG
#define PRINT_DEBUG_VLC   0
#define PRINT_DEBUG_RC    0
#define PRINT_DEBUG_RECON 0

// Prototypes
int MapQpToQlevel(dsc_state_t *dsc_state, int qp, int CType);
int FindResidualSize(int eq);
int SampToLineBuf( dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int x, int CType);

//-----------------------------------------------------------------------------
// debug dumping

FILE *g_fp_dbg = 0;

//-----------------------------------------------------------------------------
// The following are constants that are used in the code:
const int QuantDivisor[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096 };
const int QuantOffset[] = {0, 0, 1, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047 };
int qlevel_luma_8bpc[] = {0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 7 };
int qlevel_chroma_8bpc[] = {0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 8, 8, 8 };
int qlevel_luma_10bpc[] = {0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 9 };
int qlevel_chroma_10bpc[] = {0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 10, 10 };
int qlevel_luma_12bpc[] = {0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 11 };
int qlevel_chroma_12bpc[] = {0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 12, 12 };


//! Map QP to quantization level
/*! \param dsc_state DSC state structure
    \param qp      QP to map
    \param cpnt    Component to map
    \return        Corresponding qlevel */
int MapQpToQlevel(dsc_state_t *dsc_state, int qp, int cpnt)
{
	int qlevel;

	// *MODEL NOTE* MN_MAP_QP_TO_QLEVEL
	if (cpnt == 0)
		qlevel = dsc_state->quantTableLuma[qp];
	else
		qlevel = dsc_state->quantTableChroma[qp];

	return (qlevel);
}


//! Quantize a residual
/*! \param e       Raw residual
    \param qlevel  Quantization level
    \return        Quantized residual */
int QuantizeResidual(int e, int qlevel)
{
	int eq;
	
	// *MODEL NOTE* MN_ENC_QUANTIZATION
	if (e>0)
		eq = (e + QuantOffset[qlevel]) >> qlevel;
	else
		eq = -((QuantOffset[qlevel]-e) >> qlevel);

	return eq;
}


//! Get maximum residual size for a given component & quantization level
/*! \param dsc_state DSC state structure
	\param cpnt      Which component
	\param qp        Quantization parameter 
	\return          Max residual size in bits */
int MaxResidualSize(dsc_state_t *dsc_state, int cpnt, int qp)
{
	int qlevel;

	qlevel = MapQpToQlevel(dsc_state, qp, cpnt);

	return (dsc_state->cpntBitDepth[cpnt] - qlevel);
}


//! Get predicted size for unit (adjusted for QP changes)
/*! \param dsc_state DSC state structure
	\param cpnt      Which component
    \return          Predicted size for unit */
int GetQpAdjPredSize(dsc_state_t *dsc_state, int cpnt)
{
	int pred_size, max_size;
	int qlevel_old, qlevel_new;

	// *MODEL NOTE* MN_DSU_SIZE_PREDICTION
	pred_size = dsc_state->predictedSize[cpnt];

	qlevel_old = MapQpToQlevel(dsc_state, dsc_state->prevMasterQp, cpnt);
	qlevel_new = MapQpToQlevel(dsc_state, dsc_state->masterQp, cpnt);

	pred_size += qlevel_old - qlevel_new;
	max_size = MaxResidualSize(dsc_state, cpnt, dsc_state->masterQp);
	pred_size = CLAMP(pred_size, 0, max_size-1);

	return (pred_size);
}


//! Get the predicted sample value
/*! \param prevLine  Array of samples from previous (reconstructed) line
	\param currLine  Array of samples from current (reconstructed) line
	\param hPos      Horizontal position within slice of sample to predict
	\param predType  Prediction mode to use (PT_MAP or one of PT_BLOCK)
	\param qLevel    Quantization level for current component
	\param cpnt      Which component
	\return          Predicted sample value */
int SamplePredict(
	dsc_state_t *dsc_state,
	int* prevLine,          // reconstructed samples for previous line
	int* currLine,          // reconstructed samples for current line
	int hPos,               // horizontal position for sample to predict
	PRED_TYPE predType,     // predictor to use
	int qLevel,
	int cpnt)
{
	int a, b, c, d, e;
	int filt_b, filt_c, filt_d, filt_e;
	int blend_b, blend_c, blend_d, blend_e;
	int p;
	int bp_offset;
	int diff = 0;
	int h_offset_array_idx;

	h_offset_array_idx = (hPos / 3) * 3 + PADDING_LEFT; 

	// organize samples into variable array defined in dsc spec
	c = prevLine[h_offset_array_idx-1];
	b = prevLine[h_offset_array_idx];
	d = prevLine[h_offset_array_idx+1];
	e = prevLine[h_offset_array_idx+2];
	a = currLine[h_offset_array_idx-1];

#define FILT3(a,b,c) (((a)+2*(b)+(c)+2)>>2)
	filt_c = FILT3(prevLine[h_offset_array_idx-2], prevLine[h_offset_array_idx-1], prevLine[h_offset_array_idx]);
	filt_b = FILT3(prevLine[h_offset_array_idx-1], prevLine[h_offset_array_idx], prevLine[h_offset_array_idx+1]);
	filt_d = FILT3(prevLine[h_offset_array_idx], prevLine[h_offset_array_idx+1], prevLine[h_offset_array_idx+2]);
	filt_e = FILT3(prevLine[h_offset_array_idx+1], prevLine[h_offset_array_idx+2], prevLine[h_offset_array_idx+3]);

	switch (predType) {
	case PT_MAP:	// MAP prediction
		// *MODEL NOTE* MN_MMAP
		diff = CLAMP(filt_c - c, -(QuantDivisor[qLevel]/2), QuantDivisor[qLevel]/2);
		blend_c = c + diff;
		diff = CLAMP(filt_b - b, -(QuantDivisor[qLevel]/2), QuantDivisor[qLevel]/2);
		blend_b = b + diff;
		diff = CLAMP(filt_d - d, -(QuantDivisor[qLevel]/2), QuantDivisor[qLevel]/2);
		blend_d = d + diff;
		diff = CLAMP(filt_e - e, -(QuantDivisor[qLevel]/2), QuantDivisor[qLevel]/2);
		blend_e = e + diff;
		
		// Pixel on line above off the raster to the left gets same value as pixel below (ie., midpoint)
		if (hPos/SAMPLES_PER_UNIT == 0)
			blend_c = a;
		
		if ((hPos % SAMPLES_PER_UNIT)==0)  // First pixel of group
			p = CLAMP(a + blend_b - blend_c, MIN(a, blend_b), MAX(a, blend_b));
		else if ((hPos % SAMPLES_PER_UNIT)==1)   // Second pixel of group
			p = CLAMP(a + blend_d - blend_c + (dsc_state->quantizedResidual[cpnt][0] * QuantDivisor[qLevel]),
				        MIN(MIN(a, blend_b), blend_d), MAX(MAX(a, blend_b), blend_d));
		else    // Third pixel of group
			p = CLAMP(a + blend_e - blend_c + (dsc_state->quantizedResidual[cpnt][0] + dsc_state->quantizedResidual[cpnt][1])*QuantDivisor[qLevel],
						MIN(MIN(a,blend_b), MIN(blend_d, blend_e)), MAX(MAX(a,blend_b), MAX(blend_d, blend_e)));
		break;
	case PT_LEFT:
		p = a;    // First pixel of group
		if ((hPos % SAMPLES_PER_UNIT)==1)   // Second pixel of group
			p = CLAMP(a + (dsc_state->quantizedResidual[cpnt][0] * QuantDivisor[qLevel]), 0, (1<<dsc_state->cpntBitDepth[cpnt])-1);
		else if((hPos % SAMPLES_PER_UNIT)==2)  // Third pixel of group
			p = CLAMP(a + (dsc_state->quantizedResidual[cpnt][0] + dsc_state->quantizedResidual[cpnt][1])*QuantDivisor[qLevel],
						0, (1<<dsc_state->cpntBitDepth[cpnt])-1);
		break;
	default:  // PT_BLOCK+ofs = BLOCK predictor, starts at -1
		// *MODEL NOTE* MN_BLOCK_PRED
		bp_offset = (int)predType - (int)PT_BLOCK;
		p = currLine[MAX(hPos + PADDING_LEFT - 1 - bp_offset,0)];
		break;
	}

	return p;
}


//! Encoder function to estimate bits required to code original pixels
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure
	\return          Bits estimate to code original pixels */
int EstimateBitsForGroup(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state)
{
	int i, cpnt;
	int max_size[NUM_COMPONENTS];
	int total_size;
	int qlevel[NUM_COMPONENTS], size, qerr;
	int qp;
	int max_residual_size;
	int pred_size;
	int hPos;

	max_size[0] = max_size[1] = max_size[2] = 0;
	qp = dsc_state->masterQp;

	qlevel[0] = MapQpToQlevel(dsc_state, qp, 0);
	qlevel[1] = qlevel[2] = MapQpToQlevel(dsc_state, qp, 1);

	for (cpnt = 0; cpnt < NUM_COMPONENTS; ++cpnt)
	{
		for (i=0; i<PIXELS_PER_GROUP; ++i)
		{
			hPos = dsc_state->hPos + i - (PIXELS_PER_GROUP-1);
			if (hPos >= dsc_cfg->slice_width)
				continue;
			qerr = dsc_state->quantizedResidual[cpnt][i];		// use real P-mode residual sizes
			size = FindResidualSize(qerr);
			if (size > max_size[cpnt])
				max_size[cpnt] = size;
		}
		max_residual_size = MaxResidualSize(dsc_state, cpnt, dsc_state->masterQp);
		if (max_size[cpnt] > max_residual_size)
			max_size[cpnt] = max_residual_size;
	}

	total_size = 0;
	for (cpnt = 0; cpnt < NUM_COMPONENTS; ++cpnt)
	{
		pred_size = GetQpAdjPredSize(dsc_state, cpnt);
        if (max_size[cpnt] < pred_size)
			total_size += 1 + SAMPLES_PER_UNIT * pred_size;
		else if((max_size[cpnt] == MaxResidualSize(dsc_state, cpnt, dsc_state->masterQp)) && (cpnt != 0)) //Only do for Chroma
			total_size += (max_size[cpnt] - pred_size) + SAMPLES_PER_UNIT * max_size[cpnt];
		else        
			total_size += 1 + (max_size[cpnt] - pred_size) + SAMPLES_PER_UNIT * max_size[cpnt];
                                                                
	}
	//If previous group was ICH and luma is not maxsize, add another bit
	if( (max_size[0] < MaxResidualSize(dsc_state, 0, dsc_state->masterQp)) && dsc_state->prevIchSelected)
		total_size += 1;

	return (total_size);
}


//! Look up the pixel values for a given ICH index
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure
	\param entry     History index (0-31)
	\param p         Returned pixel value
	\param hPos      Horizontal position in slice (to determine upper pixels, if applicable)
	\param first_line_flag Set to 1 for first line (ie., disable upper pixels) */
void HistoryLookup(dsc_cfg_t *dsc_cfg, dsc_state_t* dsc_state, int entry, unsigned int *p, int hPos, int first_line_flag)
{
	int reserved;

	reserved = ICH_SIZE-ICH_PIXELS_ABOVE;

	hPos = (hPos/PIXELS_PER_GROUP)*PIXELS_PER_GROUP + (PIXELS_PER_GROUP/2);  // Center pixel of group as reference
	hPos = CLAMP(hPos, ICH_PIXELS_ABOVE/2, dsc_cfg->slice_width-1-(ICH_PIXELS_ABOVE/2));  // Keeps upper line history entries unique at left & right edge
	if (!first_line_flag && (entry >= reserved))
	{
		p[0] = dsc_state->prevLine[0][hPos+(entry-reserved)+PADDING_LEFT - (ICH_PIXELS_ABOVE/2)];
		p[1] = dsc_state->prevLine[1][hPos+(entry-reserved)+PADDING_LEFT - (ICH_PIXELS_ABOVE/2)];
		p[2] = dsc_state->prevLine[2][hPos+(entry-reserved)+PADDING_LEFT - (ICH_PIXELS_ABOVE/2)];
	} else {
		p[0] = dsc_state->history.pixels[0][entry];
		p[1] = dsc_state->history.pixels[1][entry];
		p[2] = dsc_state->history.pixels[2][entry];
	}
}


//! Encoder function to determine whether or not the current sample is within the quantization error of any ICH entry 
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param hPos      Current horizontal position within slice
	\param vPos      Current vertical position within slice
	\param qp        Quantization parameter for current group
	\param sampModCnt Index of current pixel within group 
	\return          1 indicates that the sample is within the quantization error */
int IsOrigWithinQerr(dsc_cfg_t* dsc_cfg, dsc_state_t *dsc_state, int hPos, int vPos, int qp, int sampModCnt)
{
	int max_qerr[NUM_COMPONENTS];
	int hit;
	int diff, absdiff;
	unsigned int orig;
	int cpnt;
	unsigned int ich_pixel[NUM_COMPONENTS];
	int modified_qp;
	int i, j;

	// *MODEL NOTE* MN_ENC_ICH_PIXEL_CHECK_QERR
	dsc_state->origWithinQerr[sampModCnt] = 0;  // Assume for now that pixel is not within QErr

	if (ICH_BITS==0)		// ICH disabled
		return(0);
	if ((hPos == 0) && (vPos == 0))
		return(0);		// 1st group of slice can't use ICH since it's empty

	if(dsc_cfg->bits_per_component == 8)
		modified_qp = MIN(15, qp+2);
	else if(dsc_cfg->bits_per_component == 10)
		modified_qp = MIN(19, qp+2);
	else
		modified_qp = MIN(23, qp+2);

	max_qerr[0] = QuantDivisor[MapQpToQlevel(dsc_state, modified_qp, 0)]/2;
	max_qerr[1] = max_qerr[2] = QuantDivisor[MapQpToQlevel(dsc_state, modified_qp, 1)]/2;

	if (dsc_state->vPos>0)
	{
		// UL/U/UR always valid for non-first-lines
		for (i=ICH_SIZE-ICH_PIXELS_ABOVE; i<ICH_SIZE; ++i)
			dsc_state->history.valid[i] = 1;
	}

	hit = 0;
	for (j=0; j<ICH_SIZE; ++j)
	{
		if (!dsc_state->history.valid[j])
			continue;
		hit = 1;
		HistoryLookup(dsc_cfg, dsc_state, j, ich_pixel, hPos, (vPos==0));  // CLAMP is to minimize duplicates among UL/U/UR
		for (cpnt=0; cpnt<NUM_COMPONENTS; ++cpnt)
		{
			orig = dsc_state->origLine[cpnt][hPos+PADDING_LEFT];
			diff = (int)ich_pixel[cpnt] - (int)orig;
			absdiff = abs(diff);
			if (absdiff > max_qerr[cpnt])
				hit = 0;
		}
		if (hit)
			break;  // Found one
	}
	if (!hit)
		return(0);  // Can't use, one pixel was a total miss

	dsc_state->origWithinQerr[sampModCnt] = 1;
	return (1);
}


//! Update the ICH state based on a reconstructed pixel
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param recon     A 3-element array containing the components of the reconstructed pixel */
void UpdateHistoryElement(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, unsigned int *recon)
{
	int hit, j;
	int loc;
	int cpnt;
	unsigned int ich_pixel[NUM_COMPONENTS];
	int reserved;

	// *MODEL NOTE* MN_ICH_UPDATE
	reserved = (dsc_state->vPos==0) ? ICH_SIZE : (ICH_SIZE-ICH_PIXELS_ABOVE);  // space for UL, U, UR

	// Update the ICH with recon as the MRU
	hit = 0;
	loc = reserved-1;  // If no match, delete LRU
	for (j=0; j<reserved; ++j)
	{
		if (!dsc_state->history.valid[j])  // Can replace any empty entry
		{
			loc = j;
			break;
		}
		HistoryLookup(dsc_cfg, dsc_state, j, ich_pixel, dsc_state->hPos, (dsc_state->vPos==0));  // Specific hPos within group is not critical 
																					// since hits against UL/U/UR don't have specific detection
		hit = 1;
		for (cpnt=0; cpnt<NUM_COMPONENTS; ++cpnt)
		{
			if (ich_pixel[cpnt] != recon[cpnt])
				hit = 0;
		}
		if (hit && ((dsc_state->isEncoder && dsc_state->ichSelected) || (!dsc_state->isEncoder && dsc_state->prevIchSelected)))
		{
			loc = j;
			break;  // Found one
		}
	}

	for (cpnt=0; cpnt<NUM_COMPONENTS; ++cpnt)
	{
		// Delete from current position (or delete LRU)
		for (j=loc; j>0; --j)
		{
			dsc_state->history.pixels[cpnt][j] = dsc_state->history.pixels[cpnt][j-1];	
			dsc_state->history.valid[j] = dsc_state->history.valid[j-1];
		}

		// Insert as most recent
		dsc_state->history.pixels[cpnt][0] = recon[cpnt];
		dsc_state->history.valid[0] = 1;
	}
}


//! Encoder function to select the best history entry & return the corresponding index
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param hPos      Horizontal position within slice
	\param orig      3-element array containing the component samples for the pixel to be matched
	\return          The ICH index that the encoder selects */
int PickBestHistoryValue(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int hPos, unsigned int *orig)
{
	int j, best, lowest_sad;
	int diff0, diff1, diff2, weighted_sad;
	unsigned int ich_pixel[NUM_COMPONENTS];

	// Initialize to large, illegal values
	lowest_sad = 9999;
	best = 99;

	// *MODEL NOTE* MN_ENC_ICH_IDX_SELECT
	for (j=0; j<ICH_SIZE; ++j)
	{
		if (!dsc_state->history.valid[j])
			continue;
		weighted_sad = 0;
		HistoryLookup(dsc_cfg, dsc_state, j, ich_pixel, hPos, (dsc_state->vPos==0));  // -2 to preserve U/UL/UR at right edge
		diff0 = (int)ich_pixel[0] - (int)orig[0];
		diff1 = (int)ich_pixel[1] - (int)orig[1];
		diff2 = (int)ich_pixel[2] - (int)orig[2];
		weighted_sad += 2*abs(diff0) + abs(diff1) + abs(diff2);
		if (lowest_sad > weighted_sad)  // Find lowest SAD
		{
			lowest_sad = weighted_sad;
			best = j;
		}
	}

	return (best);
}


//! Updates the IHC state using final reconstructed value
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param currLine  Reconstructed samples for current line
	\param sampModCnt Index of which pixel within group
	\param hPos      Horizontal position within slice (note that update uses pixels from one group prior)
	\param vPos      Vertical position within slice */
void UpdateICHistory(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int **currLine, int sampModCnt, int hPos, int vPos)
{
	int i;
	int cpnt;
	unsigned int p[NUM_COMPONENTS];
	int hPos_prev_group;

	if (ICH_BITS==0)
		return;			// Nothing to do if ICH disabled

	// Reset ICH at beginning of each line if multiple slices per line
	if (hPos==0)
	{
		if (vPos == 0)            // Beginning of slice
		{
			for (i=0; i<ICH_SIZE; ++i)
				dsc_state->history.valid[i] = 0;
		}
		else if (dsc_cfg->slice_width != dsc_cfg->pic_width)   // Multiple slices per line
		{
			for (i=0; i<(ICH_SIZE - ICH_PIXELS_ABOVE); ++i)
				dsc_state->history.valid[i] = 0;
		}
	}

	hPos_prev_group = hPos - PIXELS_PER_GROUP;

	if (hPos_prev_group < 0)
		return;			// Current code doesn't update ICH for last group of each line -- probably not much correlation anyway

	// Get final reconstructed pixel value
	for (cpnt=0; cpnt<NUM_COMPONENTS; ++cpnt)
		p[cpnt] = currLine[cpnt][hPos_prev_group + PADDING_LEFT];

	// Update ICH accordingly
	UpdateHistoryElement(dsc_cfg, dsc_state, p);
}


//! Encoder function to update the reconstructed samples & output picture if ICH is selected
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param currLine  Current line reconstructed samples (modified) */
void UseICHistory(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int **currLine)
{
	int i;
	int cpnt;
	unsigned int p[NUM_COMPONENTS];
	int hPos;

	if (ICH_BITS==0)
		return;   // If ICH is disabled, do nothing

	// Start at beginning of group
	hPos = dsc_state->hPos - PIXELS_PER_GROUP + 1;
	
	// Apply ICH decision to reconstructed line & update ICH values.
	for (i=0; i<PIXELS_PER_GROUP; ++i)
	{
		if (dsc_state->ichSelected)
		{
			// We kept track of the history values the were selected when we checked if ICH was a good match for orig pixels
			p[0] = dsc_state->ichPixels[i][0];
			p[1] = dsc_state->ichPixels[i][1];
			p[2] = dsc_state->ichPixels[i][2];

			// Update reconstructed line and output picture
			for (cpnt=0; cpnt<NUM_COMPONENTS; ++cpnt)
				currLine[cpnt][hPos+i+PADDING_LEFT] = p[cpnt];
		}
	}
}


//! Encoder function to updated reconstructed pixels if midpoint prediction was selected
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param currLine  Current line reconstructed samples (modified)
	\param flag_first_luma For 4:2:2, flag indicating that only the first luma unit has been processed so far */
void UpdateMidpoint(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int **currLine)
{
	int i;
	int cpnt;
	int hPos;

	hPos = dsc_state->hPos - PIXELS_PER_GROUP + 1;
	
	// Apply ICH decision to reconstructed line & update ICH values.  Let's do this in raster order.
	for (i=0; i<PIXELS_PER_GROUP; ++i)
	{
		for (cpnt=0; cpnt<NUM_COMPONENTS; ++cpnt)  // Loop over how much component data for this pixel
		{
			if (dsc_state->midpointSelected[cpnt])
				currLine[cpnt][hPos+PADDING_LEFT] = dsc_state->midpointRecon[cpnt][i];
		}
		hPos++;
		if (hPos>=dsc_cfg->slice_width)
			return;
	}
}


//! Returns midpoint prediction predictor
/*! \param dsc_state DSC state structure 
	\param cpnt      Component index
	\param qlevel    Current quantization level */
int FindMidpoint(dsc_state_t *dsc_state, int cpnt, int qlevel)
{
	int range;

	// *MODEL NOTE* MN_MIDPOINT_PRED
	range = 1<<dsc_state->cpntBitDepth[cpnt];

	return (range/2 + (dsc_state->leftRecon[cpnt]%(1<<qlevel)));
}


//! Function to decide block vs. MAP & which block prediction vector to use for the next line
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure
	\param cpnt      Component index
	\param currLine  Current line's reconstructed samples
	\param hPos      Horizontal position in slice
	\param recon_x   Current reconstructed sample value */
void BlockPredSearch(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int cpnt, int **currLine, int hPos, int recon_x)
{
	int i, j;
	int candidate_vector;  // a value of 0 maps to -1, 2 maps to -3, etc.
	int pred_x;
	int pixel_mod_cnt;
	int min_err;
	PRED_TYPE min_pred;
	int cursamp;
	int pixdiff;
	int bp_sads[BP_RANGE];
	int modified_abs_diff;

	// This function runs right after a reconstructed value is determined and computes the best predictor to use for the NEXT line.
	// An implementation could run the block prediction search at any time after that up until the point at which the selection is needed.
	// *MODEL NOTE* MN_BP_SEARCH

	if (hPos == 0)
	{
		// Reset prediction accumulators every line
		dsc_state->bpCount = 0;
		dsc_state->lastEdgeCount = 10;  // Arbitrary large value as initial condition
		for (i=0; i<NUM_COMPONENTS; ++i)
		{
			for (j=0; j<BP_SIZE; ++j)
			{
				for (candidate_vector=0; candidate_vector<BP_RANGE; ++candidate_vector)
					dsc_state->lastErr[i][j][candidate_vector] = 0;
			}
		}
	}

	// Last edge count check - looks at absolute differences between adjacent pixels
	//   - Don't use block prediction if the content is basically flat
	pixdiff = SampToLineBuf(dsc_cfg, dsc_state, recon_x, cpnt) - 
		      SampToLineBuf(dsc_cfg, dsc_state, currLine[cpnt][hPos+PADDING_LEFT-1], cpnt);
	pixdiff = ABS(pixdiff);
	if (cpnt == 0)
		dsc_state->edgeDetected = 0;
	if (pixdiff > (BP_EDGE_STRENGTH << (dsc_cfg->bits_per_component-8)))
		dsc_state->edgeDetected = 1;
	if (cpnt == NUM_COMPONENTS-1)
	{
		if (dsc_state->edgeDetected)
			dsc_state->lastEdgeCount = 0;
		else
			dsc_state->lastEdgeCount++;
	}

	// The BP
	cursamp = (hPos/PRED_BLK_SIZE) % BP_SIZE;

	pixel_mod_cnt = hPos % PRED_BLK_SIZE;
	for ( candidate_vector=0; candidate_vector<BP_RANGE; candidate_vector++ ) {
		if ( pixel_mod_cnt == 0 ) {			// predErr is summed over PRED_BLK_SIZE pixels
			dsc_state->predErr[cpnt][candidate_vector] = 0;
		}

		pred_x = SamplePredict( dsc_state, currLine[cpnt], currLine[cpnt], hPos, (PRED_TYPE)(candidate_vector+PT_BLOCK), 0, cpnt );

		// HW uses previous line's reconstructed samples, which may be bit-reduced
		pred_x = SampToLineBuf(dsc_cfg, dsc_state, pred_x, cpnt);
		recon_x = SampToLineBuf(dsc_cfg, dsc_state, recon_x, cpnt);

		pixdiff = recon_x - pred_x;
		pixdiff = ABS(pixdiff);
		modified_abs_diff = MIN(pixdiff>>(dsc_state->cpntBitDepth[cpnt] - 7), 0x3f);
		// ABS differences are clamped to 6 bits each, predErr for 3 pixels is 8 bits
		dsc_state->predErr[cpnt][candidate_vector] += modified_abs_diff;
	}

	if ( pixel_mod_cnt == PRED_BLK_SIZE - 1 ) 
	{
		// Track last 3 3-pixel SADs for each component (each is 7 bit)
		for (candidate_vector=0; candidate_vector<BP_RANGE; ++candidate_vector)
			dsc_state->lastErr[cpnt][cursamp][candidate_vector] = dsc_state->predErr[cpnt][candidate_vector];

		if (cpnt<NUM_COMPONENTS-1)
			return;   // SAD is across all 3 components -- wait until we've processed all 3

		for (candidate_vector=0; candidate_vector<BP_RANGE; ++candidate_vector)
		{
			bp_sads[candidate_vector] = 0;

			for (i=0; i<BP_SIZE; ++i)
			{
				int sad3x1 = 0;

				// Use all 3 components
				for(j=0; j<NUM_COMPONENTS; ++j)
					 sad3x1 += dsc_state->lastErr[j][i][candidate_vector];
				// sad3x1 is 9 bits
				sad3x1 = MIN(511, sad3x1);

				bp_sads[candidate_vector] += sad3x1;  // 11-bit SAD
			}
			// Each bp_sad can have a max value of 63*9 pixels * 3 components = 1701 or 11 bits
			bp_sads[candidate_vector] >>= 3;  // SAD is truncated to 8-bit for comparison
		}

		min_err = 1000000;
		min_pred = PT_MAP;
		for (candidate_vector=0; candidate_vector<BP_RANGE; ++candidate_vector)
		{
			if (candidate_vector==1)
				continue;												// Can't use -2 vector
			// Ties favor smallest vector
			if (min_err > bp_sads[candidate_vector])
			{
				min_err = bp_sads[candidate_vector];
				min_pred = (PRED_TYPE)(candidate_vector+PT_BLOCK);
			} 
		}

		if (dsc_cfg->block_pred_enable && (hPos>=9))  // Don't start algorithm until 10th pixel
		{
			if (min_pred > PT_BLOCK)
				dsc_state->bpCount++;
			else
				dsc_state->bpCount = 0;
		}
		if ((dsc_state->bpCount>=3) && (dsc_state->lastEdgeCount < BP_EDGE_COUNT))
			dsc_state->prevLinePred[hPos/PRED_BLK_SIZE] = (PRED_TYPE)min_pred;
		else
			dsc_state->prevLinePred[hPos/PRED_BLK_SIZE] = (PRED_TYPE)PT_MAP;
	}
}


//! Function to compute number of bits required to code a given residual
/*! \param eq        Residual  */
int FindResidualSize(int eq)
{
	int size_e;

	// Find the size in bits of e
	if(eq == 0) size_e = 0;
	else if (eq >= -1 && eq <= 0) size_e = 1;
	else if (eq >= -2 && eq <= 1) size_e = 2;
	else if (eq >= -4 && eq <= 3) size_e = 3;
	else if (eq >= -8 && eq <= 7) size_e = 4;
	else if (eq >= -16 && eq <= 15) size_e = 5;
	else if (eq >= -32 && eq <= 31) size_e = 6;
	else if (eq >= -64 && eq <= 63) size_e = 7;
	else if (eq >= -128 && eq <= 127) size_e = 8;
	else if (eq >= -256 && eq <= 255) size_e = 9;
	else if (eq >= -512 && eq <= 511) size_e = 10;
	else if (eq >= -1024 && eq <= 1023) size_e = 11;
	else if (eq >= -2048 && eq <= 2047) size_e = 12;
	else if (eq >= -4096 && eq <= 4095) size_e = 13;
	else size_e = 14;

	return size_e;
}


//! Map QP to quantization level
/*! \param dsc_cfg   DSC configuration structure
	\param qp      QP to map
    \return        Flag if flatness information sent for the current supergroup */
int IsFlatnessInfoSent(dsc_cfg_t *dsc_cfg, int qp)
{
	return ((qp>=dsc_cfg->flatness_min_qp) && (qp<=dsc_cfg->flatness_max_qp));
}


//! Check if original pixels are flat at a specified location
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param hPos    Set to the horizontal location of the group we're testing - 1 */
int IsOrigFlatHIndex(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int hPos)
{
	int i, cpnt;
	int max, min;
	int p;
	int thresh[NUM_COMPONENTS], qp;
	int somewhat_flat = 1, very_flat = 1;

	qp = MAX(dsc_state->masterQp - 4, 0);
	thresh[0] = MapQpToQlevel(dsc_state, qp, 0);
	thresh[1] = thresh[2] = MapQpToQlevel(dsc_state, qp, 1);

	// If group starts past the end of the slice, it can't be flat
	if (hPos+1 >= dsc_cfg->slice_width)
		return (0);
	
	for (cpnt=0; cpnt<NUM_COMPONENTS; ++cpnt)
	{
		max = -1; min = 99999;
		for (i=0; i < 1 + PIXELS_PER_GROUP; ++i)
		{
			p = dsc_state->origLine[cpnt][PADDING_LEFT + MIN(dsc_cfg->slice_width-1, hPos+i)];
			if (max < p) max = p;
			if (min > p) min = p;
		}
		if (max - min > MAX(dsc_cfg->flatness_det_thresh, QuantDivisor[thresh[cpnt]]))
			somewhat_flat = 0;
		if (max - min > dsc_cfg->flatness_det_thresh)
			very_flat = 0;
	}
	if (very_flat)
		return (2);
	else if (somewhat_flat)
		return (1);

	// Skip flatness check 2 if it only contains a single pixel
	if (hPos+2 >= dsc_cfg->slice_width)
		return (0);

	somewhat_flat = very_flat = 1;
	// Left adjacent isn't flat, but current group & group to the right is flat
	for (cpnt=0; cpnt<NUM_COMPONENTS; ++cpnt)
	{
		max = -1; min = 99999;
		for (i=1; i< 4 + PIXELS_PER_GROUP; ++i)
		{
			p = dsc_state->origLine[cpnt][PADDING_LEFT + MIN(dsc_cfg->slice_width-1, hPos+i)];
			if (max < p) max = p;
			if (min > p) min = p;
		}
		if (max - min > MAX(dsc_cfg->flatness_det_thresh,QuantDivisor[thresh[cpnt]]))
			somewhat_flat = 0;
		if (max - min > dsc_cfg->flatness_det_thresh)
			very_flat = 0;
	}
	if (very_flat)
		return (2);
	else if (somewhat_flat)
		return (1);
	return (0);
}


//! Function to remove one pixel's worth of bits from the encoder buffer model
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure */
void RemoveBitsEncoderBuffer(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state)
{
	dsc_state->bpgFracAccum += (dsc_cfg->bits_per_pixel & 0xf);
	dsc_state->bufferFullness -= (dsc_cfg->bits_per_pixel >> 4) + (dsc_state->bpgFracAccum>>4);
	dsc_state->numBitsChunk += (dsc_cfg->bits_per_pixel >> 4) + (dsc_state->bpgFracAccum>>4);
	dsc_state->bpgFracAccum &= 0xf;
	dsc_state->chunkPixelTimes++;
	if (dsc_state->chunkPixelTimes >= dsc_cfg->slice_width)
	{
		int adjustment_bits;
		if (dsc_cfg->vbr_enable)
		{
			int size;
			size = (dsc_state->numBitsChunk - dsc_state->bitsClamped + 7) / 8;
			adjustment_bits = size * 8 - (dsc_state->numBitsChunk - dsc_state->bitsClamped);
			dsc_state->bufferFullness -= adjustment_bits;
			dsc_state->bitsClamped = 0;
			if (dsc_state->isEncoder)
				dsc_state->chunkSizes[dsc_state->chunkCount] = size;
		}
		else   // CBR mode
		{
			adjustment_bits = dsc_cfg->chunk_size * 8 - dsc_state->numBitsChunk;
			dsc_state->bufferFullness -= adjustment_bits;
		}
		dsc_state->bpgFracAccum = 0;
		dsc_state->numBitsChunk = 0;
		dsc_state->chunkCount++;
		dsc_state->chunkPixelTimes = 0;
	}
}


//! Rate control function
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param throttle_offset  Offset to apply to buffer fullness before doing threshold comparisons
	\param bpg_offset  Bits per group adjustment for current group
	\param group_count  Current group within slice
	\param scale     Scale factor to apply to buffer fullness before doing threshold comparisons
	\param group_size Number of pixels actually in group (could be smaller than nominal group size for partial groups) */
void RateControl( dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int throttle_offset, int bpg_offset, int group_count, int scale, int group_size )
{
	int i;
	int prev_fullness;
	int rcSizeGroup;
	int rcTgtBitsGroup;
	int overflowAvoid;
	int min_QP;
	int max_QP;
	int tgtMinusOffset;
	int tgtPlusOffset;
	dsc_range_cfg_t *range_cfg;
	int rcSizeGroupPrev  = dsc_state->rcSizeGroup;
	int prevQp            = dsc_state->stQp;
	int prev2Qp           = dsc_state->prevQp;
	int rcModelBufferFullness;
	int bpg;
	int incr_amount;
	int selected_range;
	int stQp;
	int curQp;

	prev_fullness = dsc_state->bufferFullness;
	
	for (i=0; i<group_size; ++i)
	{
		dsc_state->pixelCount++;
		if (dsc_state->pixelCount >= dsc_cfg->initial_xmit_delay)
			RemoveBitsEncoderBuffer(dsc_cfg, dsc_state);
	}

	// Add up estimated bits for the Group, i.e. as if VLC sample size matched max sample size
	rcSizeGroup = 0;
	for (i = 0; i < NUM_COMPONENTS; i++)
		rcSizeGroup += dsc_state->rcSizeUnit[i];

	if (PRINT_DEBUG_RC)
	{
		fprintf(g_fp_dbg, "RC group #%d, scale=%d, offset=%d, bpg_offset=%d, ", group_count, scale, throttle_offset, bpg_offset);
		fprintf(g_fp_dbg, "RCSizeGroup=%d, coded_group_size=%d\n", rcSizeGroup, dsc_state->codedGroupSize);
		fprintf(g_fp_dbg, "Buffer fullness=%d, ", dsc_state->bufferFullness);
	}

	// Set target number of bits per Group according to buffer fullness
	range_cfg = dsc_cfg->rc_range_parameters;
	throttle_offset -= dsc_cfg->rc_model_size;

	// *MODEL NOTE* MN_RC_XFORM
	rcModelBufferFullness = (scale * (dsc_state->bufferFullness + throttle_offset)) >> RC_SCALE_BINARY_POINT;
	if (PRINT_DEBUG_RC)
		fprintf(g_fp_dbg, "RC model buffer fullness=%d\n", rcModelBufferFullness);

	// Pick the correct range
	// *MODEL NOTE* MN_RC_LONG_TERM
	for (i=NUM_BUF_RANGES-1; i>0; --i)
	{
		overflowAvoid = (dsc_state->bufferFullness + throttle_offset > OVERFLOW_AVOID_THRESHOLD);
		if ((rcModelBufferFullness > dsc_cfg->rc_buf_thresh[i-1]-dsc_cfg->rc_model_size) )
			break;
	}

	if (rcModelBufferFullness > 0)
	{
		printf("The RC model has overflowed.  To address this issue, please adjust the\n");
		printf("min_QP and max_QP higher for the top-most ranges, or decrease the rc_buf_thresh\n");
		printf("for those ranges.\n");
		exit(1);
	}

	// Add a group time of delay to RC calculation
	selected_range = dsc_state->prevRange;
	dsc_state->prevRange = i;

	// *MODEL NOTE* MN_RC_SHORT_TERM
	bpg = (dsc_cfg->bits_per_pixel * group_size);
	bpg = (bpg + 8) >> 4;
	rcTgtBitsGroup = MAX( 0, bpg + range_cfg[selected_range].range_bpg_offset + bpg_offset );
	min_QP = range_cfg[selected_range].range_min_qp;
	max_QP = range_cfg[selected_range].range_max_qp;

	if (PRINT_DEBUG_RC)
		fprintf(g_fp_dbg, "Range %d: MIN=%d, MAX=%d, QuantPrev=%d, ", selected_range, min_QP, max_QP, dsc_state->stQp);

	tgtMinusOffset = MAX( 0, rcTgtBitsGroup - dsc_cfg->rc_tgt_offset_lo );
	tgtPlusOffset  = MAX( 0, rcTgtBitsGroup + dsc_cfg->rc_tgt_offset_hi );
	incr_amount = (dsc_state->codedGroupSize - rcTgtBitsGroup) >> 1;

    if (rcSizeGroup==PIXELS_PER_GROUP)
	{
		stQp = MAX(min_QP/2, prevQp-1);
	} 
	else if ( (dsc_state->codedGroupSize < tgtMinusOffset) && (rcSizeGroup < tgtMinusOffset) ) 
	{
		stQp = MAX(min_QP, (prevQp-1));
	}
	// avoid increasing QP immediately after edge
	else if ( (dsc_state->bufferFullness >= 64) &&  dsc_state->codedGroupSize > tgtPlusOffset )  // over budget - increase qp
	{ 
		curQp = MAX(prevQp, min_QP);
		if (prev2Qp == curQp)   // 2nd prev grp == prev grp 
		{   
			if ( (rcSizeGroup*2) < (rcSizeGroupPrev*dsc_cfg->rc_edge_factor) )
				stQp = MIN(max_QP, (curQp+incr_amount));
			else
				stQp = curQp;
		}
		else if (prev2Qp < curQp) 
		{
			if ( ( (rcSizeGroup*2) < (rcSizeGroupPrev*dsc_cfg->rc_edge_factor) && (curQp < dsc_cfg->rc_quant_incr_limit0)) ) 
				stQp = MIN(max_QP, (curQp+incr_amount));
			else
				stQp = curQp;
		}
		else if ( curQp < dsc_cfg->rc_quant_incr_limit1 ) 
		{
			stQp = MIN(max_QP, (curQp+incr_amount));
		}
		else
		{
			stQp = curQp;
		}
	} else
		stQp = prevQp;

	if ( overflowAvoid )
		stQp = range_cfg[NUM_BUF_RANGES-1].range_max_qp;

	rcSizeGroupPrev = rcSizeGroup;

	dsc_state->rcSizeGroup  = rcSizeGroup;
	dsc_state->stQp         = stQp;
	dsc_state->prevQp       = prevQp;

	if (PRINT_DEBUG_RC)
		fprintf(g_fp_dbg, "New quant value=%d\n", stQp);

	if ( dsc_state->bufferFullness > dsc_cfg->rcb_bits ) {
		printf("The buffer model has overflowed.  This probably occurred due to an error in the\n");
		printf("rate control parameter programming or an attempt to decode an invalid DSC stream.\n\n");
		printf( "ERROR: RCB overflow; size is %d, tried filling to %d\n", dsc_cfg->rcb_bits, dsc_state->bufferFullness );
		printf( "  previous level: %5d\n", prev_fullness );
		printf( "  target BPP:     %5d\n", dsc_cfg->bits_per_pixel >> 4 );
		printf( "  new level:      %5d\n", dsc_state->bufferFullness );
		printf( "Range info\n" );
		printf( "  range min_qp:     %d\n", range_cfg->range_min_qp );
		printf( "  range max_qp:     %d\n", range_cfg->range_max_qp );
		printf( "  range bpg_offset: %d\n", range_cfg->range_bpg_offset );
		range_cfg--;
		printf( "Previous range info\n" );
		printf( "  range min_qp:     %d\n", range_cfg->range_min_qp );
		printf( "  range max_qp:     %d\n", range_cfg->range_max_qp );
		exit(1);
	}
}


//! Predict size for next unit
/*! \param dsc_cfg   DSC configuration structure
    \param req_size  Array of required sizes for the samples in the unit
	\return          Predicted size for next unit */
int PredictSize(dsc_cfg_t *dsc_cfg, int *req_size)
{
	int pred_size;

	pred_size = 2;  // +0.5 for rounding
	pred_size += req_size[0]+req_size[1];
	pred_size += 2*req_size[2];
	pred_size >>= 2; 

	return (pred_size);
}


//! Size to code that means "escape code"
/*! \param dsc_state DSC state structure 
	\param qp        Quantization parameter for group */
int EscapeCodeSize(dsc_state_t *dsc_state, int qp)
{
	int qlevel, alt_size_to_generate;

	qlevel = MapQpToQlevel(dsc_state, qp, 0);    // map to luma quant
	alt_size_to_generate = dsc_state->cpntBitDepth[0] + 1 - qlevel;

	return (alt_size_to_generate);
}


//! Encoder function that returns 1 if midpoint prediction will be used for the specified component
/*! \param dsc_state DSC state structure 
	\param cpnt      Component to check */
int UsingMidpoint(dsc_state_t *dsc_state, int cpnt)
{
	int qlevel, max_size, req_size[SAMPLES_PER_UNIT];
	int i;

	// Determine required size for unit
	qlevel = MapQpToQlevel(dsc_state, dsc_state->masterQp, cpnt);
	max_size = 0;
	for ( i=0; i<SAMPLES_PER_UNIT; i++ ) 
	{
		req_size[i] = FindResidualSize( dsc_state->quantizedResidual[cpnt][i] );
		max_size = MAX( req_size[i], max_size );
	}

	// Check if required size is bpc-qlevel ...
	if ( max_size >= dsc_state->cpntBitDepth[cpnt] - qlevel )  // ...if so, just use midpoint predictor
		return (1);
	else
		return (0);
}


//! Code one unit
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param cpnt      Component to code
	\param quantized_residuals Quantized residuals to code */
void VLCUnit(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int cpnt, int *quantized_residuals)
{
	int required_size[NUM_COMPONENTS];
	int size;
	int max_size;
	int prefix_value;
	int adj_predicted_size;
	int i;
	int qlevel;
	int max_pfx_size;
	int qp;
	int unit_to_send_fflag = 0, unit_to_send_fpos = 0;
	int all_orig_within_qerr;

	assert(cpnt<=NUM_COMPONENTS);
	// *MODEL NOTE* MN_ENC_VLC_UNIT

	qp = dsc_state->masterQp;

	qlevel = MapQpToQlevel(dsc_state, dsc_state->masterQp, cpnt);

	adj_predicted_size = GetQpAdjPredSize(dsc_state, cpnt);

	if (cpnt==0)
		dsc_state->prevIchSelected = dsc_state->ichSelected;

	if ((cpnt == unit_to_send_fflag) && ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP) == 3) && 
		IsFlatnessInfoSent(dsc_cfg, qp))
	{
		if (dsc_state->prevFirstFlat<0)
		{
			if (PRINT_DEBUG_VLC)
				fprintf(g_fp_dbg, "Flatness flag=0\n");
			AddBits(dsc_cfg, dsc_state, cpnt, 0, 1);
		}
		else
		{
			if (PRINT_DEBUG_VLC)
				fprintf(g_fp_dbg, "Flatness flag=1\n");
			AddBits(dsc_cfg, dsc_state, cpnt, 1, 1);
		}
	}
	if ((cpnt == unit_to_send_fpos) && ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP) == 0) && 
		(dsc_state->firstFlat >= 0))
	{
		if (dsc_state->masterQp >= SOMEWHAT_FLAT_QP_THRESH)
			AddBits(dsc_cfg, dsc_state, cpnt, dsc_state->flatnessType, 1);
		else
			dsc_state->flatnessType = 0;

		AddBits(dsc_cfg, dsc_state, cpnt, dsc_state->firstFlat, 2);
		if (PRINT_DEBUG_VLC)
			fprintf(g_fp_dbg, "First flat: %d, type: %d\n", dsc_state->firstFlat, dsc_state->flatnessType);
	}
	if ((cpnt>0) && dsc_state->ichSelected)
		return;			// Nothing to do for other components once ICH selected

	// figure out required size for unit
	max_size = 0;
	for ( i=0; i<SAMPLES_PER_UNIT; i++ ) {
		required_size[i] = FindResidualSize( quantized_residuals[i] );
		max_size = MAX( required_size[i], max_size );
	}
	
	if (PRINT_DEBUG_VLC)
	{
		fprintf(g_fp_dbg, "Component %d : ", cpnt);
		if (cpnt == 0)
			fprintf(g_fp_dbg, "uq=%d, ", dsc_state->masterQp);
	}

	// Check if required size is bpc-qlevel ...
	if ( dsc_state->forceMpp || (max_size >= dsc_state->cpntBitDepth[cpnt] - qlevel ))  // ...if so, just use midpoint predictor
	{
		max_size = dsc_state->cpntBitDepth[cpnt] - qlevel;
		for (i=0; i<SAMPLES_PER_UNIT; ++ i)
			required_size[i] = max_size;
	}

	if (adj_predicted_size < max_size ) 
	{
		prefix_value = max_size - adj_predicted_size;
		size = max_size;
	}
	else 
	{
		prefix_value = 0; // size did not grow
		size = adj_predicted_size;
	}

	if (cpnt==0) // Escape code becomes size=0 if previous group was ICH mode, all other values get boosted by 1
		prefix_value += dsc_state->prevIchSelected;

	// Determine if all 3 pixels are withing the quantization error
	dsc_state->ichSelected = 0;
	all_orig_within_qerr = 1;
	for (i=0; i<PIXELS_PER_GROUP; ++i)
		if (!dsc_state->origWithinQerr[i])
			all_orig_within_qerr = 0;

	// ICH mode selection
	if ((cpnt==0) && all_orig_within_qerr && !dsc_state->forceMpp)
	{
		int bits_ich_mode, bits_p_mode;
		int alt_pfx, alt_size_to_generate;
		int log_err_ich_mode, log_err_p_mode;
		int max_err_p_mode[3];

		// *MODEL NOTE* MN_ENC_ICH_MODE_SELECT
		alt_size_to_generate = EscapeCodeSize(dsc_state, dsc_state->masterQp);
		if (dsc_state->prevIchSelected)
		{
			alt_pfx = 0;
			bits_ich_mode = 1;
		}
		else  // For escape code, no need to send trailing one for prefix
			bits_ich_mode = alt_pfx = alt_size_to_generate - adj_predicted_size;
		bits_ich_mode += ICH_BITS * PIXELS_PER_GROUP;  // ICH indices

		for (i=0; i<NUM_COMPONENTS; ++i)
		{
			if (UsingMidpoint(dsc_state, i))
				max_err_p_mode[i] = dsc_state->maxMidError[i];
			else
				max_err_p_mode[i] = dsc_state->maxError[i];
		}

		log_err_ich_mode = 2 * ceil_log2(dsc_state->maxIchError[0]) +
			                 ceil_log2(dsc_state->maxIchError[1]) +
							 ceil_log2(dsc_state->maxIchError[2]);
		log_err_p_mode = 2 * ceil_log2(max_err_p_mode[0]) +
			                 ceil_log2(max_err_p_mode[1]) +
							 ceil_log2(max_err_p_mode[2]);
		bits_p_mode = EstimateBitsForGroup(dsc_cfg, dsc_state);

		// ICH mode decision
		if ((log_err_ich_mode <= log_err_p_mode) &&
		    (bits_ich_mode + 4 * log_err_ich_mode < bits_p_mode + 4 * log_err_p_mode))
		{
			dsc_state->ichSelected = 1;
			if (PRINT_DEBUG_VLC)
			{
				fprintf(g_fp_dbg, "size = pred (%d) + pfx (%d) = %d\n", adj_predicted_size, alt_pfx, adj_predicted_size + alt_pfx - dsc_state->prevIchSelected);
				fprintf(g_fp_dbg, "ICH mode (ecs=%d); indices: %d %d %d \n", alt_size_to_generate, dsc_state->ichLookup[0],
								dsc_state->ichLookup[1], dsc_state->ichLookup[2]); 
			}

			if (dsc_state->prevIchSelected)
				AddBits(dsc_cfg, dsc_state, cpnt, 1, alt_pfx+1);
			else
				AddBits(dsc_cfg, dsc_state, cpnt, 0, alt_pfx);

			// Put IC$ samples
			for (i=0; i<PIXELS_PER_GROUP; ++i)
				AddBits(dsc_cfg, dsc_state, i, dsc_state->ichLookup[i], ICH_BITS);

			dsc_state->rcSizeUnit[0] = ICH_BITS + 1;
			dsc_state->rcSizeUnit[1] = dsc_state->rcSizeUnit[2] = ICH_BITS;
			// Note that predicted size is not updated
			return;
		}	
	}

	// No need to send prefix if predicted size is max
	max_pfx_size = MaxResidualSize(dsc_state, cpnt, dsc_state->masterQp) + (cpnt==0) - adj_predicted_size;

	if (PRINT_DEBUG_VLC)
		fprintf(g_fp_dbg, "size = pred (%d) + pfx (%d) = %d\n", adj_predicted_size, prefix_value, size);
	
	// Write bits to output bitstream
	if (prefix_value == max_pfx_size)  // Trailing "1" can be omitted
		AddBits(dsc_cfg, dsc_state, cpnt, 0, max_pfx_size);
	else
		AddBits(dsc_cfg, dsc_state, cpnt, 1, prefix_value+1);

	// *MODEL NOTE* MN_ENC_MPP_SELECT
	for ( i=0; i<SAMPLES_PER_UNIT; i++ ) 
	{
		if (max_size == dsc_state->cpntBitDepth[cpnt] - qlevel)
		{
			AddBits(dsc_cfg, dsc_state, cpnt, dsc_state->quantizedResidualMid[cpnt][i], size);
			if (PRINT_DEBUG_VLC)
				fprintf(g_fp_dbg, "Sample delta %d = %d\n", i, dsc_state->quantizedResidualMid[cpnt][i]);
			dsc_state->midpointSelected[cpnt] = 1;
		}
		else
		{
			AddBits(dsc_cfg, dsc_state, cpnt, quantized_residuals[i], size);
			if (PRINT_DEBUG_VLC)
				fprintf(g_fp_dbg, "Sample delta %d = %d\n", i, quantized_residuals[i]);
			dsc_state->midpointSelected[cpnt] = 0;
		}
	}

	// rate control size uses max required size plus 1 (for prefix value of 0)
	dsc_state->rcSizeUnit[cpnt] = max_size*SAMPLES_PER_UNIT + 1;

	//
	// Predict size for next unit for this component
	// & store predicted size to state structure
	dsc_state->predictedSize[cpnt] = PredictSize(dsc_cfg, required_size);;
}


//! Code one unit
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param byte_out_p  Pointer to compressed bits buffer (modified) */
void VLCGroup(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, unsigned char **byte_out_p)
{
	int i;
	int start_fullness[NUM_COMPONENTS];
	int adjFullness;
	int maxBitsPerGroup;
	
	for (i=0; i<NUM_COMPONENTS; ++i)
	{
		dsc_state->midpointSelected[i] = 0;
		start_fullness[i] = dsc_state->encBalanceFifo[i].fullness;
	}

	// 444; Unit is same as CType
	dsc_state->prevNumBits = dsc_state->numBits;

	// Check stuffing condition
	dsc_state->forceMpp = 0;
	// Force MPP mode if buffer fullness is low
	//  Buffer threshold is ceil(bpp * 3) - 3, the first term is how many
	//   bits are removed from the model, the second term (3) is the minimum
	//   number of bits that a group can be coded with		
	maxBitsPerGroup = (3 * dsc_cfg->bits_per_pixel + 15) >> 4;
	adjFullness = dsc_state->bufferFullness;
	if(dsc_state->numBitsChunk + maxBitsPerGroup + 8 > dsc_cfg->chunk_size * 8)
	{
		// End of chunk check to see if there is a potential to underflow 
		// assuming adjustment bits are sent.
		adjFullness -= 8;
		if (adjFullness  < maxBitsPerGroup - 3)  // Force MPP is possible in VBR only at end of line to pad chunks to byte boundaries
			dsc_state->forceMpp = 1;
	} 
	else if((!dsc_cfg->vbr_enable) && (dsc_state->pixelCount >= dsc_cfg->initial_xmit_delay))  // underflow isn't possible if we're not removing bits
	{
		if (adjFullness  < maxBitsPerGroup - 3)
			dsc_state->forceMpp = 1;
	}

	for (i=0; i<NUM_COMPONENTS; ++i)
		VLCUnit(dsc_cfg, dsc_state, i, dsc_state->quantizedResidual[i]);

	// Keep track of fullness for each coded unit in the balance FIFO's
	for (i=0; i<NUM_COMPONENTS; ++i)
		fifo_put_bits(&(dsc_state->seSizeFifo[i]), dsc_state->encBalanceFifo[i].fullness - start_fullness[i], 6);

	if (dsc_cfg->muxing_mode == 0)  // Write data immedately to buffer
		WriteEntryToBitstream(dsc_cfg, dsc_state, *byte_out_p);
	else if (dsc_cfg->muxing_mode)  // substream muxing
	{
		//if (dsc_state->groupCount > dsc_cfg->mux_word_size + MAX_SE_SIZE - 1)
		if (dsc_state->groupCount > dsc_cfg->mux_word_size + MAX_SE_SIZE - 3)
			ProcessGroupEnc(dsc_cfg, dsc_state, *byte_out_p);
	}

	dsc_state->bufferFullness += dsc_state->numBits - dsc_state->prevNumBits;
	if ( dsc_state->bufferFullness > dsc_cfg->rcb_bits ) {
		// This check may actually belong after tgt_bpg has been subtracted
		printf("The buffer model has overflowed.  This probably occurred due to an error in the\n");
		printf("rate control parameter programming.\n\n");
		printf( "ERROR: RCB overflow; size is %d, tried filling to %d\n", dsc_cfg->rcb_bits, dsc_state->bufferFullness );
		exit(1);
	}
	dsc_state->codedGroupSize = dsc_state->numBits - dsc_state->prevNumBits;

	dsc_state->prevMasterQp = dsc_state->masterQp;
	dsc_state->groupCountLine++;
}


//! Decode one unit
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure
	\param cpnt      Component to code
	\param quantized_residuals Quantized residuals
	\param byte_in_p Pointer to compressed bits buffer (modified) */
void VLDUnit(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int cpnt, int *quantized_residuals, unsigned char **byte_in_p)
{
	int required_size[NUM_COMPONENTS];
	int size;
	int max_size;
	int prefix_value;
	int adj_predicted_size;
	int i;
	int alt_size_to_generate;
	int use_ich;
	int qlevel;
	int max_prefix;
	int midpoint_selected;
	int qp;
	int unit_to_read_fflag, unit_to_read_fpos;

	qp = dsc_state->masterQp;

	unit_to_read_fflag = 0;
	unit_to_read_fpos = 0;

	qlevel = MapQpToQlevel(dsc_state, dsc_state->masterQp, cpnt);

	adj_predicted_size = GetQpAdjPredSize(dsc_state, cpnt);
	if (cpnt==0)
	{
		dsc_state->prevIchSelected = dsc_state->ichSelected;
		dsc_state->ichSelected = 0;
	}
	if ((cpnt==unit_to_read_fflag) && ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP) == 3))
	{
		if (IsFlatnessInfoSent(dsc_cfg, qp))
		{
			if (GetBits(dsc_cfg, dsc_state, cpnt, 1, 0, *byte_in_p))
				dsc_state->prevFirstFlat = 0;
			else
				dsc_state->prevFirstFlat = -1;
			if (PRINT_DEBUG_VLC)
				fprintf(g_fp_dbg, "Flatness flag=%d\n", (dsc_state->prevFirstFlat==0));
		} else
			dsc_state->prevFirstFlat = -1;
	}
	if ((cpnt==unit_to_read_fpos) && ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP) == 0))
	{
	    if (dsc_state->prevFirstFlat >= 0)
		{
			dsc_state->flatnessType = 0;
			if (dsc_state->masterQp >= SOMEWHAT_FLAT_QP_THRESH)
				dsc_state->flatnessType = GetBits(dsc_cfg, dsc_state, cpnt, 1, 0, *byte_in_p);
			dsc_state->firstFlat = GetBits(dsc_cfg, dsc_state, cpnt, 2, 0, *byte_in_p);
			if (PRINT_DEBUG_VLC)
				fprintf(g_fp_dbg, "First flat: %d, type: %d\n", dsc_state->firstFlat, dsc_state->flatnessType);
		}
		else
			dsc_state->firstFlat = -1;
	}
	if (dsc_state->ichSelected)
		return;							// Don't read other 2 components if we're in ICH mode

	max_prefix = MaxResidualSize(dsc_state, cpnt, dsc_state->masterQp) + (cpnt==0) - adj_predicted_size; // +(CType==0) for escape code
	prefix_value = 0;
	while ((prefix_value < max_prefix) && !GetBits(dsc_cfg, dsc_state, cpnt, 1, 0, *byte_in_p))
		prefix_value++;

	if (PRINT_DEBUG_VLC)
	{
		fprintf(g_fp_dbg, "Component %d : ", cpnt);
		if (cpnt == 0)
			fprintf(g_fp_dbg, "uq=%d, ", dsc_state->masterQp);
	}

	if ((cpnt==0) && dsc_state->prevIchSelected)
		size = adj_predicted_size + prefix_value - 1;
	else
		size = adj_predicted_size + prefix_value;
	
	if (PRINT_DEBUG_VLC)
		fprintf(g_fp_dbg, "size = pred (%d) + pfx (%d) = %d\n", adj_predicted_size, prefix_value, size);
	
	alt_size_to_generate = EscapeCodeSize(dsc_state, dsc_state->masterQp);

	if (dsc_state->prevIchSelected)
		use_ich = (prefix_value==0);
	else
		use_ich = (size >= alt_size_to_generate);

	if ((cpnt==0) && use_ich)  // ICH hit
	{
		dsc_state->ichSelected = 1;
		if (PRINT_DEBUG_VLC)
			fprintf(g_fp_dbg, "ICH mode (ecs=%d); indices: ", alt_size_to_generate);
		for (i=0; i<PIXELS_PER_GROUP; ++i)
		{
			dsc_state->ichLookup[i] = GetBits(dsc_cfg, dsc_state, i, ICH_BITS, 0, *byte_in_p);
			if (PRINT_DEBUG_VLC)
				fprintf(g_fp_dbg, "%d ", dsc_state->ichLookup[i]);
		}
		if (PRINT_DEBUG_VLC)
			fprintf(g_fp_dbg, "\n");
		dsc_state->rcSizeUnit[0] = ICH_BITS + 1;
		dsc_state->rcSizeUnit[1] = dsc_state->rcSizeUnit[2] = ICH_BITS;
		
		return;
	}

	// *MODEL NOTE* MN_DEC_MPP_SELECT
	midpoint_selected = (size==dsc_state->cpntBitDepth[cpnt] - qlevel);
	dsc_state->useMidpoint[cpnt] = midpoint_selected;

	// Get bits from input bitstream
	max_size = 0;
	for ( i=0; i<SAMPLES_PER_UNIT; i++ ) {
		quantized_residuals[i] = GetBits(dsc_cfg, dsc_state, cpnt, size, 1, *byte_in_p);
		if (PRINT_DEBUG_VLC)
			fprintf(g_fp_dbg, "Sample delta %d = %d\n", i, quantized_residuals[i]);
		required_size[i] = FindResidualSize( quantized_residuals[i] );
		max_size = MAX( required_size[i], max_size );
	}

	if (midpoint_selected)
	{
		max_size = size;
		for (i=0; i<SAMPLES_PER_UNIT; ++i)
			required_size[i] = size;
	}

	// rate control size uses max required size plus 1 (for prefix value of 0)
	dsc_state->rcSizeUnit[cpnt] = max_size*SAMPLES_PER_UNIT + 1;

	//
	// Predict size for next unit for this component
	// and store predicted size to state structure
	dsc_state->predictedSize[cpnt] = PredictSize(dsc_cfg, required_size);
}


//! Decode one group
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure
	\param byte_in_p Pointer to compressed data buffer (modified) */
void VLDGroup(dsc_cfg_t* dsc_cfg, dsc_state_t* dsc_state, unsigned char** byte_in_p)
{
	int prevNumBits = dsc_state->numBits;
	int i;

	if (dsc_cfg->muxing_mode)
		ProcessGroupDec(dsc_cfg, dsc_state, *byte_in_p);

	// *MODEL NOTE* MN_DEC_ENTROPY
	// 444; Unit is same as CType
	for (i=0; i<NUM_COMPONENTS; ++i)
		VLDUnit(dsc_cfg, dsc_state, i, dsc_state->quantizedResidual[i], byte_in_p);

	dsc_state->prevMasterQp = dsc_state->masterQp;
	dsc_state->codedGroupSize = dsc_state->numBits - prevNumBits;
	dsc_state->bufferFullness += dsc_state->codedGroupSize;

	if ( dsc_state->bufferFullness > dsc_cfg->rcb_bits ) {
		// This check may actually belong after tgt_bpg has been subtracted
		printf("The buffer model has overflowed.  This probably occurred due to an attempt to decode an invalid DSC stream.\n\n");
		printf( "ERROR: RCB overflow; size is %d, tried filling to %d\n", dsc_cfg->rcb_bits, dsc_state->bufferFullness );
		exit(1);
	}
	
	if ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP) == dsc_state->firstFlat)
		dsc_state->origIsFlat = 1;
	else
		dsc_state->origIsFlat = 0;
	dsc_state->groupCountLine++;
}


//! Reduce the bits required for a previous reconstructed line sample (currently via rounding)
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure
	\param x         The sample
	\param cpnt      Component to process */
int SampToLineBuf( dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int x, int cpnt)
{
	int shift_amount, round, storedSample;

	// *MODEL NOTE* MN_LINE_STORAGE
	shift_amount = MAX(dsc_state->cpntBitDepth[cpnt] - dsc_cfg->linebuf_depth, 0);
	if (shift_amount > 0)
		round = 1<<(shift_amount-1);
	else
		round = 0;

	storedSample = MIN(((x+round)>>shift_amount), (1<<dsc_cfg->linebuf_depth) - 1);
	return (storedSample << shift_amount);	
}


//! Initialize the DSC state
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\return          Returns dsc_state that was passed in */
dsc_state_t *InitializeDSCState( dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state )
{
	int i, j, k;

	memset(dsc_state, 0, sizeof(dsc_state_t));

	// Initialize quantization table
	if (dsc_cfg->bits_per_component == 12)
	{
		dsc_state->quantTableLuma = qlevel_luma_12bpc;
		dsc_state->quantTableChroma = qlevel_chroma_12bpc;
	} else if (dsc_cfg->bits_per_component == 10) {
		dsc_state->quantTableLuma = qlevel_luma_10bpc;
		dsc_state->quantTableChroma = qlevel_chroma_10bpc;
	} else {
		dsc_state->quantTableLuma = qlevel_luma_8bpc;
		dsc_state->quantTableChroma = qlevel_chroma_8bpc;
	}

	// initialize dsc state
	dsc_state->numBits            = 0;
	dsc_state->prevNumBits        = 0;
	dsc_state->bufferFullness     = 0;
	dsc_state->stQp               = 0;
	dsc_state->prevQp             = 0;
	dsc_state->masterQp           = 0;
	dsc_state->prevMasterQp       = 0;
	dsc_state->rcSizeGroup        = 0;
	dsc_state->nonFirstLineBpgTarget = 0;
	dsc_state->rcXformOffset     = dsc_cfg->initial_offset;
	dsc_state->throttleInt       = 0;
	dsc_state->throttleFrac      = 0;
	dsc_state->firstFlat         = -1;
	dsc_state->prevFirstFlat     = -1;
	for ( i=0; i<NUM_COMPONENTS; i++ ) {
		dsc_state->predictedSize[i] = 0;
		for ( j=0; j<SAMPLES_PER_UNIT; j++ )
			dsc_state->quantizedResidual[i][j] = 0;
	}
	for ( i=0; i<MAX_UNITS_PER_GROUP; i++ ) {
		dsc_state->rcSizeUnit[i] = 0;
	}

	for (i=0; i<NUM_COMPONENTS; ++i)
	{	
		for(j=0; j<BP_RANGE; ++j)
		{
			for(k=0; k<BP_SIZE; ++k)
				dsc_state->lastErr[i][k][j] = 0;
		}
	}

	dsc_state->prevLinePred = (PRED_TYPE *)malloc(sizeof(PRED_TYPE) * (dsc_cfg->slice_width+PRED_BLK_SIZE-1) / PRED_BLK_SIZE);
	assert(dsc_state->prevLinePred != NULL);
	// Sets last predictor of line to MAP, since BP search is not done for partial groups
	memset(dsc_state->prevLinePred, 0, sizeof(PRED_TYPE) * (dsc_cfg->slice_width+PRED_BLK_SIZE-1) / PRED_BLK_SIZE);  
	
	dsc_state->history.valid = (int *)malloc(sizeof(int)*ICH_SIZE);
	assert(dsc_state->history.valid != NULL);
	for(i=0; i<ICH_SIZE; ++i)
		dsc_state->history.valid[i] = 0;
	for(i=0; i<NUM_COMPONENTS; ++i)
	{
		dsc_state->history.pixels[i] = (unsigned int*)malloc(sizeof(unsigned int)*ICH_SIZE);
	}
	dsc_state->ichSelected = 0;

	for(i=0; i<NUM_COMPONENTS; ++i)
	{
		fifo_init(&(dsc_state->shifter[i]), (dsc_cfg->mux_word_size + MAX_SE_SIZE + 7) / 8);
		fifo_init(&(dsc_state->encBalanceFifo[i]), ((dsc_cfg->mux_word_size + MAX_SE_SIZE - 1) * (MAX_SE_SIZE) + 7)/8);
		fifo_init(&(dsc_state->seSizeFifo[i]), (6 * (dsc_cfg->mux_word_size + MAX_SE_SIZE - 1) + 7)/8 );
	}

	return dsc_state;
}


//! Calculate offset & scale values for rate control
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param vPos    Current vertical position in slice
	\param group_count Group counter
	\param scale     Scale factor for RC (returned)
	\param bpgOffset BPG Offset value for RC (returned)  */
void CalcFullnessOffset(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, int vPos, int group_count, int *scale, int *bpg_offset)
{
	// positive offset takes bits away, negative offset provides more bits
	int current_bpg_target;
	int unity_scale;
	int increment;

	// *MODEL NOTE* MN_CALC_SCALE_OFFSET
	unity_scale = (1<<RC_SCALE_BINARY_POINT);

	if (group_count == 0)
	{
		dsc_state->currentScale = dsc_cfg->initial_scale_value;
		dsc_state->scaleAdjustCounter = 1;
	} 
	else if ((vPos == 0) && (dsc_state->currentScale > unity_scale))  // Reduce scale at beginning of slice
	{
		dsc_state->scaleAdjustCounter++;
		if(dsc_state->scaleAdjustCounter >= dsc_cfg->scale_decrement_interval)
		{
			dsc_state->scaleAdjustCounter = 0;
			dsc_state->currentScale--;
		}
	}
	else if (dsc_state->scaleIncrementStart)
	{
		dsc_state->scaleAdjustCounter++;
		if (dsc_state->scaleAdjustCounter >= dsc_cfg->scale_increment_interval)
		{
			dsc_state->scaleAdjustCounter = 0;
			dsc_state->currentScale++;
		}
	}

	// Account for first line boost
	if (vPos == 0) 
	{
		current_bpg_target = dsc_cfg->first_line_bpg_ofs;
		increment = -(dsc_cfg->first_line_bpg_ofs << OFFSET_FRACTIONAL_BITS);
	} else {
		current_bpg_target = -(dsc_cfg->nfl_bpg_offset >> OFFSET_FRACTIONAL_BITS);
		increment = dsc_cfg->nfl_bpg_offset;
	}

	// Account for initial delay boost
	if (dsc_state->pixelCount < dsc_cfg->initial_xmit_delay)
	{
		int num_pixels;

		if(dsc_state->pixelCount == 0)
			num_pixels = PIXELS_PER_GROUP;
		else
			num_pixels = dsc_state->pixelCount - dsc_state->prevPixelCount;
		num_pixels = MIN(dsc_cfg->initial_xmit_delay - dsc_state->pixelCount, num_pixels);
		increment -= (dsc_cfg->bits_per_pixel * num_pixels) << (OFFSET_FRACTIONAL_BITS - 4);
	} 
	else
	{
		if (dsc_cfg->scale_increment_interval && !dsc_state->scaleIncrementStart && (vPos > 0) && (dsc_state->rcXformOffset > 0))
		{
			dsc_state->currentScale = 9;
			dsc_state->scaleAdjustCounter = 0;
			dsc_state->scaleIncrementStart = 1;
		}
	}
		
	dsc_state->prevPixelCount = dsc_state->pixelCount;

	current_bpg_target -= (dsc_cfg->slice_bpg_offset>>OFFSET_FRACTIONAL_BITS); 
	increment += dsc_cfg->slice_bpg_offset;

	dsc_state->throttleFrac += increment;
	dsc_state->rcXformOffset += dsc_state->throttleFrac >> OFFSET_FRACTIONAL_BITS;
	dsc_state->throttleFrac = dsc_state->throttleFrac & ((1<<OFFSET_FRACTIONAL_BITS)-1);
	
	if(dsc_state->rcXformOffset < dsc_cfg->final_offset)
		dsc_state->rcOffsetClampEnable = 1;

	if(dsc_state->rcOffsetClampEnable)
		dsc_state->rcXformOffset = MIN(dsc_state->rcXformOffset, dsc_cfg->final_offset);

	*scale = dsc_state->currentScale;
	*bpg_offset = current_bpg_target;
}


//! Convert original pixels in pic_t format to an array of unsigned int for easy access
/*! \param dsc_cfg   DSC configuration structure
    \param dsc_state DSC state structure 
	\param ip        Input picture
	\param vPos      Which line of slice to use */
void PopulateOrigLine(dsc_cfg_t *dsc_cfg, dsc_state_t *dsc_state, pic_t *ip, int vPos)
{
	int i, cpnt, w, pic_w;

	for (cpnt = 0; cpnt < NUM_COMPONENTS; ++cpnt)
	{
		w = dsc_cfg->slice_width;
		pic_w = ip->w;
		for (i=0; i<w+PADDING_RIGHT; ++i)
		{
			// Padding for lines that fall off the bottom of the raster uses midpoint value
			if(dsc_cfg->ystart+vPos >= ip->h)
				dsc_state->origLine[cpnt][i+PADDING_LEFT] = 1<<(dsc_state->cpntBitDepth[cpnt]-1);
			else switch (cpnt)
			{
			case 0:
				dsc_state->origLine[cpnt][i+PADDING_LEFT] = ip->data.yuv.y[MIN(dsc_cfg->ystart+vPos, ip->h-1)][MIN(dsc_cfg->xstart+i, ip->w-1)];
				break;
			case 1:
				dsc_state->origLine[cpnt][i+PADDING_LEFT] = ip->data.yuv.u[MIN(dsc_cfg->ystart+vPos, ip->h-1)][MIN(dsc_cfg->xstart+i, pic_w-1)];
				break;
			case 2:
				dsc_state->origLine[cpnt][i+PADDING_LEFT] = ip->data.yuv.v[MIN(dsc_cfg->ystart+vPos, ip->h-1)][MIN(dsc_cfg->xstart+i, pic_w-1)];
				break;
			}
		}
	}
}


//! Main DSC encoding and decoding algorithm
/*! \param isEncoder Flag indicating whether to do an encode (1) or decode (0)
    \param dsc_cfg   DSC configuration structure
	\param ip        Input picutre
	\param op        Output picture (modified, only affects area of current slice)
	\param cmpr_buf  Compressed data buffer (modified for encoder)
	\param temp_pic  Array of two temporary pictures to use for RGB-YCoCg conversion and back 
	\param chunk_sizes Array to hold sizes in bytes for each slice multiplexed chunk (modified for encoder) */
int DSC_Algorithm(int isEncoder, dsc_cfg_t* dsc_cfg, pic_t* ip, pic_t* op, unsigned char* cmpr_buf, pic_t **temp_pic, int *chunk_sizes)
{
	dsc_state_t dsc_state_storage, *dsc_state;
	pic_t* pic, *orig_op, *opic;
	int i;
	int vPos;
	int hPos;
	int done;
	int hSkew = PADDING_LEFT;    // there are hSkew fake pixels to the left of first real pixel
	int sampModCnt;
	int cpnt;
	int pred_x;
	int actual_x = 0;
	int recon_x;
	int err_raw;
	int err_q;
	int *currLine[NUM_COMPONENTS];
	int *prevLine[NUM_COMPONENTS];
	int lbufWidth;
	int range[NUM_COMPONENTS];
	int maxval;
	int qp;
	int qlevel;
	int group_count = 0;
	int throttle_offset, bpg_offset;
	int scale;
	int new_quant;
	PRED_TYPE pred2use;

#ifdef PRINTDEBUG
	if(isEncoder)
		g_fp_dbg = fopen("log_encode.txt","wt");
	else
		g_fp_dbg = fopen("log_decode.txt","wt");
#endif

	dsc_state = InitializeDSCState( dsc_cfg, &dsc_state_storage );
	dsc_state->isEncoder = isEncoder;
	dsc_state->chunkSizes = chunk_sizes;

	orig_op = op;
	if ( dsc_cfg->convert_rgb ) {
		// convert rgb to ycocg
		pic = temp_pic[0];
		opic = temp_pic[1];
		op = opic;
		rgb2ycocg(ip, pic, dsc_cfg);
	} else {
		// no color conversion
		pic = ip;
	}

	// line buffers have padding to left and right
	lbufWidth = dsc_cfg->slice_width + PADDING_LEFT + PADDING_RIGHT; // pixels to left and right

	// initialize DSC variables
	for ( cpnt = 0; cpnt<NUM_COMPONENTS; cpnt++ )
	{
		int initValue;
		if ( dsc_cfg->convert_rgb ) 
		{
			initValue = 1 << (pic->bits - 1);
			if(cpnt != 0)
				initValue *= 2;
		}
		else
			initValue = 1 << (pic->bits-1);

		dsc_state->currLine[cpnt] = currLine[cpnt] = (int*) malloc(lbufWidth*sizeof(int));
		dsc_state->prevLine[cpnt] = prevLine[cpnt] = (int*) malloc(lbufWidth*sizeof(int));
		dsc_state->origLine[cpnt] = (int*) malloc(lbufWidth*sizeof(int));
		for ( i=0; i<lbufWidth; i++ ) {
			currLine[cpnt][i] = initValue;
			prevLine[cpnt][i] = initValue;
		}
	}

	//--------------------------------------------------------------------------
	// sample range handling
	//
	if ( pic->bits != dsc_cfg->bits_per_component )
	{
		printf("ERROR: Expect picture bit depth to match configuration\n");
		exit(1);
	}

	for ( i=0; i<NUM_COMPONENTS; i++ )
	{
		range[i] = 1<<dsc_cfg->bits_per_component;
		dsc_state->cpntBitDepth[i] = dsc_cfg->bits_per_component;
	}

	if (dsc_cfg->convert_rgb)
	{
		range[1] *= 2;
		range[2] *= 2;
		dsc_state->cpntBitDepth[1]++;
		dsc_state->cpntBitDepth[2]++;
	}

	dsc_state->groupCountLine = 0;
	// If decoder, read first group's worth of data
	if ( !isEncoder )
		VLDGroup( dsc_cfg, dsc_state, &cmpr_buf );

	vPos = 0;
	hPos = 0;
	sampModCnt = 0;
	done = 0;
	dsc_state->hPos = 0;
	dsc_state->vPos = 0;

	new_quant = 0;
	qp = 0;
	if (isEncoder)
		PopulateOrigLine(dsc_cfg, dsc_state, pic, 0);


	while ( !done ) {
		dsc_state->vPos = vPos;
		if ((hPos % PIXELS_PER_GROUP)==0)
		{
			// Note that UpdateICHistory works on the group to the left
			for (i=0; i<PIXELS_PER_GROUP; ++i)
				UpdateICHistory(dsc_cfg, dsc_state, currLine, sampModCnt, hPos+i, vPos);
		}

		for ( cpnt = 0; cpnt<NUM_COMPONENTS; cpnt++ ) {
			qlevel = MapQpToQlevel(dsc_state, qp, cpnt);

			if (vPos==0)
			{
				// Use left predictor.  Modified MAP doesn't make sense since there is no previous line.
				pred2use = PT_LEFT;
			}
			else
			{
				pred2use = dsc_state->prevLinePred[hPos/PRED_BLK_SIZE];
			}
			pred_x = SamplePredict( dsc_state, prevLine[cpnt], currLine[cpnt], hPos, 
									pred2use, qlevel, cpnt);

			// Compute residual and quantize:
			if ( isEncoder ) {
				//
				// find residual and quantize it
				//
				actual_x = dsc_state->origLine[cpnt][hPos+PADDING_LEFT];
				err_raw = actual_x - pred_x;

				if (sampModCnt==0)
					dsc_state->masterQp = qp;

				qlevel = MapQpToQlevel(dsc_state, qp, cpnt);

				err_q = QuantizeResidual( err_raw, qlevel);
					
				err_raw = actual_x - FindMidpoint(dsc_state, cpnt, qlevel);

				// Calculate midpoint prediction error:
				dsc_state->quantizedResidualMid[cpnt][sampModCnt] = QuantizeResidual(err_raw, qlevel);
					
				// Midpoint residuals need to be bounded to BPC-QP in size, this is for some corner cases:
				if (dsc_state->quantizedResidualMid[cpnt][sampModCnt] > 0)
					while (FindResidualSize(dsc_state->quantizedResidualMid[cpnt][sampModCnt]) > MaxResidualSize(dsc_state, cpnt, qp))
						dsc_state->quantizedResidualMid[cpnt][sampModCnt]--;
				else
					while (FindResidualSize(dsc_state->quantizedResidualMid[cpnt][sampModCnt]) > MaxResidualSize(dsc_state, cpnt, qp))
						dsc_state->quantizedResidualMid[cpnt][sampModCnt]++;
							
				// store to array
				dsc_state->quantizedResidual[cpnt][sampModCnt] = err_q;
			}
			else  // DECODER:
			{
				// Decoder takes error from bitstream
				err_q = dsc_state->quantizedResidual[cpnt][sampModCnt];

				qlevel = MapQpToQlevel(dsc_state, qp, cpnt);

				// Use midpoint prediction if selected
				if (dsc_state->useMidpoint[cpnt])
					pred_x = FindMidpoint(dsc_state, cpnt, qlevel);
			}

			//-----------------------------------------------------------------
			// reconstruct
			// *MODEL NOTE* MN_IQ_RECON
			maxval = range[cpnt] - 1;
			recon_x = CLAMP(pred_x + (err_q << qlevel), 0, maxval);
			if (isEncoder)
			{
				int absErr;

				absErr = abs(actual_x - recon_x) >> (dsc_cfg->bits_per_component - 8);
				if ((sampModCnt==0))
					dsc_state->maxError[cpnt] = absErr;
				else
					dsc_state->maxError[cpnt] = MAX(dsc_state->maxError[cpnt], absErr);
			}
			if (isEncoder)
			{
				int midpoint_pred, midpoint_recon, absErr;

				// Encoder always computes midpoint value in case any residual size >= BPC - QP
				midpoint_pred = FindMidpoint(dsc_state, cpnt, qlevel);
				midpoint_recon = midpoint_pred + (dsc_state->quantizedResidualMid[cpnt][sampModCnt] << qlevel);
				dsc_state->midpointRecon[cpnt][sampModCnt] = CLAMP(midpoint_recon, 0, maxval);
				absErr = abs(actual_x - dsc_state->midpointRecon[cpnt][sampModCnt]) >> (dsc_cfg->bits_per_component - 8);
				if ((sampModCnt==0))
					dsc_state->maxMidError[cpnt] = absErr;
				else
					dsc_state->maxMidError[cpnt] = MAX(dsc_state->maxMidError[cpnt], absErr);
			}
				
			if (!isEncoder && dsc_state->ichSelected) {  // IC$ selected on decoder - do an ICH look-up
				unsigned int p[NUM_COMPONENTS];
				HistoryLookup(dsc_cfg, dsc_state, dsc_state->ichLookup[sampModCnt], p, hPos, (vPos==0));
				recon_x = p[cpnt];
			}

			// Save reconstructed value in line store
			currLine[cpnt][hPos+hSkew] = recon_x;

			// Copy reconstructed samples to output picture structure
			if ((vPos+dsc_cfg->ystart < op->h) && (hPos+dsc_cfg->xstart < op->w)) {
				switch ( cpnt ) {
				case  0: op->data.yuv.y[vPos+dsc_cfg->ystart][hPos+dsc_cfg->xstart] = recon_x; break;
				case  1: op->data.yuv.u[vPos+dsc_cfg->ystart][hPos+dsc_cfg->xstart] = recon_x; break;
				case  2: op->data.yuv.v[vPos+dsc_cfg->ystart][hPos+dsc_cfg->xstart] = recon_x; break;
				}
			}
		}

		// Update QP per group
		if ((sampModCnt==PIXELS_PER_GROUP-1) || (hPos==dsc_cfg->slice_width-1))
		{
			if ( isEncoder ) {
				// *MODEL NOTE* MN_ENC_FLATNESS_DECISION
				if (IsFlatnessInfoSent(dsc_cfg, qp) && ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP) == 3))
				{
					if (dsc_state->firstFlat >= 0)
						dsc_state->prevIsFlat = 1;
					else
						dsc_state->prevIsFlat = 0;
					dsc_state->prevFirstFlat = -1;

					for (i=0; i<GROUPS_PER_SUPERGROUP; ++i)
					{
						int flatness_type;
						
						flatness_type = IsOrigFlatHIndex(dsc_cfg, dsc_state, hPos + (i+1)*PIXELS_PER_GROUP);
						if (!dsc_state->prevIsFlat && flatness_type)
						{
							dsc_state->prevFirstFlat = i;
							dsc_state->prevFlatnessType = flatness_type - 1;
							break;
						}
						dsc_state->prevIsFlat = flatness_type;
					}
				} else if (!IsFlatnessInfoSent(dsc_cfg, qp)
					&& ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP)==3))
				{
					dsc_state->prevFirstFlat = -1;
				}
				else if ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP)==0)
				{
					dsc_state->firstFlat = dsc_state->prevFirstFlat;
					dsc_state->flatnessType = dsc_state->prevFlatnessType;
				}
				dsc_state->origIsFlat = 0;
				if ((dsc_state->firstFlat >= 0) &&
				    ((dsc_state->groupCount % GROUPS_PER_SUPERGROUP) == dsc_state->firstFlat))
					dsc_state->origIsFlat = 1;
			}

			// *MODEL NOTE* MN_FLAT_QP_ADJ
			if (dsc_state->origIsFlat && (dsc_state->masterQp < dsc_cfg->rc_range_parameters[NUM_BUF_RANGES-1].range_max_qp))
			{
				if ((dsc_state->flatnessType==0) || (dsc_state->masterQp<SOMEWHAT_FLAT_QP_THRESH)) // Somewhat flat
				{
					dsc_state->stQp = MAX(dsc_state->stQp - 4, 0);
					qp = MAX(new_quant-4, 0);
				} else {		// very flat
					dsc_state->stQp = 1+(2*(dsc_cfg->bits_per_component-8));
					qp = 1+(2*(dsc_cfg->bits_per_component-8));
				}
			}
			else
				qp = new_quant;
		}
			
		if (!isEncoder)  // Update decoder QP
			dsc_state->masterQp = qp;
		else
		{
			if (sampModCnt == 0)
			{
				for (i=0; i<PIXELS_PER_GROUP; ++i)
					dsc_state->origWithinQerr[i] = 1;
			}
			if (IsOrigWithinQerr(dsc_cfg, dsc_state, hPos, vPos, dsc_state->masterQp, sampModCnt))
			{
				unsigned int orig[NUM_COMPONENTS];

				orig[0] = dsc_state->origLine[0][PADDING_LEFT+hPos];
				orig[1] = dsc_state->origLine[1][PADDING_LEFT+hPos];
				orig[2] = dsc_state->origLine[2][PADDING_LEFT+hPos];
				dsc_state->ichLookup[sampModCnt] = PickBestHistoryValue(dsc_cfg, dsc_state, hPos, orig);
				// Have to do ICH lookup here & remember pixel values
				HistoryLookup(dsc_cfg, dsc_state, dsc_state->ichLookup[sampModCnt], dsc_state->ichPixels[sampModCnt], hPos, (vPos==0));
				for (cpnt=0; cpnt < NUM_COMPONENTS; ++cpnt)
				{
					int absErr;

					absErr = abs((int)dsc_state->ichPixels[sampModCnt][cpnt] - (int)orig[cpnt]) >> (dsc_cfg->bits_per_component - 8);
					if (sampModCnt==0) dsc_state->maxIchError[cpnt] = 0;
					dsc_state->maxIchError[cpnt] = MAX(dsc_state->maxIchError[cpnt], absErr);
				}
			}
		}

		sampModCnt++;
		if ( (sampModCnt >= PIXELS_PER_GROUP ) || (hPos+1 == dsc_cfg->slice_width)) {
			// Pad partial group at the end of the line
			if (sampModCnt < PIXELS_PER_GROUP)
			{
				for (i = sampModCnt; i<PIXELS_PER_GROUP; ++i)
				{
					// Set ICH values to the rightmost value
					dsc_state->ichLookup[i] = dsc_state->ichLookup[sampModCnt-1];
					for (cpnt = 0; cpnt<NUM_COMPONENTS; cpnt++)
					{
						dsc_state->quantizedResidual[cpnt][i] = 0;
						dsc_state->quantizedResidualMid[cpnt][i] = 0;
					}
					hPos++;
				}
			}

			dsc_state->hPos = hPos;

			if ( isEncoder ) {
				// Code the group
				VLCGroup( dsc_cfg, dsc_state, &cmpr_buf);

				// If it turned out we needed midpoint prediction, change the reconstructed pixels to use midpoint results
				UpdateMidpoint(dsc_cfg, dsc_state, currLine);

				// If it turned out that IC$ was selected, change the reconstructed pixels to use IC$ values
				if (dsc_state->ichSelected)
					UseICHistory(dsc_cfg, dsc_state, currLine);

				for (cpnt=0; cpnt < NUM_COMPONENTS; ++cpnt)
					dsc_state->leftRecon[cpnt] = currLine[cpnt][MIN(dsc_cfg->slice_width-1, hPos)+PADDING_LEFT];
				
				// Calculate scale & offset for RC
				CalcFullnessOffset(dsc_cfg, dsc_state, vPos, group_count, &scale, &bpg_offset);
				group_count++;
				dsc_state->groupCount = group_count;
				throttle_offset = dsc_state->rcXformOffset;

				// Do rate control
				RateControl( dsc_cfg, dsc_state, throttle_offset, bpg_offset, group_count, scale, sampModCnt );  // Group is finished
				new_quant = dsc_state->stQp;

				if (dsc_state->bufferFullness < 0)
				{
					if (dsc_cfg->vbr_enable)
					{
						dsc_state->bitsClamped += -dsc_state->bufferFullness;
						dsc_state->bufferFullness = 0;
					}
					else
					{
						printf("The buffer model encountered an underflow.  This may have occurred due to\n");
						printf("an excessively high programmed constant bit rate\n");
						exit(1);
					}
				}
			}
			else 
			{  
				// Calculate scale & offset for RC
				CalcFullnessOffset(dsc_cfg, dsc_state, vPos, group_count, &scale, &bpg_offset);
				group_count++;
				dsc_state->groupCount = group_count;
				throttle_offset = dsc_state->rcXformOffset;

				// Do rate control
				RateControl( dsc_cfg, dsc_state, throttle_offset, bpg_offset, group_count, scale, sampModCnt );
				for (cpnt=0; cpnt < NUM_COMPONENTS; ++cpnt)
					dsc_state->leftRecon[cpnt] = currLine[cpnt][MIN(dsc_cfg->slice_width-1, hPos)+PADDING_LEFT];
				new_quant = dsc_state->stQp;

				if (dsc_state->bufferFullness < 0)
				{
					if (dsc_cfg->vbr_enable)
					{
						dsc_state->bitsClamped += -dsc_state->bufferFullness;
						dsc_state->bufferFullness = 0;
					}
					else
					{
						printf("The buffer model encountered an underflow.  This may have occurred due to\n");
						printf("an excessively high constant bit rate or due to an attempt to decode an\n");
						printf("invalid DSC stream.\n");
						exit(1);
					}
				}

				if (hPos>=dsc_cfg->slice_width-1)
					dsc_state->groupCountLine = 0;
				if ((hPos<dsc_cfg->slice_width-1) || (vPos<dsc_cfg->slice_height-1))  // Don't decode if we're done
					VLDGroup( dsc_cfg, dsc_state, &cmpr_buf );
			}

			sampModCnt = 0;
		}

#ifdef PRINTDEBUG
		fflush(g_fp_dbg);
#endif
		hPos++;
		if ( hPos >= dsc_cfg->slice_width ) {
			int mod_hPos;
			// end of line
			// Update block prediction based on real reconstructed values
			for (mod_hPos=0; mod_hPos<dsc_cfg->slice_width; ++mod_hPos)
			{
				if (PRINT_DEBUG_RECON)
					fprintf(g_fp_dbg, "\n%d, %d: ", mod_hPos, vPos);
				for (cpnt = 0; cpnt < NUM_COMPONENTS; ++cpnt)
				{
					if (PRINT_DEBUG_RECON)
						fprintf(g_fp_dbg, "%d ", currLine[cpnt][mod_hPos+PADDING_LEFT]);
					BlockPredSearch( dsc_cfg, dsc_state, cpnt, currLine, mod_hPos, currLine[cpnt][mod_hPos+PADDING_LEFT] );

					if ((vPos + dsc_cfg->ystart < op->h) && (mod_hPos + dsc_cfg->xstart < op->w))
					{
						switch(cpnt)
						{
						case 0: op->data.yuv.y[vPos+dsc_cfg->ystart][mod_hPos+dsc_cfg->xstart] = currLine[cpnt][PADDING_LEFT+mod_hPos]; break;
						case 1: op->data.yuv.u[vPos+dsc_cfg->ystart][mod_hPos+dsc_cfg->xstart] = currLine[cpnt][PADDING_LEFT+mod_hPos]; break;
						case 2: op->data.yuv.v[vPos+dsc_cfg->ystart][mod_hPos+dsc_cfg->xstart] = currLine[cpnt][PADDING_LEFT+mod_hPos]; break;
						}
					}
				}
#ifdef PRINTDEBUG
				fflush(g_fp_dbg);
#endif
			}

			// reduce number of bits per sample in line buffer (replicate pixels in left/right padding)
			for ( i=0; i<lbufWidth; i++ )
				for ( cpnt = 0; cpnt<NUM_COMPONENTS; cpnt++ )
					prevLine[cpnt][i] = SampToLineBuf( dsc_cfg, dsc_state, currLine[cpnt][CLAMP(i,PADDING_LEFT,PADDING_LEFT+dsc_cfg->slice_width-1)], cpnt );

			hPos = 0;
			vPos++;
			if (dsc_state->isEncoder)
				dsc_state->groupCountLine = 0;

			if ( vPos >= dsc_cfg->slice_height )
				done = 1;
			else if (isEncoder)
				PopulateOrigLine(dsc_cfg, dsc_state, pic, vPos);
		}
		//if(hIndex==785 && vIndex == 0 && dsc_cfg->ystart == 0)
		//	printf("Debug\n");

	}

	if ( sampModCnt != 0 && isEncoder ) {  // Pad last unit wih 0's if needed
		for (i = sampModCnt; i<PIXELS_PER_GROUP; ++i)
			for (cpnt = 0; cpnt<NUM_COMPONENTS; cpnt++)
				dsc_state->quantizedResidual[cpnt][i] = 0;
		VLCGroup( dsc_cfg, dsc_state, &cmpr_buf );
	}

	if (dsc_state->isEncoder && dsc_cfg->muxing_mode)
	{
		while (dsc_state->seSizeFifo[0].fullness > 0)
			ProcessGroupEnc(dsc_cfg, dsc_state, cmpr_buf);
	}

	if (dsc_state->isEncoder && dsc_cfg->vbr_enable)
	{
		int accum_bytes = 0;

		if(dsc_state->chunkCount != dsc_cfg->slice_height-1)
		{
			while(dsc_state->chunkCount < dsc_cfg->slice_height-1)
				RemoveBitsEncoderBuffer(dsc_cfg, dsc_state);
			if(dsc_state->chunkCount > dsc_cfg->slice_height - 1)
			{
				printf("Chunk count was greater than the number of slice lines. This is an unexpected\n");
				printf("fatal error.\n");
				exit(1);
			}
		}

		for(i=0; i<dsc_cfg->slice_height-1; ++i)
			accum_bytes += dsc_state->chunkSizes[i];

		dsc_state->chunkSizes[dsc_state->chunkCount] = MAX(0, dsc_state->postMuxNumBits / 8 - accum_bytes);
	}

	if ( dsc_cfg->convert_rgb ) {
		// Convert YCoCg back to RGB again
		ycocg2rgb(op, orig_op, dsc_cfg);
	}

	for ( cpnt = 0; cpnt<NUM_COMPONENTS; cpnt++ )
	{
		free(currLine[cpnt]);
		free(prevLine[cpnt]);
		free(dsc_state->origLine[cpnt]);
		fifo_free(&(dsc_state->shifter[cpnt]));
		fifo_free(&(dsc_state->encBalanceFifo[cpnt]));
		fifo_free(&(dsc_state->seSizeFifo[cpnt]));
		free(dsc_state->history.pixels[cpnt]);
	}

	free(dsc_state->prevLinePred);
	free(dsc_state->history.valid);

	if (isEncoder && (dsc_state->bufferFullness > ((dsc_cfg->initial_xmit_delay * dsc_cfg->bits_per_pixel) >> 4)))
	{
		printf("Too many bits are left in the rate buffer at the end of the slice.  This is most likely\n");
		printf("due to an invalid RC configuration.\n");
		exit(1);
	}
#ifdef PRINTDEBUG
	fclose(g_fp_dbg);
#endif
	return dsc_state->postMuxNumBits;

}


//! Wrapper function for encode
/*! \param dsc_cfg   DSC configuration structure
    \param p_in      Input picture
	\param p_out     Output picture
	\param cmpr_buf  Pointer to empty buffer to hold compressed bitstream
	\param temp_pic  Array of two pictures to use as temporary storage for YCoCg conversions
	\return          Number of bits in the resulting compressed bitstream */
int DSC_Encode(dsc_cfg_t *dsc_cfg, pic_t *p_in, pic_t *p_out, unsigned char *cmpr_buf, pic_t **temp_pic, int *chunk_sizes)
{
	return DSC_Algorithm(1, dsc_cfg, p_in, p_out, cmpr_buf, temp_pic, chunk_sizes);
}


//! Wrapper function for decode
/*! \param dsc_cfg   DSC configuration structure
	\param p_out     Output picture
	\param cmpr_buf  Pointer to buffer containing compressed bitstream
	\param temp_pic  Array of two pictures to use as temporary storage for YCoCg conversions */
void DSC_Decode(dsc_cfg_t *dsc_cfg, pic_t *p_out, unsigned char *cmpr_buf, pic_t **temp_pic)
{
	DSC_Algorithm(0, dsc_cfg, p_out, p_out, cmpr_buf, temp_pic, NULL);
}

