/***************************************************************************
*    Cofpyright (c) 2013, Broadcom Corporation
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

/*! \file fifo.c
 *    Generic bit FIFO functions */

#include <stdio.h>
#include <stdlib.h>
#include "fifo.h"

//! Initialize a FIFO object
/*! \param fifo		 Pointer to FIFO data structure
    \param size		 Specifies FIFO size in bytes */
void fifo_init(fifo_t *fifo, int size)
{
	fifo->data = (unsigned char *)malloc(sizeof(unsigned char) * size);
	fifo->size = size*8;
	fifo->fullness = 0;
	fifo->read_ptr = fifo->write_ptr = 0;
	fifo->max_fullness = 0;
	fifo->bits_added = 0;
}


//! Free a FIFO object
/*! \param fifo		Pointer to FIFO data structure */
void fifo_free(fifo_t *fifo)
{
	free(fifo->data);
}


//! Get bits from a FIFO
/*! \param fifo		Pointer to FIFO data structure 
    \param n		Number of bits to retrieve
	\param sign_extend Flag indicating to extend sign bit for return value
	\return			Value from FIFO */
int fifo_get_bits(fifo_t *fifo, int n, int sign_extend)
{
	unsigned int d = 0;
	int i, pad = 0;
	unsigned char b;
	int sign = 0;

	if (fifo->fullness < n)
	{
		printf("FIFO underflow!\n");
		exit(1);
	}
	if (fifo->fullness < n)
	{
		pad = n - fifo->fullness;
		n = fifo->fullness;
	}

	for (i=0; i<n; ++i)
	{
		b = (fifo->data[fifo->read_ptr/8] >> (7-(fifo->read_ptr%8))) & 1;
		if (i==0) sign=b;
		d = (d<<1) + b;
		fifo->fullness --;
		fifo->read_ptr ++;
		if (fifo->read_ptr >= fifo->size)
			fifo->read_ptr = 0;
	}

	if (sign_extend && sign)
	{
		int mask;
		mask = (1<<n)-1;
		d |= (~mask);
	}
	return (d << pad);
}


//! Put bits into a FIFO
/*! \param fifo		Pointer to FIFO data structure
	\param d		Value to add to FIFO
    \param n		Number of bits to add to FIFO */
void fifo_put_bits(fifo_t *fifo, unsigned int d, int nbits)
{
	int i;
	unsigned char b;

	if (fifo->fullness + nbits > fifo->size)
	{
		printf("FIFO overflow!\n");
		exit(1);
	}

	fifo->bits_added += nbits;
	fifo->fullness += nbits;
	for (i=0; i<nbits; ++i)
	{
		b = (d >> (nbits-i-1)) & 1;
		if (!b)
			fifo->data[fifo->write_ptr/8] &= ~(1<<(7-(fifo->write_ptr%8)));
		else
			fifo->data[fifo->write_ptr/8] |= (1<<(7-(fifo->write_ptr%8)));
		fifo->write_ptr++;
		if (fifo->write_ptr >= fifo->size)
			fifo->write_ptr = 0;
	}
	if (fifo->fullness > fifo->max_fullness)
		fifo->max_fullness = fifo->fullness;
}

