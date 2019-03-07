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

/*! \file utl.c
 *    Utility functions */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "vdo.h"
#include "utl.h"
#include "logging.h"


//! Allocate the memory for pixel data
/*! \param w       Picture width
    \param h       Picture height
    \return        Pointer to 2D pixel buffer */
void *palloc(int w, int h)
{
	int **p;
	int   i;

	p = (int **) calloc(h, sizeof(short *));
	if (p == NULL)
	{
		fprintf(stderr, "ERROR: Failed to allocate memory.\n");
		exit(1);
	}
	else
		for (i = 0; i < h; i++)
		{
			p[i] = calloc(w, sizeof(int));
			if (p[i] == NULL)
			{
				fprintf(stderr, "ERROR: Failed to allocate memory.\n");
				exit(1);
			}
		}
		return p;
}


//! Create a picture object
/*! \param format  indicates field or frame
    \param color   color space
	\param chroma  Chroma subsampling
	\param w       Picture width
	\param h       Picture height
    \return        Pointer to picture (pic_t) object */
pic_t *pcreate(int format, int color, int chroma, int w, int h)
{
	pic_t *p;    

	p = malloc(sizeof(pic_t));

	if (p == NULL)
	{
		fprintf(stderr, "ERROR: Failed to allocate memory.\n");
		exit(1);
	}

	p->format = format;
	p->color  = color;
	p->chroma = chroma;

	p->w = w;
	p->h = h;

	if (color == RGB)
	{
		p->data.rgb.r = (int **) palloc(w, h);
		p->data.rgb.g = (int **) palloc(w, h);
		p->data.rgb.b = (int **) palloc(w, h);
		//p->data.rgb.a = (int **) palloc(w, h);
	}
	else if (chroma == YUV_420)
	{
		p->data.yuv.y = (int **) palloc(w, h);
		p->data.yuv.u = (int **) palloc(w / 2, h / 2);
		p->data.yuv.v = (int **) palloc(w / 2, h / 2);
		//p->data.yuv.a = (int **) palloc(w, h);
	}
	else if (chroma == YUV_422)
	{

		p->data.yuv.y = (int **) palloc(w, h);
		p->data.yuv.u = (int **) palloc(w / 2, h);
		p->data.yuv.v = (int **) palloc(w / 2, h);
		//p->data.yuv.a = (int **) palloc(w, h);
	}
	else
	{
		p->data.yuv.y = (int **) palloc(w, h);
		p->data.yuv.u = (int **) palloc(w, h);
		p->data.yuv.v = (int **) palloc(w, h);
		//p->data.yuv.a = (int **) palloc(w, h);
	}

	// Set defaults for deterministic DPX output
	p->alpha = 0;
	p->ar1 = 16;
	p->ar2 = 9;
	p->framerate = 60.0;
	p->frm_no = 0;
	p->interlaced = 0;
	p->seq_len = 1;

	return p;
}


//! Destroy a picture object
/*! \param p        Pointer to picture (pic_t) object
	\return         NULL pointer */
void *pdestroy(pic_t *p)
{
	int i;

	if (p->color == RGB)
	{
		for (i = 0; i < p->h; i++)
		{
			free(p->data.rgb.r[i]);
			free(p->data.rgb.g[i]);
			free(p->data.rgb.b[i]);
			//free(p->data.rgb.a[i]);
		}
		free(p->data.rgb.r);
		free(p->data.rgb.g);
		free(p->data.rgb.b);
		//free(p->data.rgb.a);
	}
	else if (p->chroma == YUV_420)
	{
		for (i = 0; i < p->h; i++)
		{
			free(p->data.yuv.y[i]);
			//free(p->data.yuv.a[i]);
		}

		for (i = 0; i < p->h / 2; i++)
		{
			free(p->data.yuv.u[i]);
			free(p->data.yuv.v[i]);
		}
		free(p->data.yuv.y);
		free(p->data.yuv.u);
		free(p->data.yuv.v);
		//free(p->data.yuv.a);
	}
	else
	{
		for (i = 0; i < p->h; i++)
		{
			free(p->data.yuv.y[i]);
			free(p->data.yuv.u[i]);
			free(p->data.yuv.v[i]);
			//free(p->data.yuv.a[i]);
		}
		free(p->data.yuv.y);
		free(p->data.yuv.u);
		free(p->data.yuv.v);
		//free(p->data.yuv.a);
	}
	free(p);
	return(NULL);
}


//! Convert RGB to YCbCr (unsupported)
/*! \param ip      Input picture (pic_t)
    \param op      Output picture (pic_t) */
void rgb2yuv(pic_t *ip, pic_t *op)
{
	fprintf(stderr, "ERROR: RGB to YUV conversion not supported\n");
	exit(1);
}


//! Convert YCbCr to RGB (unsupported)
/*! \param ip      Input picture (pic_t)
    \param op      Output picture (pic_t) */
void yuv2rgb(pic_t *ip, pic_t *op)
{
	fprintf(stderr, "ERROR: YUV to RGB conversion not supported\n");
	exit(1);
}


//! Convert YCbCr 4:2:2 to YCbCr 4:4:4 (unsupported)
/*! \param ip      Input picture (pic_t)
    \param op      Output picture (pic_t) */
void yuv_422_444(pic_t *ip, pic_t *op)
{
	fprintf(stderr, "ERROR: 422 to 444 conversion not supported.\n");
	exit(1);
}


//! Convert YCbCr 4:4:4 to YCbCr 4:2:2 (unsupported)
/*! \param ip      Input picture (pic_t)
    \param op      Output picture (pic_t) */
void yuv_444_422(pic_t *ip, pic_t *op)
{
	fprintf(stderr, "ERROR: 444 to 422 conversion not supported.\n");
	exit(1);
}


//! Change the bit depth of a picture
/*! \param p       Picture to modify
    \param newbits New bit depth */
void convertbits(pic_t *p, int newbits)
{
	int ii, jj;
	int error = 0;
	int rs = 0, ls = 0;

	if(p->bits > newbits)
		rs = p->bits - newbits;
	else
		ls = newbits - p->bits;

	if (p->color == RGB) {           
		if (p->chroma == YUV_444) {
			// RGB 4:4:4
			for (ii=0; ii<p->h; ++ii) {
				for (jj=0; jj<p->w; ++jj) {
					p->data.rgb.r[ii][jj] = (p->data.rgb.r[ii][jj]<<ls)>>rs;
					p->data.rgb.g[ii][jj] = (p->data.rgb.g[ii][jj]<<ls)>>rs;
					p->data.rgb.b[ii][jj] = (p->data.rgb.b[ii][jj]<<ls)>>rs;
					//p->data.rgb.a[ii][jj] = (p->data.rgb.a[ii][jj]<<ls)>>rs;
				}
			}
		} else {
			// we don't handle any RGB that's not 4:4:4 at this point
			printf(" RGB 422 not supported yet.\n");
			error = 1;
		}
	} else {
		// YUV format (either SD or HD)
		if (p->chroma == YUV_444) {
			// YUV 4:4:4
			for (ii=0; ii<p->h; ++ii) {
				for (jj=0; jj<p->w; ++jj) {
					p->data.yuv.y[ii][jj] = (p->data.yuv.y[ii][jj]<<ls)>>rs;
					p->data.yuv.u[ii][jj] = (p->data.yuv.u[ii][jj]<<ls)>>rs;
					p->data.yuv.v[ii][jj] = (p->data.yuv.v[ii][jj]<<ls)>>rs;
				}
			}
		} else {
			if (p->chroma == YUV_422) {
				for (ii=0; ii<p->h; ++ii) {
					for (jj=0; jj<(p->w/2); ++jj) {
						p->data.yuv.u[ii][jj] = (p->data.yuv.u[ii][jj]<<ls)>>rs;
						p->data.yuv.y[ii][jj*2] = (p->data.yuv.y[ii][jj*2]<<ls)>>rs;
						p->data.yuv.v[ii][jj] = (p->data.yuv.v[ii][jj]<<ls)>>rs;
						p->data.yuv.y[ii][jj*2+1] = (p->data.yuv.y[ii][jj*2+1]<<ls)>>rs;
					}
				}
			} else {
				error = 1;
			}
		}
	}

	if (error) {
		fprintf(stderr, "ERROR: Calling convert8to10() with incompatible format.  Only handle RGB444 or YUV422 or YUV444.\n");
		exit(1);
	}
	p->bits = newbits;
}


//! Read PPM (portable pix map) file
/*! \param fp      Pointer to open file handle
    \return        Picture loaded from file */
pic_t *readppm(FILE *fp)
{
	pic_t *p;

	char magicnum[128];
	char line[1000];

	int ready;
	int w, h;
	int i, j;
	int r, g, b;
	int maxval;

	fgets(line, 1000, fp);
	sscanf(line, "%s", magicnum);

	if (magicnum[0] != 'P')
	{
		fprintf(stderr, "Incorrect file type.");
		exit(1);
	}

	ready = 0;

	while (ready != 2)
	{
		if (fgets(line, 1000, fp) == NULL)
		{
			fprintf(stderr, "End of file.");
			exit(1);
		}
		if (line[0] != '#')
		{
			if (ready == 0)
			{
				sscanf(line, "%d %d", &w, &h);
				ready = 1;
			} else if (ready == 1)
			{
				sscanf(line, "%d", &maxval); // max component value
				ready = 2;
			}
		}
	}

	p = pcreate(FRAME, RGB, YUV_444, w, h);
	if (maxval <= 255)
		p->bits = 8;
	else if (maxval <= 1023)
		p->bits = 10;
	else if (maxval <= 4095)
		p->bits = 12;
	else if (maxval <= 65535)
		p->bits = 16;
	else
	{
		printf("PPM read error, maxval = %d\n", maxval);
		pdestroy(p);
		return(NULL);
	}

	if (magicnum[1] == '2')
		for (i = 0; i < h; i++)
			for (j = 0; j < w; j++)
			{
				fscanf(fp, "%d", &g);  // Gray value in PGM
				p->data.rgb.r[i][j] = g;
				p->data.rgb.g[i][j] = g;
				p->data.rgb.b[i][j] = g;
			}
	else if (magicnum[1] == '3')
		for (i = 0; i < h; i++)
			for (j = 0; j < w; j++)
			{
				fscanf(fp, "%d %d %d", &r, &g, &b);
				p->data.rgb.r[i][j] = r;
				p->data.rgb.g[i][j] = g;
				p->data.rgb.b[i][j] = b;
			}
	else if (magicnum[1] == '5') // PGM binary
		for (i = 0; i < h; i++)
			for (j = 0; j < w; j++)
			{
				g = (unsigned char)fgetc(fp);  // Gray value
				if(maxval > 255)
					g = (g<<8) + (unsigned char)fgetc(fp);
				p->data.rgb.r[i][j] = g;
				p->data.rgb.g[i][j] = g;
				p->data.rgb.b[i][j] = g;
			}
	else // P6
	{
		for (i = 0; i < h; i++)
		{
			for (j = 0; j < w; j++)
			{
				p->data.rgb.r[i][j] = (unsigned char) fgetc(fp);
				if (maxval > 255)
					p->data.rgb.r[i][j] = (p->data.rgb.r[i][j] << 8) + (unsigned char)fgetc(fp);
				p->data.rgb.g[i][j] = (unsigned char) fgetc(fp);
				if (maxval > 255)
					p->data.rgb.g[i][j] = (p->data.rgb.g[i][j] << 8) + (unsigned char)fgetc(fp);
				p->data.rgb.b[i][j] = (unsigned char) fgetc(fp);
				if (maxval > 255)
					p->data.rgb.b[i][j] = (p->data.rgb.b[i][j] << 8) + (unsigned char)fgetc(fp);
			}
		}
	}

	return p;
}


//! Read PPM (portable pix map) file
/*! \param fname   Picture file name
    \param pic     Pointer to picture (pic_t) (returned)
    \return        0 = success (read errors exit the program) */
int ppm_read(char *fname, pic_t **pic)
{
	FILE *fp;

	fp = fopen(fname, "rb");

	if (fp == NULL)
		return(-1);  // Error condition

	*pic = readppm(fp);

	fclose(fp);
	return (0);
}


//! Write PPM (portable pix map) file
/*! \param fp      Pointer to open file handle
    \param p       Picture to write */
void writeppm(FILE *fp, pic_t *p)
{
	int i, j;

	fprintf(fp, "P6\n");
	fprintf(fp, "%d %d\n", p->w, p->h);
	if (p->bits == 8)
		fprintf(fp, "255\n");
	else if (p->bits == 10)
		fprintf(fp, "1023\n");
	else if (p->bits == 12)
		fprintf(fp, "2047\n");
	else 
	{
		printf("Unsupported bit depth for PPM output: %d bits\n", p->bits);
		exit(1);
	}

	for (i = 0; i < p->h; i++)
		for (j = 0; j < p->w; j++)
		{
			if (p->bits>8)
				fputc((unsigned char)(p->data.rgb.r[i][j] >> 8), fp);
			fputc((unsigned char) p->data.rgb.r[i][j] & 0xff, fp);
			if (p->bits>8)
				fputc((unsigned char)(p->data.rgb.g[i][j] >> 8), fp);
			fputc((unsigned char) p->data.rgb.g[i][j] & 0xff, fp);
			if (p->bits>8)
				fputc((unsigned char)(p->data.rgb.b[i][j] >> 8), fp);
			fputc((unsigned char) p->data.rgb.b[i][j] & 0xff, fp);
		}
}


//! Write PPM (portable pix map) file
/*! \param fname   Picture file name
    \param pic     Pointer to picture (pic_t)
    \return        0 = success, -1 = failed to open output file */
int ppm_write(char *fname, pic_t *pic)
{
	FILE *fp;

	fp = fopen(fname, "wb");

	if (fp == NULL)
		return(-1);  // Error condition

	writeppm(fp, pic);

	fclose(fp);
	return (0);
}
