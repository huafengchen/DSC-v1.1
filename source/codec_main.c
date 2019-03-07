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

/*! \file codec_main.c
 *    Main codec loop
 *  \author Frederick Walls (fwalls@broadcom.com) */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vdo.h"
#include "dsc_types.h"
#include "dsc_utils.h"
#include "dpx.h"
#include "utl.h"
#include "psnr.h"
#include "cmd_parse.h"
#include "dsc_codec.h"
#include "logging.h"

#define PATH_MAX 1024
#define MAX_OPTNAME_LEN 200
#define CFGLINE_LEN ((PATH_MAX) + MAX_OPTNAME_LEN)
#define RANGE_CHECK(s,a,b,c) { if(((a)<(b))||((a)>(c))) { UErr("%s out of range, needs to be between %d and %d\n",s,b,c); } }

static int assign_line (char* line, cmdarg_t *cmdargs);


static int      help = 0;
static char fn_i[PATH_MAX+1] = "";
static char fn_o[PATH_MAX+1] = "";
static char fn_log[PATH_MAX+1] = "";
static char filepath[PATH_MAX+1] = "";
static char option[PATH_MAX+1]   = "";
static int rcModelSize;
static float bitsPerPixel;
static int bitsPerComponent;
static int enable422;
static int lineBufferBpc;
static int bpEnable;
static int initialDelay;
static int sliceWidth;
static int sliceHeight;
static int firstLineBpgOfs;
static int initialFullnessOfs;
static int useYuvInput;
static int function;
static int *rcOffset;
static int *rcMinQp;
static int *rcMaxQp;
static int *rcBufThresh;
static int tgtOffsetHi;
static int tgtOffsetLo;
static int rcEdgeFactor;
static int quantIncrLimit0;
static int quantIncrLimit1;
static int rbSwap;
static int rbSwapOut;
static int dpxBugsOverride;
static int dpxPadLineEnds;
static int dpxWriteBSwap;
static int flatnessMinQp;
static int flatnessMaxQp;
static int flatnessDetThresh;
static int enableVbr;
static int muxingMode;
static int muxWordSize;
static  cmdarg_t cmd_args[] = {

	// The array arguments have to be first:
	{ IVARG, NULL,    "RC_OFFSET",            "-rcofs",  0,  15},  // RC offset values
	{ IVARG, NULL,     "RC_MINQP",             "-rcmqp",  0,  15},  // Min QP values
	{ IVARG, NULL,     "RC_MAXQP",             "-rcmxqp", 0,  15},  // Max QP values
	{ IVARG,  NULL, "RC_BUF_THRESH",       "-rcbt",   0,   14},  // RC buffer threshold

	{ PARG,  &rcModelSize,        "RC_MODEL_SIZE",        "-rms",   0,  0},  // RC model size
	{ FARG,  &bitsPerPixel,		  "BITS_PER_PIXEL",		  "-bpp",   0,  0},  // bits per pixel
	{ PARG,  &bitsPerComponent,   "BITS_PER_COMPONENT",   "-bpc",   0,  0},  // bits per component
	{ PARG,  &enable422,          "ENABLE_422",           "-e422",  0,  0},  // enable_422
	{ PARG,  &bpEnable,           "BLOCK_PRED_ENABLE",    "-bpe",   0,  0},  // Block prediction range
	{ PARG,  &lineBufferBpc,      "LINE_BUFFER_BPC",      "-lbpc",  0,  0},  // Line buffer storage bits/component
	{ PARG,  &sliceWidth,         "SLICE_WIDTH",         "-nsh",   0,  0},  // Slice width (0=pic width)
	{ PARG,  &sliceHeight,        "SLICE_HEIGHT",         "-nsv",   0,  0},  // slice height (0=pic height)
	{ PARG,  &firstLineBpgOfs,    "FIRST_LINE_BPG_OFFSET", "-flbo", 0,  0},  // Additional bpp budget for 1st line
	{ PARG,  &initialFullnessOfs, "INITIAL_FULLNESS_OFFSET", "-ifo", 0, 0},  // Initial fullness offset
	{ PARG,  &initialDelay,       "INITIAL_DELAY",        "-id",    0,  0},  // Initial delay (in pixel time units) from encode start to xmit start
	{ PARG,  &useYuvInput,        "USE_YUV_INPUT",        "-uyi",   0,  0},   // Use YUV input (convert if necessary)
	{ PARG,  &rbSwap,             "SWAP_R_AND_B",         "-rbswp", 0,  0},  // Swap red & blue components
	{ PARG,  &rbSwapOut,          "SWAP_R_AND_B_OUT",     "-rbswpo", 0,  0},  // Swap red & blue components
	{ PARG,  &function,           "FUNCTION",             "-do",    0,  0},   // 0=encode/decode, 1=encode, 2=decode
	{ IARG,  &dpxBugsOverride,    "DPX_BUGS_OVERRIDE",    "-dpxbugs", 0, 0},  // Sets the DPX bugs mode (else autodetect)
	{ PARG,  &dpxPadLineEnds,     "DPX_PAD_LINE_ENDS",    "-dpxpad",  0, 0},  // Pad line ends for DPX output
	{ PARG,  &dpxWriteBSwap,      "DPX_WRITE_BSWAP",      "-dpxwbs",  0, 0},  // Pad line ends for DPX output
	{ PARG,  &enableVbr,          "VBR_ENABLE",           "-vbr",  0,  0},    // 1=disable stuffing bits (on/off VBR)
	{ PARG,  &muxWordSize,        "MUX_WORD_SIZE",        "-mws",  0,  0},    // mux word size if SSM enabled

	{ PARG,  &tgtOffsetHi,		  "RC_TGT_OFFSET_HI",     "-thi",   0,  0},   // Target hi
	{ PARG,  &tgtOffsetLo,		  "RC_TGT_OFFSET_LO",     "-tlo",   0,  0},   // Target lo
	{ PARG,  &rcEdgeFactor,       "RC_EDGE_FACTOR",       "-ef",    0,  0},   // Edge factor
	{ PARG,  &quantIncrLimit0,    "RC_QUANT_INCR_LIMIT0", "-qli0",  0,  0},   // Quant limit incr 0
	{ PARG,  &quantIncrLimit1,    "RC_QUANT_INCR_LIMIT1", "-qli1",  0,  0},   // Quant limit incr 1
	{ PARG,  &flatnessMinQp,      "FLATNESS_MIN_QP",       "-fmin", 0,  0},   // Flatness min QP
	{ PARG,  &flatnessMaxQp,      "FLATNESS_MAX_QP",       "-fmax", 0,  0},   // Flatness max QP
	{ PARG,  &flatnessDetThresh,  "FLATNESS_DET_THRESH",  "-fdt",  0,  0},   // Flatness detect threshold
	{ PARG,  &muxingMode,         "MUXING_MODE",          "-mm",   0,  0},   // Multiplexing mode
     {NARG,  &help,               "",                     "-help"   , 0, 0}, // video format
     {SARG,   filepath,           "INCLUDE",              "-F"      , 0, 0}, // Cconfig file
     {SARG,   option,             "",                     "-O"      , 0, 0}, // key/value pair
     {SARG,   fn_i,               "SRC_LIST",              ""        , 0, 0}, // Input file name
     {SARG,   fn_o,               "OUT_DIR",              ""        , 0, 0}, // Output file name
     {SARG,   fn_log,             "LOG_FILENAME",         ""        , 0, 0}, // Log file name

     {PARG,   NULL,               "",                     "",       0, 0 }
};

/*!
 ************************************************************************
 * \brief
 *    set_defaults() - Set default configuration values
 ************************************************************************
 */
void set_defaults(void)
{
	// Default is for 8bpc/8bpp
	int default_rcofs[] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12 };
	int default_minqp[] = { 0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 13 };
	int default_maxqp[] = { 4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 11, 12, 13, 13, 15 };
	int default_threshold[] = { 896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064 };
	int i;

	rcModelSize = 8192;
	bitsPerPixel = 8.0;
	bitsPerComponent = 8;
	enable422 = 0;
	bpEnable = 1;
	initialFullnessOfs = 6144;
	initialDelay = 170;
	firstLineBpgOfs = 12;
	sliceWidth = 0;
	sliceHeight = 0;
	useYuvInput = 0;
	rbSwap = 0;
	rbSwapOut = 0;
	function = 0;
	lineBufferBpc = 9;
	dpxBugsOverride = -1;
	dpxPadLineEnds = 0;
	dpxWriteBSwap = 0;
	enableVbr = 0;
	muxingMode = 1;

	tgtOffsetHi = 3;
	tgtOffsetLo = 3;
	rcEdgeFactor = 6;
	quantIncrLimit0 = 11;
	quantIncrLimit1 = 11;
	flatnessMinQp = 3;
	flatnessMaxQp = 12;
	flatnessDetThresh = 2;
	muxWordSize = 0;
	for (i=0; i<15; ++i)
	{
		rcOffset[i] = default_rcofs[i];
		rcMinQp[i] = default_minqp[i];
		rcMaxQp[i] = default_maxqp[i];
		if (i<14)
			rcBufThresh[i] = default_threshold[i];
	}

	strcpy(fn_o, ".");
	strcpy(fn_log, "log.txt");
}

/*!
 ************************************************************************
 * \brief
 *    parse_cfgfile() - loop over a file's lines and process
 *
 * \param fn
 *    Config file filename
 * \param cmdargs
 *    Command argument structure
 *
 ************************************************************************
 */
static int parse_cfgfile (char* fn, cmdarg_t *cmdargs)
{
	FILE *fd;
    char line[CFGLINE_LEN+1] = "";

    if (NULL == fn)
    {
        Err("%s called with NULL file name", __FUNCTION__);
    }
    if (0 == fn[0])
    {
        Err("empty configuration file name");
    }
    fd = fopen(fn, "r");
    if (NULL == fd)
    {
        PErr("cannot open file '%s'", fn);
    }
    fn[0] = '\0';
    while (NULL != fgets(line, CFGLINE_LEN, fd)) // we re-use the storage
    {
        assign_line(line, cmdargs);    
    }
    if (!feof(fd))
    {
        PErr("while reading from file '%s'", fn);
    }
    fclose(fd);
    return 0;
}


/*!
 ************************************************************************
 * \brief
 *    assign_line() - process a single line
 *
 * \param line
 *    String containing line
 * \param cmdargs
 *    Command argument structure
 *
 ************************************************************************
 */
static int assign_line (char* line, cmdarg_t *cmdargs)
{
    if (!parse_line (line, cmdargs))
    {
        UErr("unknown configuration field '%s'", line);
    }
    if ('\0' != filepath[0])   // config file
    {
        parse_cfgfile(filepath, cmdargs);
    }
    return 0;
}

/*!
 ************************************************************************
 * \brief
 *    usage() - Print the usage message
 ************************************************************************
 */
void usage(void)
{
	printf("Usage: DSC <options>\n");
	printf(" Option list:\n");
	printf("  -help => print this message\n");
	printf("  -F <cfg_file> => Specify configuration file (required)\n");
	printf("  -O\"PARAMETER <value>\" => override config file parameter PARAMETER with new value\n");
	printf("  See README.txt for a list of parameters.\n");
	exit(1);
}

/*!
 ************************************************************************
 * \brief
 *    process_args() - set defaults and process command line
 *
 * \param argc
 *    Argument count (from main)
 * \param argv
 *    Arguments (from main)
 * \param cmddargs
 *    Command arguemnts structure
 *
 ************************************************************************
 */
static int process_args(int argc, char *argv[], cmdarg_t *cmdargs)
{
    enum arg_order_e
    {
        IN_ARG, OUT_ARG, DONE
    };
    int expected_arg = IN_ARG; // track if we've seen these args
    int incr;
	int i;
	char *arg1;

    if (1 == argc) // no arguments
    {
        usage();
    }

    /* process each argument */
    for (i=1; i<argc; i+=incr)
    {
        arg1 = (i<argc-1)? argv[i+1]: NULL;
        incr = parse_cmd(argv[i], arg1, cmdargs);

        if (incr)
        {
            if (help) // help
            {
                usage();
            }
            if (0 != filepath[0])   // config file
            {
                parse_cfgfile(filepath, cmdargs);
            }
            else if (0 != option[0]) // config line
            {
                assign_line(option, cmdargs);
                option[0] = '\0';
            }
            continue;
        }
        if (argv[i][0]=='-')
        {
            fprintf(stderr, "ERROR: Unknown option '%s'!\n", argv[i]);
            usage();
        }
        incr = 1;
        switch (expected_arg)
        {
        case IN_ARG:  // input file name */
            expected_arg = OUT_ARG;
            strcpy(fn_i, argv[i]);
            break;
        case OUT_ARG:  // output file name */
            expected_arg = DONE;
            strcpy(fn_o, argv[i]);
            break;
        default:  /* input file name */
            fprintf(stderr, "ERROR: Unexpected argument %s (in=%s, out=%s)", argv[i], fn_i, fn_o);
            usage();
        }
    }

    return 0;
}


/*!
 ************************************************************************
 * \brief
 *    split_base_and_ext() - separate base file name and extension
 *
 * \param infname
 *    Filename with (optional) path and extension
 * \param base_name
 *    Allocated string to copy base filename to
 * \param extension
 *    Returns pointer to extension (uses part of base_name storage)
 *
 ************************************************************************
 */
void split_base_and_ext(char *infname, char *base_name, char **extension)
{
	int zz;
	int dot=-1, slash=-1;

	// Find last . and / (or \) in filename
	for (zz = strlen(infname)-1; zz >= 0; --zz)
	{
		if ((slash<0) && ((infname[zz] == '\\') || (infname[zz] == '/')))
			slash = zz;
		if ((dot<0) && (infname[zz] == '.'))
			dot = zz;
	}
	if ((dot < slash) || (dot < 0))
	{
		printf("ERROR: picture format unrecognized\n");
		exit(1);
	}
	strcpy(base_name, &(infname[slash+1]));
	*extension = &(base_name[dot-slash]);
	base_name[dot-slash-1] = '\0';  // terminate base_name string
}


/*!
 ************************************************************************
 * \brief
 *    write_dsc_data() - Write a .DSC formatted file
 *
 * \param bit_buffer
 *    Pointer to bitstream buffer
 * \param nbytes
 *    Number of bytes to write
 * \param fp
 *    File handle
 * \param vbr_enable
 *    VBR enable flag
 *
 ************************************************************************
 */
void write_dsc_data(unsigned char **bit_buffer, int nbytes, FILE *fp, int vbr_enable, int slices_per_line, int slice_height, int **sizes)
{
	int i;
	int slice_x, slice_y;
	int *current_idx;

	current_idx = (int *)malloc(sizeof(int) * slices_per_line);
	for (i=0; i<slices_per_line; ++i)
		current_idx[i] = 0;

	for (slice_y = 0; slice_y < slice_height; ++slice_y)
	{
		for (slice_x = 0; slice_x < slices_per_line; ++slice_x)
		{
			if (vbr_enable)
			{
				nbytes = sizes[slice_x][slice_y];
				fputc((nbytes>>8) & 0xff, fp);
				fputc(nbytes & 0xff, fp);
			}
			for (i=0; i<nbytes; ++i)
				fputc(bit_buffer[slice_x][current_idx[slice_x] + i], fp);
			current_idx[slice_x] += nbytes;
		}
	}
	free(current_idx);
}


/*!
 ************************************************************************
 * \brief
 *    read_dsc_data() - Read a .DSC formatted file
 *
 * \param bit_buffer
 *    Pointer to bitstream buffer
 * \param nbytes
 *    Number of bytes to write (if CBR mode)
 * \param fp
 *    File handle
 * \param vbr_enable
 *    VBR enable flag
 * \return
 *    Number of bytes read
 *
 ************************************************************************
 */
int read_dsc_data(unsigned char **bit_buffer, int nbytes, FILE *fp, int vbr_enable, int slices_per_line, int slice_height)
{
	int i, slice_y, slice_x;
	int *current_idx, total_bytes = 0;

	current_idx = (int *)malloc(sizeof(int) * slices_per_line);
	for (i=0; i<slices_per_line; ++i)
		current_idx[i] = 0;

	for (slice_y = 0; slice_y < slice_height; ++slice_y)
	{
		for (slice_x = 0; slice_x < slices_per_line; ++slice_x)
		{
			if (vbr_enable)
			{
				nbytes = 0;
				for(i=0; i<2; ++i)
					nbytes = (nbytes<<8) | (fgetc(fp) & 0xff);
			}
			for(i=0; i<nbytes; ++i)
				bit_buffer[slice_x][i+current_idx[slice_x]] = fgetc(fp) & 0xff;
			current_idx[slice_x] += nbytes;
			total_bytes += nbytes;
		}
	}

	free(current_idx);
	return(total_bytes);
}



/*!
 ************************************************************************
 * \brief
 *    codec_main() - Codec mainline
 *
 * \param argc
 *    Argument count (from main)
 * \param argv
 *    Arguments (from main)
 ************************************************************************
 */
#ifdef WIN32
int codec_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
	pic_t *ip=NULL, *ip2, *ref_pic, *op_dsc;
	dsc_cfg_t dsc_codec;
	unsigned char **buf;
	char f[PATH_MAX], infname[PATH_MAX], bitsfname[PATH_MAX];
	char *extension;
	FILE *list_fp, *logfp;
	char base_name[PATH_MAX];
	int i, j;
	FILE *bits_fp = NULL;
	int fcnt;
	int bufsize;
	int xs, ys;
	int slicew, sliceh;
	int target_bpp_x16;
	pic_t **temp_pic = NULL;
	int numslices, slicecount;
	int final_scale, num_extra_mux_bits;
	int hrdDelay, groupsPerLine, rbsMin;
	int final_value;
	int slices_per_line;
	int groups_total;
	int useppm = 0;
	int **chunk_sizes;
	int sliceBits;
	int prev_min_qp, prev_max_qp, prev_thresh, prev_offset;
	unsigned char pps[PPS_SIZE];

	printf("Display Stream Compression (DSC) reference model version 1.31\n");
	printf("Copyright 2013-2014 Broadcom Corporation.  All rights reserved.\n\n");

	// Allocate variables
	cmd_args[0].var_ptr = rcOffset = (int *)malloc(sizeof(int)*15);
	cmd_args[1].var_ptr = rcMinQp = (int *)malloc(sizeof(int)*15);
	cmd_args[2].var_ptr = rcMaxQp = (int *)malloc(sizeof(int)*15);
	cmd_args[3].var_ptr = rcBufThresh = (int *)malloc(sizeof(int)*14);

	set_defaults();
	memset(&dsc_codec, 0, sizeof(dsc_cfg_t));

	/* process input arguments */
    process_args(argc, argv, cmd_args);

	if (NULL == (list_fp=fopen(fn_i, "rt")))
	{
		fprintf(stderr, "Cannot open list file %s for input\n", fn_i);
		exit(1);
	}

	if (NULL == (logfp=fopen(fn_log, "wt")))
	{
		fprintf(stderr, "Cannot open list file log.txt for output\n");
		exit(1);
	}

	if (enable422 && !useYuvInput)
	{
		fprintf(stderr, "4:2:2 not supported with RGB input\n");
		exit(1);
	}

	fcnt = 0;
	infname[0] = '\0';
	fgets(infname, 512, list_fp);

	while ((strlen(infname)>0) || !feof(list_fp))
	{
		memset(pps, 0, PPS_SIZE);
		ip = NULL;
		while ((strlen(infname)>0) && (infname[strlen(infname)-1] < '0'))
			infname[strlen(infname)-1] = '\0';

		if (strlen(infname)==0)  // Skip blank lines
		{
			infname[0] = '\0';
			fgets(infname, 512, list_fp);
			continue;
		}

		split_base_and_ext(infname, base_name, &extension);

		if (!strcmp(extension, "dpx") || !strcmp(extension, "DPX"))
		{
			useppm = 0;
			if (dpx_read(infname, &ip, dpxBugsOverride))
			{
				if (function == 2)
					printf("Could not read original image for decode, PSNR will not be computed\n");
				else
				{
					fprintf(stderr, "Error read DPX file %s\n", infname);
					exit(1);
				}
			}
		}
		else if (!strcmp(extension, "ppm") || !strcmp(extension, "PPM"))
		{
			if (useYuvInput)
			{
				printf("Error: PPM format is RGB only, USE_YUV_INPUT must be set to 0\n");
				exit(1);
			}
			useppm = 1;
			
			if (ppm_read(infname, &ip))
			{
				if (function == 2)
					printf("Could not read original image for decode, PSNR will not be computed\n");
				else
				{
					fprintf(stderr, "Error read PPM file %s\n", infname);
					exit(1);
				}
			}
		}
		else if (strcmp(extension, "dsc") && strcmp(extension, "DSC"))
		{
			fprintf(stderr, "Unrecognized file format .%s\n", extension);
			exit(1);
		}

		if (ip)
		{
			ip->alpha = 0;

			// R/B swap
			if (rbSwap && (ip->color == RGB))
			{
				for (i=0; i<ip->h; ++i)
					for (j=0; j<ip->w; ++j)
					{
						int tmp;
						tmp = ip->data.rgb.r[i][j];
						ip->data.rgb.r[i][j] = ip->data.rgb.b[i][j];
						ip->data.rgb.b[i][j] = tmp;
					}
			}

			if (bitsPerComponent > ip->bits)
				convertbits(ip, bitsPerComponent);
		}

		// 4:2:2 to 4:4:4 coversion
		if (ip && (ip->chroma == YUV_422) && !enable422)
		{
			ip2 = pcreate(FRAME, ip->color, YUV_444, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			yuv_422_444(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}

		// RGB to YUV conversion
		if (ip && (ip->color == RGB) && useYuvInput)
		{
			ip2 = pcreate(FRAME, YUV_HD, ip->chroma, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			rgb2yuv(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}
		else if (ip && ((ip->color == YUV_HD) || (ip->color == YUV_SD)) && !useYuvInput)      // YUV to RGB conversion
		{
			ip2 = pcreate(FRAME, RGB, YUV_444, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			yuv2rgb(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}

		if (ip && (ip->chroma == YUV_444) && enable422)
		{
			ip2 = pcreate(FRAME, ip->color, YUV_422, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			yuv_444_422(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}

		if (ip && (bitsPerComponent < ip->bits))
			convertbits(ip, bitsPerComponent);

		if (ip && enable422 && (ip->w%2))
		{
			ip->w--;
			printf("WARNING: 4:2:2 picture width is constrained to be a multiple of 2.\nThe image %s will be cropped to %d pixels wide.\n", base_name, ip->w);
		}

		if (ip && enable422 && (sliceWidth%2))
		{
			sliceWidth++;
			printf("WARNING: 4:2:2 slice width is constrained to be a multiple of 2.\nThe slice width will be adjusted to %d pixels wide.\n", sliceWidth);
		}

		ref_pic = ip;
		if (ip && enable422)
		{
			ip2 = pcreate(FRAME, ip->color, YUV_444, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			simple422to444(ip, ip2);
			ip = ip2;
		}

		// Constants:
		dsc_codec.muxing_mode = muxingMode;

		// Set up parameters based on configuration if encoding
		if ((function == 1) || (function == 0))
		{
			if (!ip)
			{
				fprintf(stderr, "Cannot use .dsc file as input\n");
				exit(1);
			}
			dsc_codec.pic_width = ip->w;
			RANGE_CHECK("pic_width", dsc_codec.pic_width, 0, 65535);
			dsc_codec.pic_height = ip->h;
			RANGE_CHECK("pic_height", dsc_codec.pic_height, 0, 65535);
			dsc_codec.enable_422 = enable422;
			RANGE_CHECK("enable_422", dsc_codec.enable_422, 0, 1);
			dsc_codec.linebuf_depth = lineBufferBpc;
			RANGE_CHECK("linebuf_depth", dsc_codec.linebuf_depth, 8, 13);
			dsc_codec.rcb_bits = 0;
			dsc_codec.bits_per_component = bitsPerComponent;
			if(dsc_codec.bits_per_component != 8 && dsc_codec.bits_per_component != 10 && dsc_codec.bits_per_component != 12)
				UErr("bits_per_component must be either 8, 10, or 12\n");
			if (muxWordSize==0)
				muxWordSize = (dsc_codec.bits_per_component==12) ? 64 : 48;
			dsc_codec.mux_word_size = muxWordSize;
			dsc_codec.convert_rgb = !useYuvInput;
			dsc_codec.rc_tgt_offset_hi = tgtOffsetHi;
			RANGE_CHECK("rc_tgt_offset_hi", dsc_codec.rc_tgt_offset_hi, 0, 15);
			dsc_codec.rc_tgt_offset_lo = tgtOffsetLo;
			RANGE_CHECK("rc_tgt_offset_lo", dsc_codec.rc_tgt_offset_lo, 0, 15);
			target_bpp_x16 = (int)(bitsPerPixel * 16 + 0.5);
			dsc_codec.bits_per_pixel = target_bpp_x16;
			if(enableVbr)
			{
				RANGE_CHECK("bits_per_pixel (*16)", target_bpp_x16, 96, 1023);
			} else {				
				RANGE_CHECK("bits_per_pixel (*16)", target_bpp_x16, 96, 640);  // Top is 40bpp, bit rate for force_mpp with 12bpc
			}
			dsc_codec.rc_edge_factor = rcEdgeFactor;
			RANGE_CHECK("rc_edge_factor", dsc_codec.rc_edge_factor, 0, 15);
			if (rcEdgeFactor < 2)
				printf("WARNING: The rate control will not work as designed with rc_edge_factor < 2.\n");
			dsc_codec.rc_quant_incr_limit1 = quantIncrLimit1;
			RANGE_CHECK("rc_quant_incr_limit1", dsc_codec.rc_quant_incr_limit1, 0, 31);
			dsc_codec.rc_quant_incr_limit0 = quantIncrLimit0;
			RANGE_CHECK("rc_quant_incr_limit0", dsc_codec.rc_quant_incr_limit0, 0, 31);
			prev_min_qp = rcMinQp[0];
			prev_max_qp = rcMaxQp[0];
			prev_thresh = rcBufThresh[0];
			prev_offset = rcOffset[0];
			for (i=0; i<NUM_BUF_RANGES; ++i)
			{
				dsc_codec.rc_range_parameters[i].range_bpg_offset = rcOffset[i];
				RANGE_CHECK("range_bpg_offset", dsc_codec.rc_range_parameters[i].range_bpg_offset, -32, 31);
				if ((i>0) && (prev_offset < rcOffset[i]))
					printf("WARNING: The RC_OFFSET values should not increase as the range increases\n");
				dsc_codec.rc_range_parameters[i].range_max_qp = rcMaxQp[i];
				RANGE_CHECK("range_max_qp", dsc_codec.rc_range_parameters[i].range_max_qp, 0, 15 + 2*(bitsPerComponent-8));
				if ((i>0) && (prev_max_qp > rcMaxQp[i]))
					printf("WARNING: The RC_MAX_QP values should not decrease as the range increases\n");
				dsc_codec.rc_range_parameters[i].range_min_qp = rcMinQp[i];
				RANGE_CHECK("range_min_qp", dsc_codec.rc_range_parameters[i].range_min_qp, 0, 15 + 2*(bitsPerComponent-8));
				if ((i>0) && (prev_min_qp > rcMinQp[i]))
					printf("WARNING: The RC_MIN_QP values should not decrease as the range increases\n");
				if (i<NUM_BUF_RANGES-1)
				{
					dsc_codec.rc_buf_thresh[i] = rcBufThresh[i];
					RANGE_CHECK("rc_buf_thresh", dsc_codec.rc_buf_thresh[i], 0, rcModelSize);
					if(dsc_codec.rc_buf_thresh[i] & 0x3f)
						UErr("All rc_buf_thresh must be evenly divisible by 64");
					if ((i>0) && (prev_thresh > rcBufThresh[i]))
						printf("WARNING: The RC_BUF_THRESH values should not decrease as the range increases\n");
					prev_thresh = rcBufThresh[i];
				}
				prev_min_qp = rcMinQp[i];
				prev_max_qp = rcMaxQp[i];
				prev_offset = rcOffset[i];
			}
			dsc_codec.rc_model_size = rcModelSize;
			RANGE_CHECK("rc_model_size", dsc_codec.rc_model_size, 0, 65535);		
			dsc_codec.initial_xmit_delay = initialDelay;  // Codec expects initial delay to be an integer number of groups
			RANGE_CHECK("initial_xmit_delay", dsc_codec.initial_xmit_delay, 0, 1023);		
			dsc_codec.block_pred_enable = bpEnable;
			RANGE_CHECK("block_pred_enable", dsc_codec.block_pred_enable, 0, 1);		
			dsc_codec.initial_offset = initialFullnessOfs;
			RANGE_CHECK("initial_offset", dsc_codec.initial_offset, 0, rcModelSize);		
			dsc_codec.first_line_bpg_ofs = firstLineBpgOfs;
			RANGE_CHECK("first_line_bpg_offset", dsc_codec.first_line_bpg_ofs, 0, 31);
			dsc_codec.xstart = 0;
			dsc_codec.ystart = 0;
			dsc_codec.flatness_min_qp = flatnessMinQp;
			RANGE_CHECK("flatness_min_qp", dsc_codec.flatness_min_qp, 0, 31);
			dsc_codec.flatness_max_qp = flatnessMaxQp;
			RANGE_CHECK("flatness_max_qp", dsc_codec.flatness_max_qp, 0, 31);
			dsc_codec.flatness_det_thresh = flatnessDetThresh;
			if(dsc_codec.rc_model_size <= dsc_codec.initial_offset)
				UErr("INITIAL_OFFSET must be less than RC_MODEL_SIZE\n");
			dsc_codec.initial_scale_value = 8 * dsc_codec.rc_model_size / (dsc_codec.rc_model_size - dsc_codec.initial_offset);
			RANGE_CHECK("initial_scale_value", dsc_codec.initial_scale_value, 0, 63);
			dsc_codec.vbr_enable = enableVbr;
			RANGE_CHECK("vbr_enable", dsc_codec.vbr_enable, 0, 1);

			// Compute slice dimensions
			slicew = (sliceWidth ? sliceWidth : ip->w);
			sliceh = (sliceHeight ? sliceHeight : ip->h);

			dsc_codec.slice_width = slicew;
			RANGE_CHECK("slice_width", dsc_codec.slice_width, 1, 65535);
			dsc_codec.slice_height = sliceh;
			RANGE_CHECK("slice_height", dsc_codec.slice_height, 1, 65535);

			// Compute rate buffer size for auto mode
			groupsPerLine = (dsc_codec.slice_width + 2) / 3;
			dsc_codec.chunk_size = (int)(ceil(slicew * bitsPerPixel / 8.0)); // Number of bytes per chunk
			RANGE_CHECK("chunk_size", dsc_codec.chunk_size, 0, 65535);
			rbsMin = (int)(dsc_codec.rc_model_size - initialFullnessOfs + ((int)ceil(initialDelay * bitsPerPixel)) + groupsPerLine * firstLineBpgOfs);
			hrdDelay = (int)(ceil((double)rbsMin / bitsPerPixel));
			dsc_codec.rcb_bits = (int)(ceil((double)hrdDelay * bitsPerPixel));
			dsc_codec.initial_dec_delay = hrdDelay - dsc_codec.initial_xmit_delay;
			RANGE_CHECK("initial_dec_delay", dsc_codec.initial_dec_delay, 0, 65535);

			if (dsc_codec.convert_rgb)
				num_extra_mux_bits = (muxingMode == 1) ? (3*(muxWordSize + (4*bitsPerComponent+4)-2)) : 0;
			else  // YCbCr
				num_extra_mux_bits = (muxingMode == 1) ? (3*muxWordSize + (4*bitsPerComponent+4) + 2*(4*bitsPerComponent) - 2) : 0;
			sliceBits = 8 * dsc_codec.chunk_size * dsc_codec.slice_height;
			while ((num_extra_mux_bits>0) && ((sliceBits - num_extra_mux_bits) % muxWordSize))
				num_extra_mux_bits--;

			if (groupsPerLine < dsc_codec.initial_scale_value - 8)
				dsc_codec.initial_scale_value = groupsPerLine + 8;
			if (dsc_codec.initial_scale_value > 8)
				dsc_codec.scale_decrement_interval = groupsPerLine / (dsc_codec.initial_scale_value - 8);
			else
				dsc_codec.scale_decrement_interval = 4095;
			RANGE_CHECK("scale_decrement_interval", dsc_codec.scale_decrement_interval, 0, 4095);
			final_value = dsc_codec.rc_model_size - ((dsc_codec.initial_xmit_delay * dsc_codec.bits_per_pixel + 8)>>4) + num_extra_mux_bits;
			dsc_codec.final_offset = final_value;
			RANGE_CHECK("final_offset", dsc_codec.final_offset, 0, 65535);
			if (final_value >= dsc_codec.rc_model_size)
				UErr("The final_offset must be less than the rc_model_size.  Try increasing initial_xmit_delay.\n");
			final_scale = 8 * dsc_codec.rc_model_size / (dsc_codec.rc_model_size - final_value);
			if (final_scale > 63)
				printf("WARNING: A final scale value > than 63/8 may have undefined behavior on some implementations.  Try increasing initial_xmit_delay.\n");
			if(dsc_codec.slice_height > 1)
				dsc_codec.nfl_bpg_offset = (int)ceil((double)(dsc_codec.first_line_bpg_ofs << OFFSET_FRACTIONAL_BITS) / (dsc_codec.slice_height - 1));
			else
				dsc_codec.nfl_bpg_offset = 0;
			RANGE_CHECK("nfl_bpg_offset", dsc_codec.nfl_bpg_offset, 0, 65535);
			groups_total = groupsPerLine * dsc_codec.slice_height;
			dsc_codec.slice_bpg_offset = (int)ceil((double)(1<<OFFSET_FRACTIONAL_BITS) * 
				       (dsc_codec.rc_model_size - dsc_codec.initial_offset + num_extra_mux_bits)
					   / (groups_total));
			RANGE_CHECK("slice_bpg_offset", dsc_codec.slice_bpg_offset, 0, 65535);

			if(dsc_codec.slice_height == 1)
			{
				if(dsc_codec.first_line_bpg_ofs > 0)
					UErr("For slice_height == 1, the FIRST_LINE_BPG_OFFSET must be 0\n");
			} else if(3.0 * bitsPerPixel - 
				  ((double)(dsc_codec.slice_bpg_offset + dsc_codec.nfl_bpg_offset)/(1<<OFFSET_FRACTIONAL_BITS)) < 16.0)
				UErr("The bits/pixel allocation for non-first lines is too low (<5.33bpp).\nConsider decreasing FIRST_LINE_BPG_OFFSET.");

			// BEGIN scale_increment_interval fix
			if(final_scale > 9)
			{
				// Note: the following calculation assumes that the rcXformOffset crosses 0 at some point.  If the zero-crossing
				//   doesn't occur in a configuration, we recommend to reconfigure the rc_model_size and thresholds to be smaller
				//   for that configuration.
				dsc_codec.scale_increment_interval = (int)((double)(1<<OFFSET_FRACTIONAL_BITS) * dsc_codec.final_offset / 
					                                 ((double)(final_scale - 9) * (dsc_codec.nfl_bpg_offset + dsc_codec.slice_bpg_offset)));
				if (dsc_codec.scale_increment_interval > 65535)
				{
					printf("ERROR: required scale increment interval is too high.  Consider using smaller slices or increase initial delay\n");
					exit(1);
				}
			}
			else
				dsc_codec.scale_increment_interval = 0;
			// END scale_increment_interval fix

			if (function==1)
			{
#ifdef WIN32
				sprintf(bitsfname, "%s\\%s.dsc", fn_o, base_name);
#else
				sprintf(bitsfname, "%s/%s.dsc", fn_o, base_name);
#endif
				if ((bits_fp = fopen(bitsfname, "wb")) == NULL)
				{
					printf("Fatal error: Cannot open bitstream output file %s\n", bitsfname);
					exit(1);
				}
				//fwrite((void *)&dsc_codec, sizeof(dsc_cfg_t), 1, bits_fp);
				fputc('D', bits_fp); fputc('S', bits_fp); fputc('C', bits_fp); fputc('F', bits_fp);
				write_pps(pps, &dsc_codec);
				for (i=0; i<PPS_SIZE; ++i)
					fputc(pps[i], bits_fp);
			}
		}
		else   //  (function == 2) => decode
		{
#ifdef WIN32
			sprintf(bitsfname, "%s\\%s.dsc", fn_o, base_name);
#else
			sprintf(bitsfname, "%s/%s.dsc", fn_o, base_name);
#endif
			if ((bits_fp = fopen(bitsfname, "rb")) == NULL)
			{
				printf("Fatal error: Cannot open bitstream input file %s\n", bitsfname);
				exit(1);
			}
			//fread((void *)&dsc_codec, sizeof(dsc_cfg_t), 1, bits_fp);
			if ((fgetc(bits_fp) != 'D') || (fgetc(bits_fp) != 'S') || (fgetc(bits_fp) != 'C') || (fgetc(bits_fp) != 'F'))
				UErr("DSC file read error, invalid magic number\n");
			for (i=0; i<PPS_SIZE; ++i)
				pps[i] = fgetc(bits_fp);
			parse_pps(pps, &dsc_codec);
			// Estimate rate buffer size based on delays:
			
			bitsPerPixel = (float)(dsc_codec.bits_per_pixel/16.0);
			dsc_codec.rcb_bits = (dsc_codec.initial_xmit_delay + dsc_codec.initial_dec_delay) * ((int)(ceil(bitsPerPixel * 3)));

			slicew = dsc_codec.slice_width;
			sliceh = dsc_codec.slice_height;
			if (muxWordSize==0)
				muxWordSize = (dsc_codec.bits_per_component==12) ? 64 : 48;
			dsc_codec.mux_word_size = muxWordSize;
		}
		bufsize = dsc_codec.chunk_size * sliceh;   // Total number of bytes to generate
		slices_per_line = (dsc_codec.pic_width + dsc_codec.slice_width - 1) / dsc_codec.slice_width;
		buf = (unsigned char **)malloc(sizeof(unsigned char *) * slices_per_line);
		for (i=0; i<slices_per_line; ++i)
			buf[i] = (unsigned char *)malloc(bufsize);

		op_dsc = (pic_t *)pcreate(FRAME, dsc_codec.convert_rgb ? RGB : YUV_HD, YUV_444, dsc_codec.pic_width, dsc_codec.pic_height);
		op_dsc->bits = bitsPerComponent;
		op_dsc->alpha = 0;
		if (dsc_codec.convert_rgb)
		{
			int tpidx;

			temp_pic = (pic_t **)malloc(sizeof(pic_t*) * 2);
			for (tpidx=0; tpidx<2; ++tpidx)
			{
				// Space for converting to YCoCg
				temp_pic[tpidx] = (pic_t *)pcreate(FRAME, YUV_HD, YUV_444, dsc_codec.pic_width, dsc_codec.pic_height);
				temp_pic[tpidx]->bits = bitsPerComponent;
				temp_pic[tpidx]->alpha = 0;	
			}
		}

		slicecount = 0;
		chunk_sizes = (int **)malloc(sizeof(int *) * slices_per_line);
		for(i=0; i<slices_per_line; ++i)
			chunk_sizes[i] = (int *)malloc(sizeof(int *) * sliceh);
		for (ys = 0; ys < dsc_codec.pic_height; ys+=sliceh)
		{
			if(function == 2)
				read_dsc_data(buf, dsc_codec.chunk_size, bits_fp, dsc_codec.vbr_enable, slices_per_line, dsc_codec.slice_height);
			for (xs = 0; xs < slices_per_line; xs++)
			{
				unsigned char *buf2;

				buf2 = buf[xs];
				numslices = ((dsc_codec.pic_width+slicew-1)/slicew) * ((dsc_codec.pic_height+sliceh-1)/sliceh);
				printf("Processing slice %d / %d\r", ++slicecount, numslices);
				fflush(stdout);  // For Bob.
				if(function != 2)
					memset(buf2, 0, bufsize);
				dsc_codec.xstart = xs * slicew;
				dsc_codec.ystart = ys;

				// Encoder
				if ((function==0) || (function==1))
					DSC_Encode(&dsc_codec, ip, op_dsc, buf2, temp_pic, chunk_sizes[xs]);

				// Decoder
				if ((function==0) || (function == 2))
					DSC_Decode(&dsc_codec, op_dsc, buf2, temp_pic); 
			}
			if(function == 1)
				write_dsc_data(buf, dsc_codec.chunk_size, bits_fp, dsc_codec.vbr_enable, slices_per_line, dsc_codec.slice_height, chunk_sizes);
		}
		printf("\n");
		for(i=0; i<slices_per_line; ++i)
			free(buf[i]);
		free(buf);

		if (temp_pic)
		{
			pdestroy(temp_pic[0]);
			pdestroy(temp_pic[1]);
			free(temp_pic);
		}

		// Convert 444 to 422 if coded as 422
		if (dsc_codec.enable_422)
		{
			ip2 = pcreate(FRAME, op_dsc->color, YUV_422, op_dsc->w, op_dsc->h);
			ip2->bits = op_dsc->bits;
			ip2->alpha = 0;
			simple444to422(op_dsc, ip2);
			pdestroy(op_dsc);
			op_dsc = ip2;
		}

		if (function!=1)  // Don't write if encode only
		{
			// R/B swap
			if (rbSwapOut)
			{
				for (i=0; i<op_dsc->h; ++i)
					for (j=0; j<op_dsc->w; ++j)
					{
						int tmp;
						tmp = op_dsc->data.rgb.r[i][j];
						op_dsc->data.rgb.r[i][j] = op_dsc->data.rgb.b[i][j];
						op_dsc->data.rgb.b[i][j] = tmp;
					}
			}
			strcpy(f, fn_o);
#ifdef WIN32
			strcat(f, "\\");
#else
			strcat(f, "/");
#endif
			strcat(f, base_name);
			if (!useppm)
			{
				strcat(f, ".out.dpx");
				if (dpx_write(f, op_dsc, dpxPadLineEnds, dpxWriteBSwap))
				{
					fprintf(stderr, "Error writing DPX file %s\n", f);
					exit(1);
				}
			} else {
				strcat(f, ".out.ppm");
				if (ppm_write(f, op_dsc))
				{
					fprintf(stderr, "Error writing PPM file %s\n", f);
					exit(1);
				}
			}
		}

		if (ip)
		{
			strcpy(f, fn_o);
#ifdef WIN32
			strcat(f, "\\");
#else
			strcat(f, "/");
#endif
			strcat(f, base_name);
			if (!useppm)
			{
				strcat(f, ".ref.dpx");
				if (dpx_write(f, ref_pic, dpxPadLineEnds, dpxWriteBSwap))
				{
					fprintf(stderr, "Error writing DPX file %s\n", f);
					exit(1);
				}
			} else {
				strcat(f, ".ref.ppm");
				if (ppm_write(f, ref_pic))
				{
					fprintf(stderr, "Error writing PPM file %s\n", f);
					exit(1);
				}
			}

			fprintf(logfp, "Filename: %s.%s\n",  base_name, extension);
			fprintf(logfp,"%2.2f bits/pixel, %d bits/component,", bitsPerPixel, bitsPerComponent);
			fprintf(logfp," %s, %s,", useYuvInput ? "YUV" : "RGB", dsc_codec.enable_422 ? "4:2:2" : "4:4:4");
			fprintf(logfp," %dx%d slices, block_pred_enable=%d\n", slicew, sliceh, bpEnable);
			compute_and_display_PSNR(ref_pic, op_dsc, ref_pic->bits, logfp);

			if (ref_pic != ip)
				pdestroy(ref_pic);
			pdestroy(ip);
		}
		pdestroy(op_dsc);

		fcnt++;
		infname[0] = '\0';
		fgets(infname, 512, list_fp);
	}

	fclose(list_fp);
	fclose(logfp);
	free(rcOffset);
	free(rcMinQp);
	free(rcMaxQp);
	free(rcBufThresh);
	return(0);
}
