/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <cstdlib>
#include <cstring>

#include "video_decode.h"

#define CHECK_OPTION_VALUE(argp) if(!*argp || (*argp)[0] == '-') \
                                { \
                                    cerr << "Error: value not specified for option " << arg << endl; \
                                    goto error; \
                                }

#define CSV_PARSE_CHECK_ERROR(condition, str) \
    if (condition) {\
    cerr << "Error: " << str << endl; \
    goto error; \
    }

using namespace std;

static void
print_help(void)
{
    cerr << "\nvideo_decode <in-format> [options] <in-file>\n\n"
            "Supported formats:\n"
            "\tVP9\n"
            "\tVP8\n"
            "\tH264\n"
            "\tH265\n"
            "\tMPEG2\n"
            "\tMPEG4\n\n"
            "OPTIONS:\n"
            "\t-h,--help            Prints this text\n"
            "\t--dbg-level <level>  Sets the debug level [Values 0-3]\n\n"
            "\t--stats              Report profiling data for the app\n\n"
            "\tNOTE: this should not be used alongside -o option as it decreases the FPS value shown in --stats\n"
            "\t--disable-rendering  Disable rendering\n"
            "\t--max-perf           Enable maximum Performance \n"
            "\tNOTE: this should be set only for platform T194 or above\n"
            "\t--fullscreen         Fullscreen playback [Default = disabled]\n"
            "\t-ww <width>          Window width in pixels [Default = video-width]\n"
            "\t-wh <height>         Window height in pixels [Default = video-height]\n"
            "\t-loop <count>        Playback in a loop.[count = 1,2,...,n times looping , 0 = infinite looping]\n"
            "\t-queue [<file1> <file2> ....] Files are played in a queue manner\n"
            "\tNOTE: -queue should be the last option mentioned in the command line no other option should be mentioned after that.\n"
            "\t-wx <x-offset>       Horizontal window offset [Default = 0]\n"
            "\t-wy <y-offset>       Vertical window offset [Default = 0]\n\n"
            "\t-fps <fps>           Display rate in frames per second [Default = 30]\n\n"
            "\t-o <out-file>        Write to output file\n\n"
            "\tNOTE: Not to be used along-side -loop and -queue option.\n"
            "\t-f <out_pixfmt>      1 NV12, 2 I420 [Default = 1]\n\n"
            "\t-sf <value>          Skip frames while decoding [Default = 0]\n"
            "\tAllowed values for the skip-frames parameter:\n"
            "\t0 = Decode all frames\n"
            "\t1 = Skip non-reference frames\n"
            "\t2 = Decode only key frames\n\n"
            "\t--input-nalu         Input to the decoder will be nal units\n"
            "\t--input-chunks       Input to the decoder will be a chunk of bytes [Default]\n\n"
            "\t--copy-timestamp <st> <fps> Enable copy timestamp with start timestamp(st) in seconds for decode fps(fps) (for input-nalu mode)\n"
            "\tNOTE: copy-timestamp used to demonstrate how timestamp can be associated with an individual H264/H265 frame to achieve video-synchronization.\n"
            "\t      currenly only supported for H264 & H265 video encode using MM APIs and is only for demonstration purpose.\n"
            "\t--report-metadata    Enable metadata reporting\n\n"
            "\t--blocking-mode <val> Set blocking mode, 0 is non-blocking, 1 for blocking (Default) \n\n"
            "\t--report-input-metadata  Enable metadata reporting for input header parsing error\n\n"
            "\t-v4l2-memory-out-plane <num>       Specify memory type to be used on Output Plane [1 = V4L2_MEMORY_MMAP, 2 = V4L2_MEMORY_USERPTR], Default = V4L2_MEMORY_MMAP\n\n"
            "\t-v4l2-memory-cap-plane <num>       Specify memory type to be used on Capture Plane [1 = V4L2_MEMORY_MMAP, 2 = V4L2_MEMORY_DMABUF], Default = V4L2_MEMORY_DMABUF\n\n"
            "\t-s <loop-count>      Stress test [Default = 1]\n\n"
            "\t-extra_cap_plane_buffer <num>      Specify extra capture plane buffers (Default=1, MAX=32) to be allocated\n"
#ifndef USE_NVBUF_TRANSFORM_API
            "\t--do-yuv-rescale     Rescale decoded YUV from full range to limited range\n\n"
#endif
            ;
}

static uint32_t
get_decoder_type(char *arg)
{
    if (!strcmp(arg, "H264"))
        return V4L2_PIX_FMT_H264;
    if (!strcmp(arg, "H265"))
        return V4L2_PIX_FMT_H265;
    if (!strcmp(arg, "VP9"))
        return V4L2_PIX_FMT_VP9;
    if (!strcmp(arg, "VP8"))
        return V4L2_PIX_FMT_VP8;
    if (!strcmp(arg, "MPEG2"))
        return V4L2_PIX_FMT_MPEG2;
    if (!strcmp(arg, "MPEG4"))
        return V4L2_PIX_FMT_MPEG4;
    return 0;
}

static int32_t
get_dbg_level(char *arg)
{
    int32_t log_level = atoi(arg);

    if (log_level < 0)
    {
        cout << "Warning: invalid log level input, defaulting to setting 0" << endl;
        return 0;
    }

    if (log_level > 3)
    {
        cout << "Warning: invalid log level input, defaulting to setting 3" << endl;
        return 3;
    }

    return log_level;
}

int
parse_csv_decoder(context_de * ctx)
{
    ctx->decoder_pixfmt = get_decoder_type("H264");
    CSV_PARSE_CHECK_ERROR(ctx->decoder_pixfmt == 0,
                          "Incorrect input format"); 
    ctx->input_nalu = true;
    ctx->disable_rendering = true;      
    return 0;

error:
    print_help();
    return -1;
}
