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
#include <linux/v4l2-controls.h>
#include "video_encode.h"

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
    cerr << "\nvideo_encode <in-file> <in-width> <in-height> <encoder-type> <out-file> [OPTIONS]\n\n"
            "Encoder Types:\n"
            "\tH264\n"
            "\tH265\n"
            "\tVP8\n"
            "\tVP9\n\n"
            "OPTIONS:\n"
            "\t-h,--help             Prints this text\n"
            "\t--dbg-level <level>   Sets the debug level [Values 0-3]\n\n"
            "\t--stats               Report profiling data for the app\n\n"
            "\t-br <bitrate>         Bitrate [Default = 4000000]\n"
            "\t-pbr <peak_bitrate>   Peak bitrate [Default = 1.2*bitrate]\n\n"
            "NOTE: Peak bitrate takes effect in VBR more; must be >= bitrate\n\n"
            "\t-p <profile>          Encoding Profile [Default = baseline]\n"
            "\t-l <level>            Encoding Level [Default set by the library]\n"
            "\t-rc <rate-control>    Ratecontrol mode [Default = cbr]\n"
            "\t--elossless           Enable Lossless encoding [Default = disabled,"
                                     "Option applicable only with YUV444 input and H264 encoder]\n"
            "\t--max-perf            Enable maximum Performance \n"
            "\t-ifi <interval>       I-frame Interval [Default = 30]\n"
            "\t-idri <interval>      IDR Interval [Default = 256]\n"
            "\t--insert-spspps-idr   Insert SPS PPS at every IDR [Default = disabled]\n"
            "\t--insert-vui          Insert VUI [Default = disabled]\n"
            "\t--enable-extcolorfmt  Set Extended ColorFormat (Only works with insert-vui) [Default = disabled]\n"
            "\t--insert-aud          Insert AUD [Default = disabled]\n"
            "\t--alliframes          Enable all I-frame encoding [Default = disabled]\n"
            "\t-fps <num> <den>      Encoding fps in num/den [Default = 30/1]\n\n"
            "\t-tt <level>           Temporal Tradeoff level [Default = 0]\n"
            "\t-vbs <size>           Virtual buffer size [Default = 0]\n"
            "\t-nrf <num>            Number of reference frames [Default = 1]\n\n"
            "\t-slt <type>           Slice length type (1 = Number of MBs, 2 = Bytes) [Default = 1]\n"
            "\t-hpt <type>           HW preset type (1 = ultrafast, 2 = fast, 3 = medium,  4 = slow)\n"
            "\t-slen <length>        Slice length [Default = 0]\n"
            "\t--sle                 Slice level encode output [Default = disabled]\n"
            "\t--cd                  CABAC Disable for H264 [Default = disabled]\n"
            "\t-sir <interval>       Slice intrarefresh interval [Default = 0]\n\n"
            "\t-nbf <num>            Number of B frames [Default = 0]\n\n"
            "\t-rpc <string>         Change configurable parameters at runtime\n\n"
            "\t-goldcrc <string>     GOLD CRC\n\n"
            "\t--rcrc                Reconstructed surface CRC\n\n"
            "\t-rl <cordinate>       Reconstructed surface Left cordinate [Default = 0]\n\n"
            "\t-rt <cordinate>       Reconstructed surface Top cordinate [Default = 0]\n\n"
            "\t-rw <val>             Reconstructed surface width\n\n"
            "\t-rh <val>             Reconstructed surface height\n\n"
            "\t-rcrcf <reconref_file_path> Specify recon crc reference param file\n\n"
            "\t--report-metadata     Print encoder output metadata\n"
            "\t--blocking-mode <val> Set blocking mode, 0 is non-blocking, 1 for blocking (Default) \n\n"
            "\t--input-metadata      Enable encoder input metadata\n"
            "\t--copy-timestamp <st> Enable copy timestamp with start timestamp(st) in seconds\n"
            "\t--mvdump              Dump encoded motion vectors\n\n"
            "\t--eroi                Enable ROI [Default = disabled]\n\n"
            "\t-roi <roi_file_path>  Specify roi param file\n\n"
            "\t--erps                Enable External RPS [Default = disabled]\n\n"
            "\t--egdr                Enable GDR [Default = disabled]\n\n"
            "\t--gif                 Enable Gaps in FrameNum [Default = disabled]\n\n"
            "\t-fnb <num_bits>       H264 FrameNum bits [Default = 0]\n\n"
            "\t-plb <num_bits>       H265 poc lsb bits [Default = 0]\n\n"
            "\t--ni                  No I-frames [Default = disabled]\n\n"
            "\t-rpsf <rps_file_path> Specify external rps param file\n\n"
            "\t--erh                 Enable External picture RC [Default = disabled]\n\n"
            "\t-mem_type_oplane <num> Specify memory type for the output plane to be used [1 = V4L2_MEMORY_MMAP, 2 = V4L2_MEMORY_USERPTR, 3 = V4L2_MEMORY_DMABUF]\n\n"
            "\t-gdrf <gdr_file_path> Specify GDR Parameters filename \n\n"
            "\t-gdrof <gdr_out_file_path> Specify GDR Out filename \n\n"
            "\t-smq <max_qp_value>   Max QP per session when external picture RC enabled\n\n"
            "\t-hf <hint_file_path>  Specify external rate control param file\n\n"
            "\t-MinQpI               Specify minimum Qp Value for I frame\n\n"
            "\t-MaxQpI               Specify maximum Qp Value for I frame\n\n"
            "\t-MinQpP               Specify minimum Qp Value for P frame\n\n"
            "\t-MaxQpP               Specify maximum Qp Value for P frame\n\n"
            "\t-MinQpB               Specify minimum Qp Value for B frame\n\n"
            "\t-MaxQpB               Specify maximum Qp Value for B frame\n\n"
            "\t-s <loop-count>       Stress test [Default = 1]\n\n"
            "\t-sf <val>             Starting frame number [Default = 0]\n\n"
            "\t-ef <val>             Ending frame number [Default = 0, all frame will be encoded]\n\n"
            "NOTE: roi parameters need to be feed per frame in following format\n"
            "      <no. of roi regions> <Qpdelta> <left> <top> <width> <height> ...\n"
            "      e.g. [Each line corresponds roi parameters for one frame] \n"
            "      2   -2   34  33  16  19  -3  68  68  16  16\n"
            "      1   -5   40  40  40  40\n"
            "      3   -4   34  34  16  16  -5  70  70  18  18  -3  100  100  34  34\n"
            "Supported Encoding profiles for H.264:\n"
            "\tbaseline\tmain\thigh\n"
            "Supported Encoding profiles for H.265:\n"
            "\tmain\n"
            "\tmain10\n\n"
            "Supported Encoding levels for H.264\n"
            "\t1.0\t1b\t1.1\t1.2\t1.3\n"
            "\t2.0\t2.1\t2.2\n"
            "\t3.0\t3.1\t3.2\n"
            "\t4.0\t4.1\t4.2\n"
            "\t5.0\t5.1\n"
            "Supported Encoding levels for H.265\n"
            "\tmain1.0\thigh1.0\n"
            "\tmain2.0\thigh2.0\tmain2.1\thigh2.1\n"
            "\tmain3.0\thigh3.0\tmain3.1\thigh3.1\n"
            "\tmain4.0\thigh4.0\tmain4.1\thigh4.1\n"
            "\tmain5.0\thigh5.0\tmain5.1\thigh5.1\tmain5.2\thigh5.2\n"
            "\tmain6.0\thigh6.0\tmain6.1\thigh6.1\tmain6.2\thigh6.2\n\n"
            "Supported Encoding rate control modes:\n"
            "\tcbr\tvbr\n\n"
            "Supported Temporal Tradeoff levels:\n"
            "0:Drop None       1:Drop 1 in 5      2:Drop 1 in 3\n"
            "3:Drop 1 in 2     4:Drop 2 in 3\n\n"
            "Runtime configurable parameter string should be of the form:\n"
            "\"f<frame_num1>,<prop_id1><val>,<prop_id2><val>#f<frame_num1>,<prop_id1><val>,<prop_id2><val>#...\"\n"
            "e.g. \"f20,b8000000,i1#f300,b6000000,r40/1\"\n\n"
            "Property ids:\n"
            "\tb<bitrate>  Bitrate\n"
            "\tp<peak_bitrate>  Peak Bitrate\n"
            "\tr<num/den>  Framerate\n"
            "\ti1          Force I-frame\n\n"
            "NOTE: These encoding parameters are slightly imprecisely updated depending upon the number of\n"
            "frames in queue and/or processed already.\n";
}

static uint32_t
get_encoder_type(char *arg)
{
    if (!strcmp(arg, "H264"))
        return V4L2_PIX_FMT_H264;
    if (!strcmp(arg, "H265"))
        return V4L2_PIX_FMT_H265;
    if (!strcmp(arg, "VP8"))
        return V4L2_PIX_FMT_VP8;
    if (!strcmp(arg, "VP9"))
        return V4L2_PIX_FMT_VP9;
    return 0;
}

static int32_t
get_encoder_ratecontrol(char *arg)
{
    if (!strcmp(arg, "cbr"))
        return V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;

    if (!strcmp(arg, "vbr"))
        return V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;

    return -1;
}

static int32_t
get_encoder_profile_h264(char *arg)
{
    if (!strcmp(arg, "baseline"))
        return V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;

    if (!strcmp(arg, "main"))
        return V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;

    if (!strcmp(arg, "high"))
        return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
    return -1;
}

static int32_t
get_encoder_profile_h265(char *arg)
{
    if (!strcmp(arg, "main"))
        return V4L2_MPEG_VIDEO_H265_PROFILE_MAIN;

    if (!strcmp(arg, "main10"))
        return V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10;

    return -1;
}

static int32_t
get_h264_encoder_level(char *arg)
{
    if (!strcmp(arg, "1.0"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_1_0;

    if (!strcmp(arg, "1b"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_1B;

    if (!strcmp(arg, "1.1"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_1_1;

    if (!strcmp(arg, "1.2"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_1_2;

    if (!strcmp(arg, "1.3"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_1_3;

    if (!strcmp(arg, "2.0"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_2_0;

    if (!strcmp(arg, "2.1"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_2_1;

    if (!strcmp(arg, "2.2"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_2_2;

    if (!strcmp(arg, "3.0"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_3_0;

    if (!strcmp(arg, "3.1"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_3_1;

    if (!strcmp(arg, "3.2"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_3_2;

    if (!strcmp(arg, "4.0"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_4_0;

    if (!strcmp(arg, "4.1"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_4_1;

    if (!strcmp(arg, "4.2"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_4_2;

    if (!strcmp(arg, "5.0"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_5_0;

    if (!strcmp(arg, "5.1"))
        return V4L2_MPEG_VIDEO_H264_LEVEL_5_1;

    return -1;
}

static int32_t
get_h265_encoder_level(char *arg)
{
    if (!strcmp(arg, "main1.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_1_0_MAIN_TIER;

    if (!strcmp(arg, "high1.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_1_0_HIGH_TIER;

    if (!strcmp(arg, "main2.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_2_0_MAIN_TIER;

    if (!strcmp(arg, "high2.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_2_0_HIGH_TIER;

    if (!strcmp(arg, "main2.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_2_1_MAIN_TIER;

    if (!strcmp(arg, "high2.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_2_1_HIGH_TIER;

    if (!strcmp(arg, "main3.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_3_0_MAIN_TIER;

    if (!strcmp(arg, "high3.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_3_0_HIGH_TIER;

    if (!strcmp(arg, "main3.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_3_1_MAIN_TIER;

    if (!strcmp(arg, "high3.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_3_1_HIGH_TIER;

    if (!strcmp(arg, "main4.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_4_0_MAIN_TIER;

    if (!strcmp(arg, "high4.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_4_0_HIGH_TIER;

    if (!strcmp(arg, "main4.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_4_1_MAIN_TIER;

    if (!strcmp(arg, "high4.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_4_1_HIGH_TIER;

    if (!strcmp(arg, "main5.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_5_0_MAIN_TIER;

    if (!strcmp(arg, "high5.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_5_0_HIGH_TIER;

    if (!strcmp(arg, "main5.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_5_1_MAIN_TIER;

    if (!strcmp(arg, "high5.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_5_1_HIGH_TIER;

    if (!strcmp(arg, "main5.2"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_5_2_MAIN_TIER;

    if (!strcmp(arg, "high5.2"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_5_2_HIGH_TIER;

    if (!strcmp(arg, "main6.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_6_0_MAIN_TIER;

    if (!strcmp(arg, "high6.0"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_6_0_HIGH_TIER;

    if (!strcmp(arg, "main6.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_6_1_MAIN_TIER;

    if (!strcmp(arg, "high6.1"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_6_1_HIGH_TIER;

    if (!strcmp(arg, "main6.2"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_6_2_MAIN_TIER;

    if (!strcmp(arg, "high6.2"))
        return V4L2_MPEG_VIDEO_H265_LEVEL_6_2_HIGH_TIER;

    return -1;
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
parse_csv_encoder(context_t * ctx,int yuv_width,int yuv_heigh)
{
    ctx->width = yuv_width;
    ctx->height = yuv_heigh;
    ctx->encoder_pixfmt = get_encoder_type("H264");
    CSV_PARSE_CHECK_ERROR(ctx->encoder_pixfmt == 0,
                          "Incorrect input format");     
    return 0;

error:
    print_help();
    return -1;
}
