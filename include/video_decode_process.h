#ifndef VIDEO_DECODE_PROCESS_H
#define VIDEO_DECODE_PROCESS_H
#include "NvApplicationProfiler.h"
#include "NvUtils.h"
#include <errno.h>
#include <fstream>
#include <iostream>
#include <linux/videodev2.h>
#include <malloc.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <nvbuf_utils.h>

#include "nvbuf_utils.h"

#include "NvVideoDecoder.h"
#include "NvVideoConverter.h"
#include "NvEglRenderer.h"
#include <queue>

#include <semaphore.h>

//#include "video_encode_process.h"
#include "mutex"
#include "condition_variable"
#include "thread"
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "video_encode_process.h"

#define TEST_ERROR(cond, str, label) if(cond) { \
                                        cerr << str << endl; \
                                        error = 1; \
                                        goto label; }

#define MICROSECOND_UNIT 1000000
#define CHUNK_SIZE 4000000
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

#define IS_NAL_UNIT_START(buffer_ptr) (!buffer_ptr[0] && !buffer_ptr[1] && \
        !buffer_ptr[2] && (buffer_ptr[3] == 1))

#define IS_NAL_UNIT_START1(buffer_ptr) (!buffer_ptr[0] && !buffer_ptr[1] && \
        (buffer_ptr[2] == 1))

#define H264_NAL_UNIT_CODED_SLICE  1
#define H264_NAL_UNIT_CODED_SLICE_IDR  5

#define HEVC_NUT_TRAIL_N  0
#define HEVC_NUT_RASL_R  9
#define HEVC_NUT_BLA_W_LP  16
#define HEVC_NUT_CRA_NUT  21

#define IVF_FILE_HDR_SIZE   32
#define IVF_FRAME_HDR_SIZE  12

#define IS_H264_NAL_CODED_SLICE(buffer_ptr) ((buffer_ptr[0] & 0x1F) == H264_NAL_UNIT_CODED_SLICE)
#define IS_H264_NAL_CODED_SLICE_IDR(buffer_ptr) ((buffer_ptr[0] & 0x1F) == H264_NAL_UNIT_CODED_SLICE_IDR)

#define GET_H265_NAL_UNIT_TYPE(buffer_ptr) ((buffer_ptr[0] & 0x7E) >> 1)
#define YUV_NUMS 6

#define USE_NVBUF_TRANSFORM_API

#define MAX_BUFFERS 32

typedef struct
{
    NvVideoDecoder *dec;
    NvVideoConverter *conv;
    uint32_t decoder_pixfmt;

    NvEglRenderer *renderer;

    char **in_file_path;
    std::ifstream **in_file;

    char *out_file_path;
    std::ofstream *out_file;

    bool disable_rendering;
    bool fullscreen;
    uint32_t window_height;
    uint32_t window_width;
    uint32_t window_x;
    uint32_t window_y;
    uint32_t out_pixfmt;
    uint32_t video_height;
    uint32_t video_width;
    uint32_t display_height;
    uint32_t display_width;
    uint32_t file_count;
    float fps;

    bool disable_dpb;

    bool input_nalu;

    bool copy_timestamp;
    bool flag_copyts;
    uint32_t start_ts;
    float dec_fps;
    uint64_t timestamp;
    uint64_t timestampincr;

    bool stats;

    int  stress_test;
    bool enable_metadata;
    bool bLoop;
    bool bQueue;
    bool enable_input_metadata;
    enum v4l2_skip_frames_type skip_frames;
    enum v4l2_memory output_plane_mem_type;
    enum v4l2_memory capture_plane_mem_type;
#ifndef USE_NVBUF_TRANSFORM_API
    enum v4l2_yuv_rescale_method rescale_method;
#endif

    std::queue < NvBuffer * > *conv_output_plane_buf_queue;
    pthread_mutex_t queue_lock;
    pthread_cond_t queue_cond;

    sem_t pollthread_sema; // Polling thread waits on this to be signalled to issue Poll
    sem_t decoderthread_sema; // Decoder thread waits on this to be signalled to continue q/dq loop
    pthread_t   dec_pollthread; // Polling thread, created if running in non-blocking mode.

    pthread_t dec_capture_loop; // Decoder capture thread, created if running in blocking mode.
    bool got_error;
    bool got_eos;
    bool vp9_file_header_flag;
    bool vp8_file_header_flag;
    int dst_dma_fd;
    int dmabuff_fd[MAX_BUFFERS];
    int numCapBuffers;
    int loop_count;
    int max_perf;
    int extra_cap_plane_buffer;
    int blocking_mode; // Set to true if running in blocking mode
} context_de;

using namespace std;
using namespace cv;
class NVDecoder
{
public:
NVDecoder();
~NVDecoder();

mutex mutex_basedata;
condition_variable cond_basedata;
bool gFlagNewBasedata = false;
unsigned char *yuv_datas[YUV_NUMS];

bool isWriteYUV = false;
int yuv_width = 1280;
int yuv_height =720;
int output_plane_index = 0;
//int pub_w = 608;
//int pub_h = 608;
int decoder_index = 0;
context_de ctx;
int decode_proc(int dec_index,int width, int height,string url);
int decode_h264(unsigned char *h264_data,int frameSize);
//unsigned char *getYUVData();
void query_and_set_capture(context_de * ctx);
static void *dec_capture_loop_fcn(void *arg);
int yuv_read = 0;
private:
NVEncoder nv_encoder;

};

#endif // MPPDECODER_H
