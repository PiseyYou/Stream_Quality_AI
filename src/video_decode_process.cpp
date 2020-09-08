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
#include "video_decode_process.h"
#define OPEN_ENCODER
#define ENCODE_NUMS 30

//#define OPEN_IMAGE_SHOW
//context_t ctx_encoder;

NVDecoder::NVDecoder()
{
}

NVDecoder::~NVDecoder()
{
}

static void
abort(context_de *ctx)
{
    ctx->got_error = true;
    ctx->dec->abort();
}

void
NVDecoder::query_and_set_capture(context_de * ctx)
{
    NvVideoDecoder *dec = ctx->dec;
    struct v4l2_format format;
    struct v4l2_crop crop;
    int32_t min_dec_capture_buffers;
    int ret = 0;
    int error = 0;
    uint32_t window_width;
    uint32_t window_height;
    NvBufferCreateParams input_params = {0};
    NvBufferCreateParams cParams = {0};

    // Get capture plane format from the decoder. This may change after
    // an resolution change event
    ret = dec->capture_plane.getFormat(format);
    TEST_ERROR(ret < 0,
               "Error: Could not get format from decoder capture plane", error);

    // Get the display resolution from the decoder
    ret = dec->capture_plane.getCrop(crop);
    TEST_ERROR(ret < 0,
               "Error: Could not get crop from decoder capture plane", error);

    cout << "Video Resolution: " << crop.c.width << "x" << crop.c.height
        << endl;
    ctx->display_height = crop.c.height;
    ctx->display_width = crop.c.width;

    if(ctx->dst_dma_fd != -1)
    {
        NvBufferDestroy(ctx->dst_dma_fd);
        ctx->dst_dma_fd = -1;
    }

    input_params.payloadType = NvBufferPayload_SurfArray;
    input_params.width = yuv_width;
    input_params.height = yuv_height;
    input_params.layout = NvBufferLayout_Pitch;
    input_params.colorFormat = ctx->out_pixfmt == 1 ? NvBufferColorFormat_NV12 : NvBufferColorFormat_YUV420;
    input_params.nvbuf_tag = NvBufferTag_VIDEO_CONVERT;

    ret = NvBufferCreateEx (&ctx->dst_dma_fd, &input_params);
    TEST_ERROR(ret == -1, "create dmabuf failed", error);

    if (!ctx->disable_rendering)
    {
        // Destroy the old instance of renderer as resolution might have changed
        delete ctx->renderer;

        if (ctx->fullscreen)
        {
            // Required for fullscreen
            window_width = window_height = 0;
        }
        else if (ctx->window_width && ctx->window_height)
        {
            // As specified by user on commandline
            window_width = ctx->window_width;
            window_height = ctx->window_height;
        }
        else
        {
            // Resolution got from the decoder
            window_width = crop.c.width/2;
            window_height = crop.c.height/2;
        }

        // If height or width are set to zero, EglRenderer creates a fullscreen
        // window
        ctx->renderer =
                NvEglRenderer::createEglRenderer("renderer0", window_width,
                                           window_height, ctx->window_x,
                                           ctx->window_y);
        TEST_ERROR(!ctx->renderer,
                   "Error in setting up renderer. "
                   "Check if X is running or run with --disable-rendering",
                   error);
        if (ctx->stats)
        {
            ctx->renderer->enableProfiling();
        }

        ctx->renderer->setFPS(ctx->fps);
    }

    // deinitPlane unmaps the buffers and calls REQBUFS with count 0
    dec->capture_plane.deinitPlane();
    if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
    {
        for(int index = 0 ; index < ctx->numCapBuffers ; index++)
        {
            if(ctx->dmabuff_fd[index] != 0)
            {
                ret = NvBufferDestroy (ctx->dmabuff_fd[index]);
                TEST_ERROR(ret < 0, "Failed to Destroy NvBuffer", error);
            }
        }
    }

    // Not necessary to call VIDIOC_S_FMT on decoder capture plane.
    // But decoder setCapturePlaneFormat function updates the class variables
    ret = dec->setCapturePlaneFormat(format.fmt.pix_mp.pixelformat,
                                     format.fmt.pix_mp.width,
                                     format.fmt.pix_mp.height);
    TEST_ERROR(ret < 0, "Error in setting decoder capture plane format", error);

    ctx->video_height = format.fmt.pix_mp.height;
    ctx->video_width = format.fmt.pix_mp.width;
    // Get the minimum buffers which have to be requested on the capture plane
    ret = dec->getMinimumCapturePlaneBuffers(min_dec_capture_buffers);
    TEST_ERROR(ret < 0,
               "Error while getting value of minimum capture plane buffers",
               error);

    // Request (min + extra) buffers, export and map buffers
    if(ctx->capture_plane_mem_type == V4L2_MEMORY_MMAP)
    {
        ret =
            dec->capture_plane.setupPlane(V4L2_MEMORY_MMAP,
                                          min_dec_capture_buffers + ctx->extra_cap_plane_buffer, false,
                                          false);
        TEST_ERROR(ret < 0, "Error in decoder capture plane setup", error);
    }
    else if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
    {
        switch(format.fmt.pix_mp.colorspace)
        {
            case V4L2_COLORSPACE_SMPTE170M:
                if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
                {
                    cout << "Decoder colorspace ITU-R BT.601 with standard range luma (16-235)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12;
                }
                else
                {
                    cout << "Decoder colorspace ITU-R BT.601 with extended range luma (0-255)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_ER;
                }
                break;
            case V4L2_COLORSPACE_REC709:
                if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
                {
                    cout << "Decoder colorspace ITU-R BT.709 with standard range luma (16-235)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_709;
                }
                else
                {
                    cout << "Decoder colorspace ITU-R BT.709 with extended range luma (0-255)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_709_ER;
                }
                break;
            case V4L2_COLORSPACE_BT2020:
                {
                    cout << "Decoder colorspace ITU-R BT.2020" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_2020;
                }
                break;
            default:
                cout << "supported colorspace details not available, use default" << endl;
                if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
                {
                    cout << "Decoder colorspace ITU-R BT.601 with standard range luma (16-235)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12;
                }
                else
                {
                    cout << "Decoder colorspace ITU-R BT.601 with extended range luma (0-255)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_ER;
                }
                break;
        }

        ctx->numCapBuffers = min_dec_capture_buffers + ctx->extra_cap_plane_buffer;
	
      

        for (int index = 0; index < ctx->numCapBuffers; index++)
        {
            cParams.width = crop.c.width;
            cParams.height = crop.c.height;
            cParams.layout = NvBufferLayout_BlockLinear;
            cParams.payloadType = NvBufferPayload_SurfArray;
            cParams.nvbuf_tag = NvBufferTag_VIDEO_DEC;
            ret = NvBufferCreateEx(&ctx->dmabuff_fd[index], &cParams);
            TEST_ERROR(ret < 0, "Failed to create buffers", error);
        }
        ret = dec->capture_plane.reqbufs(V4L2_MEMORY_DMABUF,ctx->numCapBuffers);
            TEST_ERROR(ret, "Error in request buffers on capture plane", error);
    }

    // Capture plane STREAMON
    ret = dec->capture_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in decoder capture plane streamon", error);

    // Enqueue all the empty capture plane buffers
    cout << "dec->capture_plane.getNumBuffers():" << dec->capture_plane.getNumBuffers() << endl;
    for (uint32_t i = 0; i < dec->capture_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory = ctx->capture_plane_mem_type;
        if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
            v4l2_buf.m.planes[0].m.fd = ctx->dmabuff_fd[i];
        ret = dec->capture_plane.qBuffer(v4l2_buf, NULL);
        TEST_ERROR(ret < 0, "Error Qing buffer at output plane", error);
    }
    cout << "Query and set capture successful" << endl;
    return;

error:
    if (error)
    {
        abort(ctx);
        cerr << "Error in " << __func__ << endl;
    }
}


void*
NVDecoder::dec_capture_loop_fcn(void *arg)
{
    NVDecoder *ptr = (NVDecoder*)arg;
    context_de *ctx = &(ptr->ctx);
    NvVideoDecoder *dec = ctx->dec;
    struct v4l2_event ev;
    int ret;

    cout << "Starting decoder capture loop thread" << endl;
    // Need to wait for the first Resolution change event, so that
    // the decoder knows the stream resolution and can allocate appropriate
    // buffers when we call REQBUFS
    do
    {
	//cout << "ev.type: " << ev.type << endl;
	//cout << "V4L2_EVENT_RESOLUTION_CHANGE: " << V4L2_EVENT_RESOLUTION_CHANGE << endl;
        ret = dec->dqEvent(ev, 50000);
	//cout << "ret................... : " << ret << endl;
        if (ret < 0)
        {
            if (errno == EAGAIN)
            {
                cerr <<
                    "Timed out waiting for first V4L2_EVENT_RESOLUTION_CHANGE"
                    << endl;
            }
            else
            {
                cerr << "Error in dequeueing decoder event" << endl;
            }
            abort(ctx);
            break;
        }
    }
    while ((ev.type != V4L2_EVENT_RESOLUTION_CHANGE) && !ctx->got_error);
    // query_and_set_capture acts on the resolution change event
    if (!ctx->got_error)
        ptr->query_and_set_capture(ctx);

    // Exit on error or EOS which is signalled in main()
    while (!(ctx->got_error || dec->isInError() || ctx->got_eos))
    {
        NvBuffer *dec_buffer;
        // Check for Resolution change again
        ret = dec->dqEvent(ev, false);
        if (ret == 0)
        {
            switch (ev.type)
            {
                case V4L2_EVENT_RESOLUTION_CHANGE:
                    ptr->query_and_set_capture(ctx);
                    continue;
            }
        }
	int count1 = 0;
	/* FPS calculator */
	long int fps_ms;
	long int diff;
	int fps_counter;
	float fps = 0.0;
	struct timeval curTime_tv;
	struct timeval curTime_ts;
	gettimeofday(&curTime_tv,NULL);
	fps_ms = curTime_tv.tv_sec*1000 + curTime_tv.tv_usec/1000;
	//cout << "curTime.tv_usec:" << curTime_tv.tv_usec << endl;
	//cout << "curTime.tv_sec:" << curTime_tv.tv_sec << endl;

        while (1)
        {
            struct v4l2_buffer v4l2_buf;
            struct v4l2_plane planes[MAX_PLANES];
	    //if(ptr->decoder_index == 0) cout << "capture: " << count1 << endl;
	    count1 ++;
            memset(&v4l2_buf, 0, sizeof(v4l2_buf));
            memset(planes, 0, sizeof(planes));
            v4l2_buf.m.planes = planes;

            // Dequeue a filled buffer
            if (dec->capture_plane.dqBuffer(v4l2_buf, &dec_buffer, NULL, 0))
            {
                if (errno == EAGAIN)
                {
                    usleep(1000);
                }
                else
                {
                    abort(ctx);
                    cerr << "Error while calling dequeue at capture plane" <<
                        endl;
                }
                break;
            }

            if (ctx->copy_timestamp && ctx->input_nalu && ctx->stats)
            {
              cout << "[" << v4l2_buf.index << "]" "dec capture plane dqB timestamp [" <<
                  v4l2_buf.timestamp.tv_sec << "s" << v4l2_buf.timestamp.tv_usec << "us]" << endl;
            }

            if (!ctx->disable_rendering && ctx->stats)
            {
                // EglRenderer requires the fd of the 0th plane to render the buffer
                if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
                    dec_buffer->planes[0].fd = ctx->dmabuff_fd[v4l2_buf.index];
                ctx->renderer->render(dec_buffer->planes[0].fd);
            }

            // If we need to write to file or display the buffer,
            // give the buffer to video converter output plane
            // instead of returning the buffer back to decoder capture plane
            if (true)
            {
                /* Clip & Stitch can be done by adjusting rectangle */
                NvBufferRect src_rect, dest_rect;
                src_rect.top = 0;
                src_rect.left = 0;
                src_rect.width = ctx->display_width;
                src_rect.height = ctx->display_height;
                dest_rect.top = 0;
                dest_rect.left = 0;
                //dest_rect.width = ctx->display_width;
                //dest_rect.height = ctx->display_height;
		dest_rect.width = ptr->yuv_width;
                dest_rect.height = ptr->yuv_height;

                NvBufferTransformParams transform_params;
                memset(&transform_params,0,sizeof(transform_params));
                /* Indicates which of the transform parameters are valid */
                transform_params.transform_flag = NVBUFFER_TRANSFORM_FILTER;
		//transform_params.transform_flag = NVBUFFER_TRANSFORM_CROP_DST;
                transform_params.transform_flip = NvBufferTransform_None;
                transform_params.transform_filter = NvBufferTransform_Filter_Smart;
                transform_params.src_rect = src_rect;
                transform_params.dst_rect = dest_rect;

                if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
                    dec_buffer->planes[0].fd = ctx->dmabuff_fd[v4l2_buf.index];

#ifdef OPEN_ENCODER
//if(ptr->decoder_index >= ENCODE_NUMS) ret = NvBufferTransform(dec_buffer->planes[0].fd, ctx->dst_dma_fd, &transform_params);
//else
{
// add encode here
		struct v4l2_buffer v4l2_buf_en;
		struct v4l2_plane planes_en[MAX_PLANES];
		NvBuffer *buffer_en;

		memset(&v4l2_buf_en, 0, sizeof(v4l2_buf_en));
		memset(planes_en, 0, sizeof(planes_en));

		v4l2_buf_en.m.planes = planes_en;

		//dequeue buffer from encoder output plane //
		if (ptr->nv_encoder.ctx.enc->output_plane.dqBuffer(v4l2_buf_en, &buffer_en, NULL, 10) < 0)
		{
		    cerr << "ERROR while DQing buffer at output plane" << endl;
		}
//add encode
                // Convert Blocklinear to PitchLinear
                //ret = NvBufferTransform(dec_buffer->planes[0].fd, ctx->dst_dma_fd, &transform_params);
		ret = NvBufferTransform(dec_buffer->planes[0].fd, buffer_en->planes[0].fd, &transform_params);
                if (ret == -1)
                {
                    cerr << "Transform failed" << endl;
                    break;
                }
		/*int dmabuf_fd = ctx->dst_dma_fd;
		int ret = -1;
		NvBufferParams parm;
		ret = NvBufferGetParams(dmabuf_fd, &parm);

		if (ret != 0)
		{
			printf("GetParams failed \n");
		}
		//isWriteYUV = false;
		char *data;
		void *psrc_data;
		for(int j = 0;j<3; j++)
		{
		    ret = NvBufferMemMap(dmabuf_fd, j, NvBufferMem_Read_Write, &psrc_data);
		    //printf("parm.height[%d]:%d\n",j,parm.height[j]);
		    //printf("parm.width[%d]:%d\n",j,parm.width[j]);
		    //printf("parm.pitch[%d]:%d\n",j,parm.pitch[j]);
		    if (ret == 0)
		    {
			unsigned int i = 0;
			NvBufferMemSyncForCpu(dmabuf_fd, j, &psrc_data);
			NvBuffer::NvBufferPlane &plane = buffer_en->planes[j];
			data = (char *) plane.data;
			for (i = 0; i < parm.height[j]; ++i)
			{
			    
			    if(j == 0)
			    {
				memcpy(data + i * plane.fmt.stride,
					(unsigned char *)psrc_data + i * parm.pitch[j],parm.width[j]);
			    }
			    else if(j == 1)
			    {
				memcpy(data + i * plane.fmt.stride,
					(unsigned char *)psrc_data + i * parm.pitch[j],parm.width[j]);
			    }
			    else if(j == 2)
			    {
				memcpy(data + i * plane.fmt.stride,
					(unsigned char *)psrc_data + i * parm.pitch[j],parm.width[j]);
			    }
			    //cout << "parm.height["<< j <<"]=" << parm.height[j] << endl;
			    //cout << "parm.pitch["<< j <<"]=" << parm.pitch[j] << endl;
			    //cout << "plane.fmt.stride...=" << plane.fmt.stride << endl;
			    //cout << "plane.fmt.height...=" << plane.fmt.height << endl;	    
			}
			//memcpy(data,(unsigned char *)psrc_data,parm.pitch[j] * parm.height[j]);
			plane.bytesused = plane.fmt.stride * plane.fmt.height;
			NvBufferMemUnMap(dmabuf_fd, j, &psrc_data);
		    }
		    else
		    {
			printf("NvBufferMap failed \n");
		    }
		}*/

//add encode here
		if(ptr->nv_encoder.ctx.output_memory_type == V4L2_MEMORY_DMABUF || ptr->nv_encoder.ctx.output_memory_type == V4L2_MEMORY_MMAP)
		{
		    for (uint32_t j = 0 ; j < buffer_en->n_planes ; j++)
		    {
			buffer_en->planes[j].bytesused = buffer_en->planes[j].fmt.stride * buffer_en->planes[j].fmt.height;
			ret = NvBufferMemSyncForDevice (buffer_en->planes[j].fd, j, (void **)&buffer_en->planes[j].data);
			if (ret < 0)
			{
			    cerr << "Error while NvBufferMemSyncForDevice at output plane for V4L2_MEMORY_DMABUF" << endl;
			}
		    }
		}

		if(ptr->nv_encoder.ctx.output_memory_type == V4L2_MEMORY_DMABUF)
		{
		    for (uint32_t j = 0 ; j < buffer_en->n_planes ; j++)
		    {
			v4l2_buf_en.m.planes[j].bytesused = buffer_en->planes[j].bytesused;
		    }
		}
		// encoder qbuffer for output plane //
		ret = ptr->nv_encoder.ctx.enc->output_plane.qBuffer(v4l2_buf_en, NULL);
		if (ret < 0)
		{
		    cerr << "Error while queueing buffer at output plane" << endl;
		}
}
//add encode
#else
ret = NvBufferTransform(dec_buffer->planes[0].fd, ctx->dst_dma_fd, &transform_params);
#endif
                // Write raw video frame to file
                if (true)
                {
		    fps_counter ++;
		    gettimeofday(&curTime_ts,NULL);
		    long int now = curTime_ts.tv_sec*1000 + curTime_ts.tv_usec/1000; 		
		    if ((diff = now - fps_ms) >= 5000) {
                            float fps = fps_counter / (diff / 1000.0);
                            fps_counter = 0;
                            fps_ms = now;
                            //cout <<"frame_index:" << ptr->decoder_index << endl;
			    if(ptr->decoder_index == 0) cout << "fps_de: "<< fps << endl;
                    }
		    //ptr->isWriteYUV = true;
		    ptr->yuv_read ++;
		    if(ptr->yuv_read >= YUV_NUMS) ptr->yuv_read = 0;

                }

                if (!ctx->stats && !ctx->disable_rendering)
                {
                    ctx->renderer->render(ctx->dst_dma_fd);
                }

                // Not writing to file
                // Queue the buffer back once it has been used.
                if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
                    v4l2_buf.m.planes[0].m.fd = ctx->dmabuff_fd[v4l2_buf.index];
                if (dec->capture_plane.qBuffer(v4l2_buf, NULL) < 0)
                {
                    abort(ctx);
                    cerr <<
                        "Error while queueing buffer at decoder capture plane"
                        << endl;
                    break;
                }

            }
            else
            {
                // Not writing to file
                // Queue the buffer back once it has been used.
                if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
                    v4l2_buf.m.planes[0].m.fd = ctx->dmabuff_fd[v4l2_buf.index];
                if (dec->capture_plane.qBuffer(v4l2_buf, NULL) < 0)
                {
                    abort(ctx);
                    cerr <<
                        "Error while queueing buffer at decoder capture plane"
                        << endl;
                    break;
                }
            }
        }
    }
    cout << "Exiting decoder capture loop thread" << endl;
    return NULL;
}

static void
set_defaults(context_de * ctx)
{
    memset(ctx, 0, sizeof(context_de));
    ctx->fullscreen = false;
    ctx->window_height = 0;
    ctx->window_width = 0;
    ctx->window_x = 0;
    ctx->window_y = 0;
    ctx->out_pixfmt = 2;
    ctx->fps = 25;
    ctx->output_plane_mem_type = V4L2_MEMORY_MMAP;
    ctx->capture_plane_mem_type = V4L2_MEMORY_DMABUF;
    ctx->vp9_file_header_flag = 0;
    ctx->vp8_file_header_flag = 0;
    ctx->stress_test = 1;
    ctx->copy_timestamp = false;
    ctx->flag_copyts = false;
    ctx->start_ts = 0;
    ctx->file_count = 1;
    ctx->dec_fps = 25;
    ctx->dst_dma_fd = -1;
    ctx->bLoop = false;
    ctx->bQueue = false;
    ctx->loop_count = 0;
    ctx->max_perf = 0;
    ctx->extra_cap_plane_buffer = 1;
    ctx->blocking_mode = 1;
    ctx->decoder_pixfmt = V4L2_PIX_FMT_H264;
    ctx->input_nalu = true;
    ctx->disable_rendering = true;
    pthread_mutex_init(&ctx->queue_lock, NULL);
    pthread_cond_init(&ctx->queue_cond, NULL);
    
}

int NVDecoder::decode_proc(int dec_index,int width, int height,string url)
{
    int ret = 0;
    int error = 0;
    uint32_t current_file = 0;
    uint32_t i=0;
    bool eos = false;
    yuv_width = width;
    yuv_height = height;
    //pub_w = pub_width;
    //pub_h = pub_height;
    decoder_index = dec_index;
    set_defaults(&ctx);

    pthread_setname_np(pthread_self(), "DecOutPlane");

    if (ctx.blocking_mode)
    {
        cout << "Creating decoder in blocking mode \n";
        ctx.dec = NvVideoDecoder::createVideoDecoder("dec0");
    }
    else
    {
        cout << "Creating decoder in non-blocking mode \n";
        ctx.dec = NvVideoDecoder::createVideoDecoder("dec0", O_NONBLOCK);
    }
    TEST_ERROR(!ctx.dec, "Could not create decoder", cleanup);

    // Subscribe to Resolution change event
    ret = ctx.dec->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0);
    TEST_ERROR(ret < 0, "Could not subscribe to V4L2_EVENT_RESOLUTION_CHANGE",
               cleanup);

    // Set format on the output plane
    ret = ctx.dec->setOutputPlaneFormat(ctx.decoder_pixfmt, CHUNK_SIZE);
    TEST_ERROR(ret < 0, "Could not set output plane format", cleanup);

    if (ctx.input_nalu)
    {
	//for(int i= 0;i<YUV_NUMS;i++) yuv_datas[i] = new unsigned char[pub_h*pub_w*3/2];
        printf("Setting frame input mode to 0 \n");
        ret = ctx.dec->setFrameInputMode(0);
        TEST_ERROR(ret < 0,
                "Error in decoder setFrameInputMode", cleanup);
    }

    // V4L2_CID_MPEG_VIDEO_DISABLE_DPB should be set after output plane
    // set format

    // Query, Export and Map the output plane buffers so that we can read
    // encoded data into the buffers
    if (ctx.output_plane_mem_type == V4L2_MEMORY_MMAP) {
        ret = ctx.dec->output_plane.setupPlane(V4L2_MEMORY_MMAP, 2, true, false);
    } else if (ctx.output_plane_mem_type == V4L2_MEMORY_USERPTR) {
        ret = ctx.dec->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10, false, true);
    }

    TEST_ERROR(ret < 0, "Error while setting up output plane", cleanup);

    ret = ctx.dec->output_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in output plane stream on", cleanup);

    cout << "ctx.dec->output_plane.getNumBuffers():" << ctx.dec->output_plane.getNumBuffers() << endl;
    /*while (!eos && !ctx.got_error && !ctx.dec->isInError() &&
           i < ctx.dec->output_plane.getNumBuffers())
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer;

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        buffer = ctx.dec->output_plane.getNthBuffer(i);
        if ((ctx.decoder_pixfmt == V4L2_PIX_FMT_H264) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_H265) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_MPEG2) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_MPEG4))
        {
            if (ctx.input_nalu)
            {
		char *buffer_ptr = (char *) buffer->planes[0].data;
		unsigned char nalu[4] = {0x00,0x00,0x00,0x01};
    		memcpy(buffer_ptr,nalu,4);
		buffer->planes[0].bytesused = 4;
            }

        }
        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.m.planes[0].bytesused = buffer->planes[0].bytesused;
	//v4l2_buf.m.planes[0].bytesused = 4000000;
        // It is necessary to queue an empty buffer to signal EOS to the decoder
        // i.e. set v4l2_buf.m.planes[0].bytesused = 0 and queue the buffer
        ret = ctx.dec->output_plane.qBuffer(v4l2_buf, NULL);
        i++;
	//return 0;
    }*/
    //cout << "ctx.blocking_mode: " << ctx.blocking_mode << endl;
    if (ctx.blocking_mode)
    {
        pthread_create(&ctx.dec_capture_loop, NULL, dec_capture_loop_fcn, (void*)this);
        pthread_setname_np(ctx.dec_capture_loop, "DecCapPlane");
    }
    //cout << "return" << endl;
#ifdef OPEN_ENCODER
    if(decoder_index < ENCODE_NUMS) nv_encoder.Encode_proc(url, width, height);
#endif
    return 0;
cleanup:
    if (ctx.blocking_mode && ctx.dec_capture_loop)
    {
        pthread_join(ctx.dec_capture_loop, NULL);
    }
    else if (!ctx.blocking_mode)
    {
        // Clear the poll interrupt to get the decoder's poll thread out.
        ctx.dec->ClearPollInterrupt();
        // If Pollthread is waiting on, signal it to exit the thread.
        sem_post(&ctx.pollthread_sema);
        pthread_join(ctx.dec_pollthread, NULL);
    }
    if(ctx.capture_plane_mem_type == V4L2_MEMORY_DMABUF)
    {
        for(int index = 0 ; index < ctx.numCapBuffers ; index++)
        {
            if(ctx.dmabuff_fd[index] != 0)
            {
                ret = NvBufferDestroy (ctx.dmabuff_fd[index]);
                if(ret < 0)
                {
                    cerr << "Failed to Destroy NvBuffer" << endl;
                }
            }
        }
    }

    if (ctx.dec && ctx.dec->isInError())
    {
        cerr << "Decoder is in error" << endl;
        error = 1;
    }

    if (ctx.got_error)
    {
        error = 1;
    }

    // The decoder destructor does all the cleanup i.e set streamoff on output and capture planes,
    // unmap buffers, tell decoder to deallocate buffer (reqbufs ioctl with counnt = 0),
    // and finally call v4l2_close on the fd.
    delete ctx.dec;

    // Similarly, EglRenderer destructor does all the cleanup
    delete ctx.renderer;

    if(ctx.dst_dma_fd != -1)
    {
        NvBufferDestroy(ctx.dst_dma_fd);
        ctx.dst_dma_fd = -1;
    }

    //for(int i= 0;i<YUV_NUMS;i++) delete[] yuv_datas[i];
    if (!ctx.blocking_mode)
    {
        sem_destroy(&ctx.pollthread_sema);
        sem_destroy(&ctx.decoderthread_sema);
    }

    return -error;
}

int NVDecoder::decode_h264(unsigned char *h264_data, int frameSize)
{
    // Since all the output plane buffers have been queued, we first need to
    // dequeue a buffer from output plane before we can read new data into it
    // and queue it again.
    int ret = 0;
    int allow_DQ = true;
    struct v4l2_buffer temp_buf;

    if (output_plane_index < ctx.dec->output_plane.getNumBuffers())
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer;

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        buffer = ctx.dec->output_plane.getNthBuffer(output_plane_index);
        if ((ctx.decoder_pixfmt == V4L2_PIX_FMT_H264) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_H265) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_MPEG2) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_MPEG4))
        {
            if (ctx.input_nalu)
            {
		char *buffer_ptr = (char *) buffer->planes[0].data;
		unsigned char nalu[4] = {0x00,0x00,0x00,0x01};
    		memcpy(buffer_ptr,nalu,4);
    		memcpy(buffer_ptr+4, h264_data, frameSize);
		buffer->planes[0].bytesused = 4 + frameSize;
            }

        }
        v4l2_buf.index = output_plane_index;
        v4l2_buf.m.planes = planes;
        v4l2_buf.m.planes[0].bytesused = buffer->planes[0].bytesused;
	//v4l2_buf.m.planes[0].bytesused = 4000000;
        // It is necessary to queue an empty buffer to signal EOS to the decoder
        // i.e. set v4l2_buf.m.planes[0].bytesused = 0 and queue the buffer
        ret = ctx.dec->output_plane.qBuffer(v4l2_buf, NULL);
        output_plane_index ++;
    }

    else//while (!ctx.got_error && !ctx.dec->isInError())
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer;

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;

        if(allow_DQ)
        {
            ret = ctx.dec->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, -1);
            if (ret < 0)
            {
                cerr << "Error DQing buffer at output plane" << endl;
                abort(&ctx);
                //break;
            }
        }
        else
        {
            allow_DQ = true;
            memcpy(&v4l2_buf,&temp_buf,sizeof(v4l2_buffer));
            buffer = ctx.dec->output_plane.getNthBuffer(v4l2_buf.index);
        }

        if ((ctx.decoder_pixfmt == V4L2_PIX_FMT_H264) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_H265) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_MPEG2) ||
                (ctx.decoder_pixfmt == V4L2_PIX_FMT_MPEG4))
        {
            if (ctx.input_nalu)
            {
		char *buffer_ptr = (char *) buffer->planes[0].data;
		unsigned char nalu[4] = {0x00,0x00,0x00,0x01};
    		memcpy(buffer_ptr,nalu,4);
    		memcpy(buffer_ptr+4, h264_data, frameSize);
		buffer->planes[0].bytesused = 4 + frameSize;
            }
        }
        v4l2_buf.m.planes[0].bytesused = buffer->planes[0].bytesused;
	//cout << "v4l2_buf.m.planes[0].bytesused:" << v4l2_buf.m.planes[0].bytesused << endl;

        ret = ctx.dec->output_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error Qing buffer at output plane" << endl;
            abort(&ctx);
            //break;
        }
        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            //eos = true;
            cout << "Input file read complete2" << endl;
            //break;
        }
    }
    return 0;
}


/*unsigned char *NVDecoder::getYUVData()
{
	//cout << "yuv_read:" << yuv_read << endl;
	if(yuv_read==0) return yuv_datas[YUV_NUMS - 1];
	return yuv_datas[yuv_read - 1];
}*/

