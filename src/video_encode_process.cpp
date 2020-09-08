#include "video_encode_process.h"
#include "../SpsDecode.h"
#define OPEN_RTMP 1
NVEncoder::NVEncoder()
{
}

NVEncoder::~NVEncoder()
{
}

/**
  * Abort on error.
  *
  * @param ctx : Encoder context
  */
static void
abort(context_en *ctx)
{
    ctx->got_error = true;
    ctx->enc->abort();
}


/**
  * Encoder capture-plane deque buffer callback function.
  *
  * @param v4l2_buf      : v4l2 buffer
  * @param buffer        : NvBuffer
  * @param shared_buffer : shared NvBuffer
  * @param arg           : context pointer
  */
bool
NVEncoder::encoder_capture_plane_dq_callback(struct v4l2_buffer *v4l2_buf, NvBuffer * buffer,
                                  NvBuffer * shared_buffer, void *arg)
{
    //context_en *ctx = (context_en *) arg;
    NVEncoder *ptr = (NVEncoder *)arg;
    context_en *ctx = &(ptr->ctx);
    NvVideoEncoder *enc = ctx->enc;
    pthread_setname_np(pthread_self(), "EncCapPlane");
    uint32_t frame_num = ctx->enc->capture_plane.getTotalDequeuedBuffers() - 1;
    uint32_t ReconRef_Y_CRC = 0;
    uint32_t ReconRef_U_CRC = 0;
    uint32_t ReconRef_V_CRC = 0;
    static uint32_t num_encoded_frames = 1;
    struct v4l2_event ev;
    int ret = 0;

    if (v4l2_buf == NULL)
    {
        cout << "Error while dequeing buffer from output plane" << endl;
        abort(ctx);
        return false;
    }

    /* Received EOS from encoder. Stop dqthread. */
    if (buffer->planes[0].bytesused == 0)
    {
        cout << "Got 0 size buffer in capture \n";
        return false;
    }

    if (!ctx->stats)
    {
#ifdef OPEN_RTMP
	int len = buffer->planes[0].bytesused;
	//cout << "h264 len:" << len << endl;
	if(ptr->is_h264_head)
	{
		unsigned char *pH264_head = (unsigned char *) buffer->planes[0].data;

		for(int i=0;i<10;i++)
			printf("%x", pH264_head[i]);
		printf("\n");
		RTMPMetadata metaData;
		memset(&metaData,0,sizeof(RTMPMetadata));
		NaluUnit naluUnit;
		ptr->rtmpSender.ReadOneNaluFromBuf(naluUnit,(unsigned char *) buffer->planes[0].data,len);

		metaData.nSpsLen = naluUnit.size;
		memcpy(metaData.Sps,naluUnit.data,naluUnit.size);
		//for(int i=0;i<naluUnit.size;i++)
		//printf("%x", metaData.Sps[i]);
		//printf("\n");
		//printf("读取SPS帧\n");
		ptr->rtmpSender.ReadOneNaluFromBuf(naluUnit,(unsigned char *) buffer->planes[0].data,len);
		metaData.nPpsLen = naluUnit.size;
		memcpy(metaData.Pps,naluUnit.data,naluUnit.size);
		//for(int i=0;i<naluUnit.size;i++)
		//printf("%x", metaData.Pps[i]);
		//printf("\n");
		//printf("读取PPS帧\n");
		int width = 1280,height = 720;
    		h264_decode_sps(metaData.Sps,metaData.nSpsLen,width,height);
		metaData.nWidth = width;
       		metaData.nHeight = height;
		//metaData.nWidth = ptr->rtmp_width;
		//metaData.nHeight = ptr->rtmp_height;
		metaData.nFrameRate = 25;
		printf("解码SPS,width=%d,height=%d\n",width,height);
		// 发送MetaData
		ptr->rtmpSender.SendMetadata(&metaData);
		printf("发送MetaData\n");
		ptr->is_h264_head = false;
	}
	else
	{
		//for(int i=0;i<10;i++)
		//	printf("%x", (unsigned char *) buffer->planes[0].data[i]);
		NaluUnit naluUnit;
		naluUnit.size = len - 4;
		bool flag = ptr->rtmpSender.ReadOneNaluFromBuf(naluUnit,(unsigned char *) buffer->planes[0].data);
		//printf("flag = %d\n",flag);
		bool bKeyframe  = (naluUnit.type == 0x05) ? TRUE : FALSE;
		if(bKeyframe)
		{
			ptr->is_key_frame =  true;
			ptr->rtmpSender.SendH264Packet(naluUnit.data,naluUnit.size,bKeyframe,ptr->tick);	
			ptr->tick += 40;
		}
		else if(ptr->is_key_frame)
		{
			ptr->rtmpSender.SendH264Packet(naluUnit.data,naluUnit.size,bKeyframe,ptr->tick);
			ptr->tick += 40;
		}
		//printf("bkeyframe=%d\n",bKeyframe);
		// 发送H264数据帧
		
	}
#endif
    }
    //write_encoder_output_frame(buffer);

    num_encoded_frames++;

    /* encoder qbuffer for capture plane */
    if (enc->capture_plane.qBuffer(*v4l2_buf, NULL) < 0)
    {
        cerr << "Error while Qing buffer at capture plane" << endl;
        abort(ctx);
        return false;
    }
    //cout << "return true" << endl;
    return true;
}

/**
  * Set encoder context defaults values.
  *
  * @param ctx : Encoder context
  */
static void
set_defaults(context_en * ctx)
{
    memset(ctx, 0, sizeof(context_en));

    ctx->raw_pixfmt = V4L2_PIX_FMT_YUV420M;
    ctx->bitrate = 1024 * 1024;
    ctx->peak_bitrate = 0;
    ctx->profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
    ctx->ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
    ctx->iframe_interval = 25;
    ctx->externalRPS = false;
    ctx->enableGDR = false;
    ctx->enableROI = false;
    ctx->bnoIframe = false;
    ctx->bGapsInFrameNumAllowed = false;
    ctx->bReconCrc = false;
    ctx->enableLossless = false;
    ctx->nH264FrameNumBits = 0;
    ctx->nH265PocLsbBits = 0;
    ctx->idr_interval = 50;
    ctx->level = -1;
    ctx->fps_n = 25;
    ctx->fps_d = 1;
    ctx->gdr_start_frame_number = 0xffffffff;
    ctx->gdr_num_frames = 0xffffffff;
    ctx->gdr_out_frame_number = 0xffffffff;
    ctx->num_b_frames = (uint32_t) -1;
    ctx->nMinQpI = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpI = (uint32_t)QP_RETAIN_VAL;
    ctx->nMinQpP = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpP = (uint32_t)QP_RETAIN_VAL;
    ctx->nMinQpB = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpB = (uint32_t)QP_RETAIN_VAL;
    ctx->use_gold_crc = false;
    ctx->externalRCHints = false;
    ctx->input_metadata = false;
    ctx->sMaxQp = 51;
    ctx->stats = false;
    ctx->stress_test = 1;
    ctx->output_memory_type = V4L2_MEMORY_DMABUF;
    ctx->copy_timestamp = false;
    ctx->start_ts = 0;
    ctx->max_perf = 0;
    ctx->blocking_mode = 1;
    ctx->startf = 0;
    ctx->endf = 0;
    ctx->num_output_buffers = 6;
    ctx->num_frames_to_encode = -1;
    ctx->encoder_pixfmt = V4L2_PIX_FMT_H264;
    //ctx->alliframes = true;
}

/**
  * Setup output plane for DMABUF io-mode.
  *
  * @param ctx         : encoder context
  * @param num_buffers : request buffer count
  */
static int
setup_output_dmabuf(context_en *ctx, uint32_t num_buffers )
{
    int ret=0;
    NvBufferCreateParams cParams;
    int fd;
    ret = ctx->enc->output_plane.reqbufs(V4L2_MEMORY_DMABUF,num_buffers);
    if(ret)
    {
        cerr << "reqbufs failed for output plane V4L2_MEMORY_DMABUF" << endl;
        return ret;
    }
    for (uint32_t i = 0; i < ctx->enc->output_plane.getNumBuffers(); i++)
    {
        cParams.width = ctx->width;
        cParams.height = ctx->height;
        cParams.layout = NvBufferLayout_Pitch;
        if (ctx->enableLossless && ctx->encoder_pixfmt == V4L2_PIX_FMT_H264)
        {
            cParams.colorFormat = NvBufferColorFormat_YUV444;
        }
        else if (ctx->profile == V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10)
        {
            cParams.colorFormat = NvBufferColorFormat_NV12_10LE;
        }
        else
        {
            cParams.colorFormat = ctx->enable_extended_colorformat ?
                 NvBufferColorFormat_YUV420_ER : NvBufferColorFormat_YUV420;
        }
        cParams.nvbuf_tag = NvBufferTag_VIDEO_ENC;
        cParams.payloadType = NvBufferPayload_SurfArray;
        /* Create output plane fd for DMABUF io-mode */
        ret = NvBufferCreateEx(&fd, &cParams);
        if(ret < 0)
        {
            cerr << "Failed to create NvBuffer" << endl;
            return ret;
        }
        ctx->output_plane_fd[i]=fd;
    }
    return ret;
}

/**
  * Encode processing function for blocking mode.
  *
  * @param ctx : Encoder context
  * @param eos : end of stream
  */
/*int NVEncoder::encoder_proc_blocking(context_en &ctx, NVDecoder& ndecoder)
{
    int ret = 0;
    bool eos = false;
    cout << "encoder_proc_blocking" << endl;
    long int fps_ms;
	long int diff;
	int fps_counter;
	float fps = 0.0;
	struct timeval curTime_tv;
	struct timeval curTime_ts;
	gettimeofday(&curTime_tv,NULL);
	fps_ms = curTime_tv.tv_sec*1000 + curTime_tv.tv_usec/1000;
    // Keep reading input till EOS is reached 
    while (!ctx.got_error && !ctx.enc->isInError() && !eos)
    {
	if(!ndecoder.isWriteYUV) 
	{
		usleep(1000);
		continue;
	}
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer;

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;

        //dequeue buffer from encoder output plane //
        if (ctx.enc->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0)
        {
            cerr << "ERROR while DQing buffer at output plane" << endl;
            abort(&ctx);
            goto cleanup;
        }

	uint32_t k, j;
	char *data;
	unsigned char *yuv_ptr;
	yuv_ptr = ndecoder.getYUVData();

	for (k = 0; k < buffer->n_planes; k++)
	{
	NvBuffer::NvBufferPlane &plane = buffer->planes[k];
	std::streamsize bytes_to_read =
	    plane.fmt.bytesperpixel * plane.fmt.width;
	data = (char *) plane.data;
	plane.bytesused = 0;
	for (j = 0; j < plane.fmt.height; j++)
	{
	    memcpy(data, yuv_ptr,bytes_to_read);
	    yuv_ptr += bytes_to_read;
	    data += plane.fmt.stride;
	}
	plane.bytesused = plane.fmt.stride * plane.fmt.height;
	//cout << "kk...=" << k << endl;
	//cout << "bytes_to_read...=" << bytes_to_read << endl;
	//cout << "plane.fmt.stride...=" << plane.fmt.stride << endl;
	//cout << "plane.bytesused...=" << plane.bytesused << endl;
	}
	ndecoder.isWriteYUV = false;
        fps_counter ++;
	gettimeofday(&curTime_ts,NULL);
	long int now = curTime_ts.tv_sec*1000 + curTime_ts.tv_usec/1000; 		
	if ((diff = now - fps_ms) >= 5000) {
	    float fps = fps_counter / (diff / 1000.0);
	    fps_counter = 0;
	    fps_ms = now;
	    //cout <<"frame_index:" << ptr->decoder_index << endl;
	    if(ndecoder.decoder_index == 0) cout << "fps_en: "<< fps << endl;
	}

        if(ctx.output_memory_type == V4L2_MEMORY_DMABUF || ctx.output_memory_type == V4L2_MEMORY_MMAP)
        {
            for (uint32_t j = 0 ; j < buffer->n_planes ; j++)
            {
                ret = NvBufferMemSyncForDevice (buffer->planes[j].fd, j, (void **)&buffer->planes[j].data);
                if (ret < 0)
                {
                    cerr << "Error while NvBufferMemSyncForDevice at output plane for V4L2_MEMORY_DMABUF" << endl;
                    abort(&ctx);
                    goto cleanup;
                }
            }
        }

        if(ctx.output_memory_type == V4L2_MEMORY_DMABUF)
        {
            for (uint32_t j = 0 ; j < buffer->n_planes ; j++)
            {
                v4l2_buf.m.planes[j].bytesused = buffer->planes[j].bytesused;
            }
        }
        // encoder qbuffer for output plane //
        ret = ctx.enc->output_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error while queueing buffer at output plane" << endl;
            abort(&ctx);
            goto cleanup;
        }

        ctx.input_frames_queued_count++;
        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            cerr << "File read complete." << endl;
            eos = true;
            ctx.got_eos = true;
            return 0;
        }
    }
cleanup:
    return -1;
}*/

/**
  * Encode processing function.
  *
  * @param ctx  : Encoder context
  * @param argc : Argument Count
  * @param argv : Argument Vector
  */
int NVEncoder::Encode_proc(string rtmp_url,int yuv_width,int yuv_height)
{
    int ret = 0;
    int error = 0;
    bool eos = false;
    //cout << "encode proc begin." << endl;
    //cout << "ndecoder.yuv_width:" << ndecoder.yuv_width << endl;
    rtmp_width = yuv_width;
    rtmp_height = yuv_height;
    cout << "rtmp_height:" << rtmp_height << endl;
#ifdef OPEN_RTMP
    while(1)
    {
    	bool bRet = rtmpSender.Connect(rtmp_url.c_str());
	if(bRet)
	{
		cout << "connect: " << rtmp_url.c_str() << endl;
		break;
	}
	else
	{
		cout << "!!!can not connect: " << rtmp_url.c_str() << endl;
		sleep(1);
	}
    }
#endif
    /* Set default values for encoder context members. */
    set_defaults(&ctx);
    ctx.width = rtmp_width;
    ctx.height = rtmp_height;
   
    /* Set thread name for encoder Output Plane thread. */
    pthread_setname_np(pthread_self(),"EncOutPlane");

    if (ctx.encoder_pixfmt == V4L2_PIX_FMT_H265)
    {
        TEST_ERROR(ctx.width < 144 || ctx.height < 144, "Height/Width should be"
            " > 144 for H.265", cleanup);
    }

    /* Create NvVideoEncoder object for blocking or non-blocking I/O mode. */
    if (ctx.blocking_mode)
    {
        cout << "Creating Encoder in blocking mode \n";
        ctx.enc = NvVideoEncoder::createVideoEncoder("enc0");
    }
    TEST_ERROR(!ctx.enc, "Could not create encoder", cleanup);

    /* Set encoder capture plane format.
       NOTE: It is necessary that Capture Plane format be set before Output Plane
       format. It is necessary to set width and height on the capture plane as well */
    ret =
        ctx.enc->setCapturePlaneFormat(ctx.encoder_pixfmt, ctx.width,
                                      ctx.height, 2 * 1024 * 1024);
    TEST_ERROR(ret < 0, "Could not set capture plane format", cleanup);

    switch (ctx.profile)
    {
        case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10:
            ctx.raw_pixfmt = V4L2_PIX_FMT_P010M;
            break;
        case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN:
        default:
            ctx.raw_pixfmt = V4L2_PIX_FMT_YUV420M;
    }

    /* Set encoder output plane format */
    {
        ret =
            ctx.enc->setOutputPlaneFormat(ctx.raw_pixfmt, ctx.width,
                                      ctx.height);
    }
    TEST_ERROR(ret < 0, "Could not set output plane format", cleanup);

    ret = ctx.enc->setBitrate(ctx.bitrate);
    TEST_ERROR(ret < 0, "Could not set encoder bitrate", cleanup);

    if (ctx.encoder_pixfmt == V4L2_PIX_FMT_H264)
    {
        /* Set encoder profile for H264 format */
        ret = ctx.enc->setProfile(ctx.profile);
        TEST_ERROR(ret < 0, "Could not set encoder profile", cleanup);

        if (ctx.level == (uint32_t)-1)
        {
            ctx.level = (uint32_t)V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
        }

        /* Set encoder level for H264 format */
        ret = ctx.enc->setLevel(ctx.level);
        TEST_ERROR(ret < 0, "Could not set encoder level", cleanup);
    }
    else if (ctx.encoder_pixfmt == V4L2_PIX_FMT_H265)
    {
        /* Set encoder profile for HEVC format */
        ret = ctx.enc->setProfile(ctx.profile);
        TEST_ERROR(ret < 0, "Could not set encoder profile", cleanup);

        if (ctx.level != (uint32_t)-1)
        {
            /* Set encoder level for HEVC format */
            ret = ctx.enc->setLevel(ctx.level);
            TEST_ERROR(ret < 0, "Could not set encoder level", cleanup);
        }
    }

    {
        /* Set rate control mode for encoder */
        ret = ctx.enc->setRateControlMode(ctx.ratecontrol);
        TEST_ERROR(ret < 0, "Could not set encoder rate control mode", cleanup);
        if (ctx.ratecontrol == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
            uint32_t peak_bitrate;
            if (ctx.peak_bitrate < ctx.bitrate)
                peak_bitrate = 1.2f * ctx.bitrate;
            else
                peak_bitrate = ctx.peak_bitrate;
            /* Set peak bitrate value for variable bitrate mode for encoder */
            ret = ctx.enc->setPeakBitrate(peak_bitrate);
            TEST_ERROR(ret < 0, "Could not set encoder peak bitrate", cleanup);
        }
    }

    /* Set IDR frame interval for encoder */
    ret = ctx.enc->setIDRInterval(ctx.idr_interval);
    TEST_ERROR(ret < 0, "Could not set encoder IDR interval", cleanup);

    /* Set I frame interval for encoder */
    ret = ctx.enc->setIFrameInterval(ctx.iframe_interval);
    TEST_ERROR(ret < 0, "Could not set encoder I-Frame interval", cleanup);

    /* Set framerate for encoder */
    ret = ctx.enc->setFrameRate(ctx.fps_n, ctx.fps_d);
    TEST_ERROR(ret < 0, "Could not set framerate", cleanup);

    if (ctx.alliframes)
    {
        /* Enable all I-frame encode */
        ret = ctx.enc->setAlliFramesEncode(true);
        TEST_ERROR(ret < 0, "Could not set Alliframes encoding", cleanup);
    }

    if (ctx.num_b_frames != (uint32_t) -1)
    {
        /* Set number of B-frames to to be used by encoder */
        ret = ctx.enc->setNumBFrames(ctx.num_b_frames);
        TEST_ERROR(ret < 0, "Could not set number of B Frames", cleanup);
    }

    if ((ctx.nMinQpI != (uint32_t)QP_RETAIN_VAL) ||
        (ctx.nMaxQpI != (uint32_t)QP_RETAIN_VAL) ||
        (ctx.nMinQpP != (uint32_t)QP_RETAIN_VAL) ||
        (ctx.nMaxQpP != (uint32_t)QP_RETAIN_VAL) ||
        (ctx.nMinQpB != (uint32_t)QP_RETAIN_VAL) ||
        (ctx.nMaxQpB != (uint32_t)QP_RETAIN_VAL))
    {
        /* Set Min & Max qp range values for I/P/B-frames to be used by encoder */
        ret = ctx.enc->setQpRange(ctx.nMinQpI, ctx.nMaxQpI, ctx.nMinQpP,
                ctx.nMaxQpP, ctx.nMinQpB, ctx.nMaxQpB);
        TEST_ERROR(ret < 0, "Could not set quantization parameters", cleanup);
    }

    /* Query, Export and Map the output plane buffers so that we can read
       raw data into the buffers */
    switch(ctx.output_memory_type)
    {
        case V4L2_MEMORY_MMAP:
            ret = ctx.enc->output_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true, false);
            TEST_ERROR(ret < 0, "Could not setup output plane", cleanup);
            break;

        case V4L2_MEMORY_USERPTR:
            ret = ctx.enc->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10, false, true);
            TEST_ERROR(ret < 0, "Could not setup output plane", cleanup);
            break;

        case V4L2_MEMORY_DMABUF:
            ret = setup_output_dmabuf(&ctx,10);
            TEST_ERROR(ret < 0, "Could not setup plane", cleanup);
            break;
        default :
            TEST_ERROR(true, "Not a valid plane", cleanup);
    }

    /* Query, Export and Map the capture plane buffers so that we can write
       encoded bitstream data into the buffers */
    ret = ctx.enc->capture_plane.setupPlane(V4L2_MEMORY_MMAP, ctx.num_output_buffers,
        true, false);
    TEST_ERROR(ret < 0, "Could not setup capture plane", cleanup);

    /* Subscibe for End Of Stream event */
    ret = ctx.enc->subscribeEvent(V4L2_EVENT_EOS,0,0);
    TEST_ERROR(ret < 0, "Could not subscribe EOS event", cleanup);

    {
        /* set encoder output plane STREAMON */
        ret = ctx.enc->output_plane.setStreamStatus(true);
        TEST_ERROR(ret < 0, "Error in output plane streamon", cleanup);

        /* set encoder capture plane STREAMON */
        ret = ctx.enc->capture_plane.setStreamStatus(true);
        TEST_ERROR(ret < 0, "Error in capture plane streamon", cleanup);
    }

    cout << "capture_plane.getNumBuffers(): " << ctx.enc->capture_plane.getNumBuffers() << endl;
    /* Enqueue all the empty capture plane buffers. */
    for (uint32_t i = 0; i < ctx.enc->capture_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        ret = ctx.enc->capture_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error while queueing buffer at capture plane" << endl;
            abort(&ctx);
            goto cleanup;
        }
    }

    cout << "output_plane.getNumBuffers(): " << ctx.enc->output_plane.getNumBuffers() << endl;
    /* Read video frame and queue all the output plane buffers. */
    for (uint32_t i = 0; i < ctx.enc->output_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer = ctx.enc->output_plane.getNthBuffer(i);

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        if(ctx.output_memory_type == V4L2_MEMORY_DMABUF)
        {
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            v4l2_buf.memory = V4L2_MEMORY_DMABUF;
            /* Map output plane buffer for memory type DMABUF. */
            ret = ctx.enc->output_plane.mapOutputBuffers(v4l2_buf, ctx.output_plane_fd[i]);

            if (ret < 0)
            {
                cerr << "Error while mapping buffer at output plane" << endl;
                abort(&ctx);
                goto cleanup;
            }
        }
        /* Read yuv frame data from input file */
        uint32_t k, j;
	char *data;

	for (k = 0; k < buffer->n_planes; k++)
	{
		NvBuffer::NvBufferPlane &plane = buffer->planes[k];
		std::streamsize bytes_to_read =
		    plane.fmt.bytesperpixel * plane.fmt.width;
		data = (char *) plane.data;
		plane.bytesused = 0;
		for (j = 0; j < plane.fmt.height; j++)
		{
		    //memcpy(data, data,bytes_to_read);
		    data += plane.fmt.stride;
		}
		plane.bytesused = plane.fmt.stride * plane.fmt.height;
	}

        if(ctx.output_memory_type == V4L2_MEMORY_DMABUF || ctx.output_memory_type == V4L2_MEMORY_MMAP)
        {
            for (uint32_t j = 0 ; j < buffer->n_planes; j++)
            {
                ret = NvBufferMemSyncForDevice (buffer->planes[j].fd, j, (void **)&buffer->planes[j].data);
                if (ret < 0)
                {
                    cerr << "Error while NvBufferMemSyncForDevice at output plane for V4L2_MEMORY_DMABUF" << endl;
                    abort(&ctx);
                    goto cleanup;
                }
            }
        }

        if(ctx.output_memory_type == V4L2_MEMORY_DMABUF)
        {
            for (uint32_t j = 0 ; j < buffer->n_planes ; j++)
            {
                v4l2_buf.m.planes[j].bytesused = buffer->planes[j].bytesused;
            }
        }
        /* encoder qbuffer for output plane */
        ret = ctx.enc->output_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error while queueing buffer at output plane" << endl;
            abort(&ctx);
            goto cleanup;
        }
        
        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            cerr << "File read complete." << endl;
            eos = true;
            break;
        }
        ctx.input_frames_queued_count++;
    }

    if (ctx.blocking_mode)
    {
        /* Set encoder capture plane dq thread callback for blocking io mode */
        ctx.enc->capture_plane.
            setDQThreadCallback(encoder_capture_plane_dq_callback);

        /* startDQThread starts a thread internally which calls the
           encoder_capture_plane_dq_callback whenever a buffer is dequeued
           on the plane */
        //ctx.enc->capture_plane.startDQThread(&ctx);
	ctx.enc->capture_plane.startDQThread((void*)this);
    }
    //cout << "init encoder_capture_plane_dq_callback." << endl;
    if (ctx.blocking_mode)
    {
        /* Wait till capture plane DQ Thread finishes
           i.e. all the capture plane buffers are dequeued. */
        //if (encoder_proc_blocking(ctx, ndecoder) != 0)
        //    goto cleanup;
        ctx.enc->capture_plane.waitForDQThread(-1);
    }
    //cout << "encoder return" << endl;
    if (ctx.stats)
    {
        ctx.enc->printProfilingStats(cout);
    }
    return 0;
cleanup:
    if (ctx.enc && ctx.enc->isInError())
    {
        cerr << "Encoder is in error" << endl;
        error = 1;
    }
    if (ctx.got_error)
    {
        error = 1;
    }

    if(ctx.output_memory_type == V4L2_MEMORY_DMABUF)
    {
        for (uint32_t i = 0; i < ctx.enc->output_plane.getNumBuffers(); i++)
        {
            /* Unmap output plane buffer for memory type DMABUF. */
            ret = ctx.enc->output_plane.unmapOutputBuffers(i, ctx.output_plane_fd[i]);
            if (ret < 0)
            {
                cerr << "Error while unmapping buffer at output plane" << endl;
                goto cleanup;
            }

            ret = NvBufferDestroy(ctx.output_plane_fd[i]);
            if(ret < 0)
            {
                cerr << "Failed to Destroy NvBuffer\n" << endl;
                return ret;
            }
        }
    }

    /* Release encoder configuration specific resources. */
    //delete ctx.enc;
    //delete ctx.in_file;

    //free(ctx.in_file_path);
    //free(ctx.out_file_path);

    if (!ctx.blocking_mode)
    {
        sem_destroy(&ctx.pollthread_sema);
        sem_destroy(&ctx.encoderthread_sema);
    }
    return -error;
}
