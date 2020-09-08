cmake_minimum_required(VERSION 2.8.4)
project(testRTSPClient)
project(thread_close)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER "gcc")
#set(COLLECT_GCC_OPTIONS = 'v' '-march = armv7-a' '-mfloat-abi=hard' '-mfpu=vfpv3-d16' '-mthumb' -mfloat-abi=hard)
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_C_FLAGS "-pipe -O2 -Wall -W -fPIC")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DSOCKLEN_T=socklen_t -D_LARGEFILE_SOURCE=1 -D_FILE_OFFSET_BITS=64 -DALLOW_RTSP_SERVER_PORT_REUSE=1")
set(CMAKE_CXX_STANDARD 11)

include_directories(/usr/include /usr/local/lib ./include ./src ./zmq ./jetson_multimedia_api/include/ /usr/include/libdrm/ ./jetson_multimedia_api/include/libjpeg-8b/ /usr/include/librtmp/
./video_code)

#set(MAIN_FILE
#	testRTSPClient.cpp
#	src/video_decode_process.cpp
#	src/video_encode_process.cpp
#)
set(MAIN_CLOSE_FILE
	testRTSPClient_close.cpp
	video_code/decode_close_process.cpp
	video_code/encode_close_process.cpp
)
set(SOURCE_FILES 
	RTMPStream.cpp
	src/AC3AudioFileServerMediaSubsession.cpp
	src/AC3AudioRTPSink.cpp
	src/AC3AudioRTPSource.cpp
	src/AC3AudioStreamFramer.cpp
	src/ADTSAudioFileServerMediaSubsession.cpp
	src/ADTSAudioFileSource.cpp
	src/AMRAudioFileServerMediaSubsession.cpp
	src/AMRAudioFileSink.cpp
	src/AMRAudioFileSource.cpp
	src/AMRAudioRTPSink.cpp
	src/AMRAudioRTPSource.cpp
	src/AMRAudioSource.cpp
	src/AudioInputDevice.cpp
	src/AudioRTPSink.cpp
	src/AVIFileSink.cpp
	src/Base64.cpp
	src/BasicHashTable.cpp
	src/BasicTaskScheduler.cpp
	src/BasicTaskScheduler0.cpp
	src/BasicUDPSink.cpp
	src/BasicUDPSource.cpp
	src/BasicUsageEnvironment.cpp
	src/BasicUsageEnvironment0.cpp
	src/BitVector.cpp
	src/ByteStreamFileSource.cpp
	src/ByteStreamMemoryBufferSource.cpp
	src/ByteStreamMultiFileSource.cpp
	src/DelayQueue.cpp
	src/DeviceSource.cpp
	src/DigestAuthentication.cpp
	src/DVVideoFileServerMediaSubsession.cpp
	src/DVVideoRTPSink.cpp
	src/DVVideoRTPSource.cpp
	src/DVVideoStreamFramer.cpp
	src/EBMLNumber.cpp
	src/EBMLNumber.hh
	src/FileServerMediaSubsession.cpp
	src/FileSink.cpp
	src/FramedFileSource.cpp
	src/FramedFilter.cpp
	src/FramedSource.cpp
	src/GenericMediaServer.cpp
	src/GroupEId.cpp
	src/Groupsock.cpp
	src/GroupsockHelper.cpp
	src/GSMAudioRTPSink.cpp
	src/H261VideoRTPSource.cpp
	src/H263plusVideoFileServerMediaSubsession.cpp
	src/H263plusVideoRTPSink.cpp
	src/H263plusVideoRTPSource.cpp
	src/H263plusVideoStreamFramer.cpp
	src/H263plusVideoStreamParser.cpp
	src/H263plusVideoStreamParser.hh
	src/H264or5VideoFileSink.cpp
	src/H264or5VideoRTPSink.cpp
	src/H264or5VideoStreamDiscreteFramer.cpp
	src/H264or5VideoStreamFramer.cpp
	src/H264VideoFileServerMediaSubsession.cpp
	src/H264VideoFileSink.cpp
	src/H264VideoRTPSink.cpp
	src/H264VideoRTPSource.cpp
	src/H264VideoStreamDiscreteFramer.cpp
	src/H264VideoStreamFramer.cpp
	src/H265VideoFileServerMediaSubsession.cpp
	src/H265VideoFileSink.cpp
	src/H265VideoRTPSink.cpp
	src/H265VideoRTPSource.cpp
	src/H265VideoStreamDiscreteFramer.cpp
	src/H265VideoStreamFramer.cpp
	src/HashTable.cpp
	src/inet.c
	src/InputFile.cpp
	src/IOHandlers.cpp
	src/JPEGVideoRTPSink.cpp
	src/JPEGVideoRTPSource.cpp
	src/JPEGVideoSource.cpp
	src/Locale.cpp
	src/MatroskaDemuxedTrack.cpp
	src/MatroskaDemuxedTrack.hh
	src/MatroskaFile.cpp
	src/MatroskaFileParser.cpp
	src/MatroskaFileParser.hh
	src/MatroskaFileServerDemux.cpp
	src/MatroskaFileServerMediaSubsession.cpp
	src/MatroskaFileServerMediaSubsession.hh
	src/Media.cpp
	src/MediaSession.cpp
	src/MediaSink.cpp
	src/MediaSource.cpp
	src/MP3ADU.cpp
	src/MP3ADUdescriptor.cpp
	src/MP3ADUdescriptor.hh
	src/MP3ADUinterleaving.cpp
	src/MP3ADURTPSink.cpp
	src/MP3ADURTPSource.cpp
	src/MP3ADUTranscoder.cpp
	src/MP3AudioFileServerMediaSubsession.cpp
	src/MP3AudioMatroskaFileServerMediaSubsession.cpp
	src/MP3AudioMatroskaFileServerMediaSubsession.hh
	src/MP3FileSource.cpp
	src/MP3Internals.cpp
	src/MP3Internals.hh
	src/MP3InternalsHuffman.cpp
	src/MP3InternalsHuffman.hh
	src/MP3InternalsHuffmanTable.cpp
	src/MP3StreamState.cpp
	src/MP3StreamState.hh
	src/MP3Transcoder.cpp
	src/MPEG1or2AudioRTPSink.cpp
	src/MPEG1or2AudioRTPSource.cpp
	src/MPEG1or2AudioStreamFramer.cpp
	src/MPEG1or2Demux.cpp
	src/MPEG1or2DemuxedElementaryStream.cpp
	src/MPEG1or2DemuxedServerMediaSubsession.cpp
	src/MPEG1or2FileServerDemux.cpp
	src/MPEG1or2VideoFileServerMediaSubsession.cpp
	src/MPEG1or2VideoRTPSink.cpp
	src/MPEG1or2VideoRTPSource.cpp
	src/MPEG1or2VideoStreamDiscreteFramer.cpp
	src/MPEG1or2VideoStreamFramer.cpp
	src/MPEG2IndexFromTransportStream.cpp
	src/MPEG2TransportFileServerMediaSubsession.cpp
	src/MPEG2TransportStreamAccumulator.cpp
	src/MPEG2TransportStreamFramer.cpp
	src/MPEG2TransportStreamFromESSource.cpp
	src/MPEG2TransportStreamFromPESSource.cpp
	src/MPEG2TransportStreamIndexFile.cpp
	src/MPEG2TransportStreamMultiplexor.cpp
	src/MPEG2TransportStreamTrickModeFilter.cpp
	src/MPEG2TransportUDPServerMediaSubsession.cpp
	src/MPEG4ESVideoRTPSink.cpp
	src/MPEG4ESVideoRTPSource.cpp
	src/MPEG4GenericRTPSink.cpp
	src/MPEG4GenericRTPSource.cpp
	src/MPEG4LATMAudioRTPSink.cpp
	src/MPEG4LATMAudioRTPSource.cpp
	src/MPEG4VideoFileServerMediaSubsession.cpp
	src/MPEG4VideoStreamDiscreteFramer.cpp
	src/MPEG4VideoStreamFramer.cpp
	src/MPEGVideoStreamFramer.cpp
	src/MPEGVideoStreamParser.cpp
	src/MPEGVideoStreamParser.hh
	src/MultiFramedRTPSink.cpp
	src/MultiFramedRTPSource.cpp
	src/NetAddress.cpp
	src/NetInterface.cpp
	src/OggDemuxedTrack.cpp
	src/OggDemuxedTrack.hh
	src/OggFile.cpp
	src/OggFileParser.cpp
	src/OggFileParser.hh
	src/OggFileServerDemux.cpp
	src/OggFileServerMediaSubsession.cpp
	src/OggFileServerMediaSubsession.hh
	src/OggFileSink.cpp
	src/OnDemandServerMediaSubsession.cpp
	src/ourMD5.cpp
	src/OutputFile.cpp
	src/PassiveServerMediaSubsession.cpp
	src/ProxyServerMediaSession.cpp
	src/QCELPAudioRTPSource.cpp
	src/QuickTimeFileSink.cpp
	src/QuickTimeGenericRTPSource.cpp
	src/RawVideoRTPSource.cpp
	src/RTCP.cpp
	src/rtcp_from_spec.c
	src/rtcp_from_spec.h
	src/RTPInterface.cpp
	src/RTPSink.cpp
	src/RTPSource.cpp
	src/RTSPClient.cpp
	src/RTSPCommon.cpp
	src/RTSPRegisterSender.cpp
	src/RTSPServer.cpp
	src/RTSPServerRegister.cpp
	src/RTSPServerSupportingHTTPStreaming.cpp
	src/ServerMediaSession.cpp
	src/SimpleRTPSink.cpp
	src/SimpleRTPSource.cpp
	src/SIPClient.cpp
	src/strDup.cpp
	src/StreamParser.cpp
	src/StreamParser.hh
	src/StreamReplicator.cpp
	src/T140TextRTPSink.cpp
	src/TCPStreamSink.cpp
	src/TextRTPSink.cpp
	src/TheoraVideoRTPSink.cpp
	src/TheoraVideoRTPSource.cpp
	src/uLawAudioFilter.cpp
	src/UsageEnvironment.cpp
	src/VideoRTPSink.cpp
	src/VorbisAudioRTPSink.cpp
	src/VorbisAudioRTPSource.cpp
	src/VP8VideoRTPSink.cpp
	src/VP8VideoRTPSource.cpp
	src/VP9VideoRTPSink.cpp
	src/VP9VideoRTPSource.cpp
	src/WAVAudioFileServerMediaSubsession.cpp
	src/WAVAudioFileSource.cpp
)
set(DECODE_FILES
	./jetson_multimedia_api/samples/common/classes/NvApplicationProfiler.cpp
	./jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
	./jetson_multimedia_api/samples/common/classes/NvDrmRenderer.cpp
	./jetson_multimedia_api/samples/common/classes/NvEglRenderer.cpp
	./jetson_multimedia_api/samples/common/classes/NvElement.cpp
	./jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
	./jetson_multimedia_api/samples/common/classes/NvJpegDecoder.cpp
	./jetson_multimedia_api/samples/common/classes/NvJpegEncoder.cpp
	./jetson_multimedia_api/samples/common/classes/NvLogging.cpp
	./jetson_multimedia_api/samples/common/classes/NvUtils.cpp
	./jetson_multimedia_api/samples/common/classes/NvV4l2Element.cpp
	./jetson_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp
	./jetson_multimedia_api/samples/common/classes/NvVideoConverter.cpp
	./jetson_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
	./jetson_multimedia_api/samples/common/classes/NvVideoEncoder.cpp
)	
find_package(OpenCV 4.1.1 REQUIRED HINTS /usr/local/lib/ INCLUDE_DIRECTORIES(/usr/include/opencv4/opencv2))
if(OpenCV_FOUND)
  #add_executable(testRTSPClient ${MAIN_FILE} ${SOURCE_FILES} ${DECODE_FILES})
  #TARGET_LINK_LIBRARIES(testRTSPClient ${OpenCV_LIBS})
  #target_link_libraries(testRTSPClient -lrtmp -lpthread -lv4l2 -lEGL -lGLESv2 -lX11 -lzmq
#	/usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so /usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so /usr/lib/aarch64-linux-gnu/tegra/libnvosd.so -ldrm
#	-lcuda /usr/local/cuda-10.2/targets/aarch64-linux/lib/libcudart.so
#	-lnvinfer -lnvparsers)
endif()
if(OpenCV_FOUND)
  add_executable(thread_close ${MAIN_CLOSE_FILE} ${SOURCE_FILES} ${DECODE_FILES})
  TARGET_LINK_LIBRARIES(thread_close ${OpenCV_LIBS})
  target_link_libraries(thread_close -lrtmp -lpthread -lv4l2 -lEGL -lGLESv2 -lX11 -lzmq
	/usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so /usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so /usr/lib/aarch64-linux-gnu/tegra/libnvosd.so -ldrm
	-lcuda /usr/local/cuda-10.2/targets/aarch64-linux/lib/libcudart.so
	-lnvinfer -lnvparsers)
endif()

