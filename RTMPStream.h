#pragma once
#include "rtmp.h"
#include "rtmp_sys.h"
#include "amf.h"
#include <stdio.h>
#include <bitset>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <time.h>

#define FILEBUFSIZE (1024 * 1024 * 10)       //  10M
 
// NALU单元
typedef struct _NaluUnit
{
	int type;
	int size;
	unsigned char *data;
}NaluUnit;
 
typedef struct _RTMPMetadata
{
	// video, must be h264 type
	unsigned int	nWidth;
	unsigned int	nHeight;
	unsigned int	nFrameRate;		// fps
	unsigned int	nVideoDataRate;	// bps
	unsigned int	nSpsLen;
	unsigned char	Sps[1024];
	unsigned int	nPpsLen;
	unsigned char	Pps[1024];
 
	// audio, must be aac type
	bool	        bHasAudio;
	unsigned int	nAudioSampleRate;
	unsigned int	nAudioSampleSize;
	unsigned int	nAudioChannels;
	char		    pAudioSpecCfg;
	unsigned int	nAudioSpecCfgLen;
 
} RTMPMetadata,*LPRTMPMetadata;
 
 
class CRTMPStream
{
public:
	CRTMPStream(void);
	~CRTMPStream(void);
public:
	// 连接到RTMP Server
	bool Connect(const char* url);
    // 断开连接
	void Close();
    // 发送MetaData
	bool SendMetadata(LPRTMPMetadata lpMetaData);
    // 发送H264数据帧
	bool SendH264Packet(unsigned char *data,unsigned int size,bool bIsKeyFrame,unsigned int nTimeStamp);
	// 发送H264文件
	bool SendH264File(const char *pFileName);
public:
	// 送缓存中读取一个NALU包
	bool ReadOneNaluFromBuf(NaluUnit &nalu);
	// 送h264buffer读取一个NALU包
	bool ReadOneNaluFromBuf(NaluUnit &nalu,unsigned char *pH264Buffer);
	// 送h264buffer读取一个NALU包
	bool ReadOneNaluFromBuf(NaluUnit &nalu,unsigned char *pH264Buffer,int len);
	// 发送数据
	int SendPacket(unsigned int nPacketType,unsigned char *data,unsigned int size,unsigned int nTimestamp);
private:
	RTMP* m_pRtmp;
	unsigned char* m_pFileBuf;
	unsigned int  m_nFileBufSize;
	unsigned int  m_nCurPos;
	unsigned int  m_nH264Pos;
};
