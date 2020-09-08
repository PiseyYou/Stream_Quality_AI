#include "liveMedia.hh"
//#include "RTSPClient.hh"
#include "BasicUsageEnvironment.hh"

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <bitset>
#include <string>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <time.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include "RTMPStream.h"
#include "video_decode_process.h"
#include "video_encode_process.h"
//*********************
#include <locale.h>
#include <wchar.h> 
#include <fstream>
#include <codecvt>
#include <string>
//*********************

#include <stdlib.h>
#include <sys/time.h>
//#include "rkdrm.h"

//*********************
//********************
#include "zmq.hpp"
#include "zhelpers.hpp"
#include "zmq_req.hpp"
//*********************
//#define Is_Record_Datas 1
//#define Is_Record_H264 1
// Forward function definitions:
#define TCP_PORT "5562"
#define IMAGE_TCP_PORT "5562"
#define RTSP_DECODER 1
//#define RTSP_ENCODER 1
#define OPEN_ZMQ 1
#define DETECTOR1_WIDTH 608
#define DETECTOR1_HEIGHT 358
using namespace cv;
using namespace std;
using namespace chrono;
int IMAGE_WIDTH = 1920;
int IMAGE_HEIGHT = 1080;

int record_count = 0;
int record_flag = 1;
string dir11;
string dir22;

//extern uint8_t         *packet_buffer;
//extern uint32_t        packet_wpos;

std::ofstream fw_remove;
bool is_begin_write = false;


//extern mutex mutex_basedata;
//extern condition_variable cond_basedata;
//extern bool gFlagNewBasedata;

void *thread_rtsp_pub(void *threadarg);
void *thread_encoder(void *threadarg);
void *thread_decoder(void *threadarg);
void thread_rep();
void thread_rep_ui();
void thread_req();

#define DETECTORNUM 32
bool isStartPlay = true;
bool isDetect = true;
bool isDetectType[DETECTORNUM];
bool isStartSaveVideo[DETECTORNUM];
bool isSendMsg[DETECTORNUM];
bool isSendState[DETECTORNUM];
string DetectorState[DETECTORNUM];
string DetectorMsg[DETECTORNUM];
string rtsp_url[DETECTORNUM];
string rtmp_url[DETECTORNUM];
bool isAddClient[DETECTORNUM];
string detectIndex[DETECTORNUM];
string rtmpStreamType[DETECTORNUM];
string ai_event[DETECTORNUM];
string ai_board[DETECTORNUM];
int rtmp_width[DETECTORNUM];
int rtmp_height[DETECTORNUM];
//int pub_width[DETECTORNUM];
//int pub_height[DETECTORNUM];
string plateInfo_temp;
int line_count = 0;
unsigned char *yuv_data[DETECTORNUM];
pthread_t rtsp_mpp[DETECTORNUM];
pthread_t rtsp_zmq[DETECTORNUM];
//string rtsp_url = "rtsp://admin:cos(2*pi)!=-1@192.168.3.13/live0";
//string rtsp_url = "rtsp://192.168.3.211/live0";
NVDecoder ndecoder[DETECTORNUM];
NVEncoder nencoder[DETECTORNUM];
string rtsp_sessionId[DETECTORNUM];
int sessionId_count = 0;
bool is_init_sessionId = true;
//context_de ctx_de[DETECTORNUM];
//context_t ctx_en[DETECTORNUM];
extern int is_restart_client;
// RTSP 'response handlers':
void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString);
void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString);
void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString);

// Other event handler functions:
void subsessionAfterPlaying(void* clientData); // called when a stream's subsession (e.g., audio or video substream) ends
void subsessionByeHandler(void* clientData); // called when a RTCP "BYE" is received for a subsession
void streamTimerHandler(void* clientData);
  // called at the end of a stream's expected duration (if the stream has not already signaled its end using a RTCP "BYE")

// The main streaming routine (for each "rtsp://" URL):
void openURL(int index,UsageEnvironment& env, char const* progName, char const* rtspURL);

// Used to iterate through each stream's 'subsessions', setting up each one:
void setupNextSubsession(RTSPClient* rtspClient);

// Used to shut down and close a stream (including its "RTSPClient" object):
void shutdownStream(RTSPClient* rtspClient, int exitCode = 1);

// A function that outputs a string that identifies each stream (for debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env, const RTSPClient& rtspClient) {
  return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

// A function that outputs a string that identifies each subsession (for debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env, const MediaSubsession& subsession) {
  return env << subsession.mediumName() << "/" << subsession.codecName();
}

//*************************************************************************************//

//*************************************************************************************//

string timestr;
string imagelenstr;

//*****************************************************//
struct thread_data{
   int  thread_id;
   int width;
   int height;
};

void usage(UsageEnvironment& env, char const* progName) {
  env << "Usage: " << progName << " <rtsp-url-1> ... <rtsp-url-N>\n";
  env << "\t(where each <rtsp-url-i> is a \"rtsp://\" URL)\n";
}

char eventLoopWatchVariable = 0;
RTSPClient* rtspClient_copy[DETECTORNUM];
int close_index = -1;
bool firtst_init_rtsp = true;
int init_mpp = 0;

#define DEBUG
int RECORD = 1;
//*************************************************************************************//
void infoMessage(string paras, string &rtsp_url_temp,string &rtmp_url_temp,string &videoType,string &detectorType)
{
	int index1 = paras.find_first_of(',');
	rtsp_url_temp = paras.substr(0,index1);
	string rtmp = paras.substr(index1+1,paras.length()-index1-1);
	int index2 = rtmp.find_first_of(',');
	rtmp_url_temp = rtmp.substr(0,index2);
	string videoStreamType = rtmp.substr(index2+1,rtmp.length()-index2-1);
	int index3 = videoStreamType.find_first_of(',');
	videoType = videoStreamType.substr(0,index3);
	string detectType = videoStreamType.substr(index3+1,videoStreamType.length()-index3-1);
	int index4 = detectType.find_first_of(',');
	detectorType = detectType.substr(0,index4);
	if(detectorType == "1")
	{
		int index5 = detectType.find_first_of('}');
		plateInfo_temp = detectType.substr(index4+1,index5-index4);
	}
	return;
}

int main(int argc, char** argv) { 

        for(int i=0; i < DETECTORNUM; i++)
	{
		isDetectType[i] = false;
		isSendMsg[i] = false;
		isSendState[i] = false;
		DetectorState[i] = "unconnected";
		DetectorMsg[i] = "";
		isAddClient[i] = false;
		detectIndex[i] = "0";
		rtsp_url[i] = "rtsp://192.168.1.211/live0";
		rtmp_url[i] = "rtmp://192.168.1.100:1935/live/room";
		rtmpStreamType[i] = "0";
		ai_event[i] = "0";
		ai_board[i] = "0";
		rtmp_width[i] = 1280;
		rtmp_height[i] = 720;
		//pub_width[i] = 608;
		//pub_height[i] = 608;
	}
	/*CRTMPStream rtmpSender;  
	bool bRet = rtmpSender.Connect("rtmp://push.hwvideo.hwcloudlive.com/live/room006");
	cout << "connect:" << bRet << endl;
	rtmpSender.SendH264File("/home/xavier/Videos/20190712054952.h264");  
	rtmpSender.Close();*/
	//cv::Mat aaa;
	//aaa = cv::imread("/home/xavier/Downloads/test_opencv/demo.jpg");
	
	//imshow("aaa",aaa);
	//waitKey(0);
	//std::vector<uchar> data_encode;
	//cv::imencode(".jpg", aaa, data_encode);
		//printf("%d\n",__LINE__);
		//cout << aaa.size() << endl;
	cout << "begin." << endl;
	sleep(1);
	//printf("MPP_VIDEO_CodingAVC=%d\n",MPP_VIDEO_CodingAVC);
	
	//read config file
	std::ifstream infile("config.txt");
	if(infile.is_open())
	{
		std::string line;
		int i = 0;
		while (std::getline(infile, line))
		{
		    std::istringstream iss(line);
		    if (!(iss >> rtsp_url[i] >> rtmp_url[i] >> rtmp_width[i] >> rtmp_height[i])) { break; } // error
		    cout << "rtsp_url[" << i << "]:" << rtsp_url[i] << endl;
		    cout << "rtmp_url[" << i << "]:" << rtmp_url[i] << endl;
		    cout << "rtmp_width[" << i << "]:" << rtmp_width[i] << endl;;
		    cout << "rtmp_height[" << i << "]:" << rtmp_height[i] << endl;
		    //cout << "pub_width[" << i << "]:" << pub_width[i] << endl;;
		    //cout << "pub_height[" << i << "]:" << pub_height[i] << endl;
		    i ++;
		    line_count = i;
		    
		}
		infile.close();
	}
	else
	{
		cout << "can not open config.txt" << endl;
	}
	//line_count = 15;
#ifdef RTSP_DECODER
	struct thread_data td[DETECTORNUM];
	for(int i=0;i<line_count;i++)
	{
		td[i].thread_id = i;
		int rc = pthread_create(&rtsp_mpp[i],NULL,thread_decoder,(void *)&td[i]);
		usleep(1000);
		if (rc)
		{
		    cout << "Error:无法创建线程," << rc << endl;
		    exit(-1);
      		}
	}
#endif
	/*CRTMPStream rtmpSender;  
	bool bRet = rtmpSender.Connect("rtmp://192.168.1.100:1935/live/room");
	cout << "connect:" << bRet << endl;
	rtmpSender.SendH264File("result.h264");  
	rtmpSender.Close();*/

	// Begin by setting up our usage environment:
	TaskScheduler* scheduler = BasicTaskScheduler::createNew();
	UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);

	// We need at least one "rtsp://" URL argument:
	if (argc < 1)
	{
		usage(*env, argv[0]);
		return 1;
	}
	sleep(1);
#ifdef RTSP_ENCODER
	struct thread_data td[DETECTORNUM];
	for(int i=0;i<line_count;i++)
	{
		td[i].thread_id = i;
		int rc = pthread_create(&rtsp_mpp[i],NULL,thread_encoder,(void *)&td[i]);
		usleep(1000);
		if (rc)
		{
		    cout << "Error:无法创建线程," << rc << endl;
		    exit(-1);
      		}
	}
#endif

#ifdef OPEN_ZMQ
	thread zmq_thread(thread_rep);
	thread zmq_thread_ui(thread_rep_ui);
	thread req_thread(thread_req);
#endif
        //cpu_set_t mask;
        //CPU_ZERO(&mask);
        //CPU_SET(0,&mask);
        //CPU_SET(1,&mask);
        //CPU_SET(2,&mask);
        //CPU_SET(3,&mask);
        /*if (sched_setaffinity(0,sizeof(mask),&mask) == -1){
            printf("error_set!\n");
        }*/

	//************************************************//
	while(1)
	{
	// There are argc-1 URLs: argv[1] through argv[argc-1].  Open and start streaming each one:		
		if(!isStartPlay)
		{
			sleep(1);
			continue;
		}
		else if(firtst_init_rtsp)
		{	
			for(int i=0;i<line_count;i++)
			{	
				openURL(i,*env, to_string(i).c_str(), rtsp_url[i].c_str());
			}
			firtst_init_rtsp = false;
		}

		// All subsequent activity takes place within the event loop:
		env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
		// This function call does not return, unless, at some point in time, "eventLoopWatchVariable" gets set to something non-zero.
		for(int i=0;i<line_count;i++)
		{	
			if(close_index == i)
			{
				cout << "close_index: " << close_index << endl;
				shutdownStream(rtspClient_copy[i]);
				openURL(i,*env, argv[0], rtsp_url[i].c_str());
				close_index = -1;
				break;
			}
		}
		//shutdownStream(rtspClient_copy);
		sleep(1);
	}

	
	//*****************************************************//	
	/*for(int i=0;i<line_count;i++)
	{
		rtsp_mpp[i].join();
	}*/
#ifdef OPEN_ZMQ
	zmq_thread.join();
	zmq_thread_ui.join();
	req_thread.join();
#endif
	pthread_exit(NULL);
	return 0;
}

void *thread_encoder(void *threadarg)
{       
	sleep(1);
	struct thread_data *my_data;
   	my_data = (struct thread_data *) threadarg;
	int index = my_data->thread_id;
	nencoder[index].Encode_proc(rtmp_url[index],rtmp_width[index],rtmp_height[index]);
	cout << "init nvencoder[" << index << "]" << endl;
	cout << "*********index:" << index << endl;
}

void *thread_decoder(void *threadarg)
{       
	struct thread_data *my_data;
   	my_data = (struct thread_data *) threadarg;
	int index = my_data->thread_id;
	ndecoder[index].decode_proc(index,rtmp_width[index],rtmp_height[index],rtmp_url[index]);
	cout << "init nvdecoder[" << index << "]" << endl;
}

#ifdef OPEN_ZMQ
void thread_rep()
{       
        zmq::context_t context(1);
	zmq::socket_t socket_rep(context, ZMQ_REP);
	string tcp_s = "tcp://*:5555";
        socket_rep.bind(tcp_s);
        char* res;

        std::cout << "rep begin" << std::endl;
	int recv_count = 1,recv_sum = 1;
                                         
	while(1)
	{	
		zmq::message_t message;
		socket_rep.recv (&message);
		std::string remsg = std::string(static_cast<char*>(message.data()), message.size());
		//char* remsg = reinterpret_cast<char*>(message.data());
		//std::cout << "below is ui msg send..." << std::endl;
        	std::cout << remsg << std::endl;
		int index1 = remsg.find_first_of('=');
		int index2 = remsg.find_first_of(';');
		string name = remsg.substr(index1+1,index2-index1-1);
		//cout << "name = " << name << endl;
		bool is_find_detector = false;
		//if(name != "UI")
		{
		    for(int i=0;i<line_count;i++)
		    {
			string detectname = "detector" + to_string(i+1);
			if(name == detectname)
			{
				if(!isAddClient[i]) isAddClient[i] = true;
				//if(DetectorState[i]=="unconnected")
				{
					DetectorState[i] = "work";
					isSendState[i] = true;
					isDetectType[i] = true;
				}
				string action = remsg.substr(index2+1,remsg.length()-index2-1);
				int index3 = action.find_first_of('=');
				//cout << action.substr(0,index3) << endl;
				if(action.substr(0,index3) == "action")
				{
					string action_value = action.substr(index3+1,action.length()-index3-1);
					//cout << "action_value:" << action_value << endl;
					if(action_value == "getinfo")
					{
						
						if(isDetect && isStartPlay)
						{
							std::string str = to_string(i) + "," + rtmp_url[i] + "," + plateInfo_temp;
							zmq::message_t reply(str.length());
							memcpy((void *)reply.data(),str.c_str(),str.length());
							socket_rep.send (reply);
							is_find_detector = true;
							break;
						}
						else
						{
							ai_board[i] = "1";
							std::string str1 = "noinfo";
							zmq::message_t reply1(str1.length());
							memcpy((void *)reply1.data(),str1.c_str(),str1.length());
							socket_rep.send (reply1);
							DetectorState[i] = "work";
							isSendState[i] = true;
							isDetectType[i] = true;
							is_find_detector = true;
							break;
						}
					}
					else if(action_value == "getport")
					{
						
						if(isDetect && isStartPlay)
						{
							std::string str = "553" + to_string(i);
							zmq::message_t reply(str.length());
							memcpy((void *)reply.data(),str.c_str(),str.length());
							socket_rep.send (reply);
							is_find_detector = true;
							break;
						}
					}
					else if(action_value == "get_rtsp_url")
					{
						
						if(isDetect && isStartPlay)
						{
							std::string str = rtsp_url[i];
							zmq::message_t reply(str.length());
							memcpy((void *)reply.data(),str.c_str(),str.length());
							socket_rep.send (reply);
							is_find_detector = true;
							break;
						}
					}
				
				}
			  }
		    }	
		}

		if(!is_find_detector)
		{
			std::string str = "you are not a detector.";
			zmq::message_t reply(str.length());
			memcpy((void *)reply.data(),str.c_str(),str.length());
			socket_rep.send (reply);
		}
			
		
        }
      	
	std::cerr<<"zmq thread end."<<std::endl;	
	//return 0;
}

void thread_rep_ui()
{       
        zmq::context_t context(1);
	zmq::socket_t socket_rep(context, ZMQ_REP);
	string tcp_s = "tcp://*:5557";
        socket_rep.bind(tcp_s);
        char* res;

        std::cout << "rep begin" << std::endl;
	int recv_count = 1,recv_sum = 1;
                                         
	while(1)
	{	
		zmq::message_t message;
		socket_rep.recv (&message);
		std::string remsg = std::string(static_cast<char*>(message.data()), message.size());
		//char* remsg = reinterpret_cast<char*>(message.data());
		//std::cout << "below is ui msg send..." << std::endl;
        	std::cout << remsg << std::endl;
		int index1 = remsg.find_first_of('=');
		int index2 = remsg.find_first_of(';');
		string name = remsg.substr(index1+1,index2-index1-1);
		//cout << "name = " << name << endl;
		if(name == "UI")
		{
			string event = remsg.substr(index2+1,remsg.length()-index2-1);
			int index3 = event.find_first_of('=');
			int index4 = event.find_first_of(';');
			string event_name = event.substr(index3+1,index4-index3-1);
			cout << "event_name = " << event_name << endl;
			int aievent_count = 0;
			if(event_name == "frameInfo")
			{
				string para = event.substr(index4+1,event.length()-index4-1);
				int index5 = para.find_first_of('=');
				string para_value = para.substr(0,index5);
				if(para_value == "para")
				{
					std::ofstream outfile("config.txt",ios::out);
					if(!outfile.is_open())
					{
						cout << "can not open config.txt." << endl;
					}
					string para_msg = para.substr(index5+2,para.length()-index5-1);
					for(int i = 0;;i++)
					{
						int index6 = para_msg.find_first_of('[');
						int index7 = para_msg.find_first_of(']');
						if(index6 == -1 || index7 == -1) break;
						string para_temp = para_msg.substr(index6+1,index7-index6-1);
						infoMessage(para_temp, rtsp_url[i],rtmp_url[i],rtmpStreamType[i],detectIndex[i]);
						if(outfile.is_open())
						{
							outfile << rtsp_url[i] << " " << rtmp_url[i] << " ";
							if(rtmpStreamType[i] == "0")
							{
								outfile << "352 288";
							}
							else if(rtmpStreamType[i] == "1")
							{
								outfile << "720 576";
							}
							else if(rtmpStreamType[i] == "2")
							{
								outfile << "1280 720";
							}
							//outfile << pub_width[i] << " " << pub_height[i] << endl;
							outfile << endl;
						}
						if(ai_board[i] == "1") ai_event[i] = "1";
						else if(detectIndex[i]=="0") ai_event[i] = "0";
						else ai_event[i] = "-1";
						aievent_count ++;
						if(index7 + 1 >= para_msg.length()) break;
						else
						{
							para_msg = para_msg.substr(index7+1,para_msg.length()-index7-1);
						}
						cout << "rtsp_url[i] = " << rtsp_url[i] << endl;
						cout << "rtmp_url[i] = " << rtmp_url[i] << endl;
						cout << "rtmpStreamType[i] = " << rtmpStreamType[i] << endl;
						cout << "detectIndex[i] = " << detectIndex[i] << endl;
						cout << "plateInfo_temp = " << plateInfo_temp << endl;
					}
					if(outfile.is_open()) outfile.close();
					isDetect = true;
					isStartPlay = true;
				}
				string eventstr="";
				for(int i=0;i < aievent_count;i++) 
				{	
					eventstr = eventstr + ai_event[i];
					if(i+1<aievent_count) eventstr = eventstr + ",";
				}
				std::string str = "name=server;event=frameInfo;para=[" + eventstr + "]";
				cout << str << endl;
				zmq::message_t reply(str.length());
				memcpy((void *)reply.data(),str.c_str(),str.length());
				socket_rep.send (reply);
				continue;
			}
			
		}	
		else
		{
			std::string str = "no ui name.";
			zmq::message_t reply(str.length());
			memcpy((void *)reply.data(),str.c_str(),str.length());
			socket_rep.send (reply);
		}
				
		
        }
      	
	std::cerr<<"zmq thread end."<<std::endl;	
	//return 0;
}
void thread_req()
{       
	zmq::context_t context(1);
	zmq::socket_t socket_req(context, ZMQ_REQ);
	string tcp_s = "tcp://192.168.1.210:5556";
        socket_req.connect(tcp_s);
        char* res;
	int timesend = 3000;
        std::cout << "req begin" << std::endl;
	int recv_count = 1,recv_sum = 1;
        //********************************************************************
	bool isadd[DETECTORNUM];
	int StateCount[DETECTORNUM];
	string State_temp[DETECTORNUM];
	for(int i=0; i < DETECTORNUM; i++)
	{
		isadd[i] = false;
		StateCount[i] = 0;
		State_temp[i] = "unconnected";
	}                                     
	while(1)
	{	
	    for(int i=0; i < line_count; i++)
	    {
		//cout << "isDetect:" << isDetect << endl;
		//cout << "isStartPlay:" << isDetect << endl;
		//cout << "isSendState[i]:" << isSendState[i] << endl;
		//cout << "DetectorState[i]:" << DetectorState[i] << endl;
		if(isDetect && isStartPlay && isSendState[i])
		{
			/*if(StateCount[i] < timesend/line_count && State_temp[i] == DetectorState[i])
			{
				//cout << "No Send State." << endl;
				//StateCount[i] = 0;
				isSendState[i] = false;
				continue;
			}
			else*/
			{
				cout << "send state" << endl;
				std::string str = "name=server;event=aistate;para=["+to_string(i)+","+ detectIndex[i]+ "," +DetectorState[i]+ "]";
				zmq::message_t message(str.length());
				memcpy((void *)message.data(),str.c_str(),str.length());
				socket_req.send (message);
				zmq::message_t reply;
				socket_req.recv (&reply);
				std::string remsg = std::string(static_cast<char*>(reply.data()), reply.size());
				State_temp[i] = DetectorState[i];
				isSendState[i] = false;
				StateCount[i] = 0;
				std::cout << remsg << std::endl;
			}
		}
		else if(isDetect && isStartPlay && isDetectType[i] && StateCount[i] < timesend/line_count)
		{
			StateCount[i] ++;
			//cout << "StateCount[i]: " << endl;
			usleep(10000);
		}
		else if(isDetect && isStartPlay && isDetectType[i] && StateCount[i] >= timesend/line_count)
		{
			if(DetectorState[i]=="work") 
			{
				DetectorState[i] = "offline";
				isSendState[i] = true;
			}	
		}
	    }
        }
      	
	std::cerr<<"req thread end."<<std::endl;	
	//return 0;
}
#endif
// Define a class to hold per-stream state that we maintain throughout each stream's lifetime:

class StreamClientState {
public:
  StreamClientState();
  virtual ~StreamClientState();

public:
  MediaSubsessionIterator* iter;
  MediaSession* session;
  MediaSubsession* subsession;
  TaskToken streamTimerTask;
  double duration;
};

// If you're streaming just a single stream (i.e., just from a single URL, once), then you can define and use just a single
// "StreamClientState" structure, as a global variable in your application.  However, because - in this demo application - we're
// showing how to play multiple streams, concurrently, we can't do that.  Instead, we have to have a separate "StreamClientState"
// structure for each "RTSPClient".  To do this, we subclass "RTSPClient", and add a "StreamClientState" field to the subclass:

class ourRTSPClient: public RTSPClient {
public:
  static ourRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL,
				  int verbosityLevel = 0,
				  char const* applicationName = NULL,
				  portNumBits tunnelOverHTTPPortNum = 0);

protected:
  ourRTSPClient(UsageEnvironment& env, char const* rtspURL,
		int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum);
    // called only by createNew();
  virtual ~ourRTSPClient();

public:
  StreamClientState scs;
};

// Define a data sink (a subclass of "MediaSink") to receive the data for each subsession (i.e., each audio or video 'substream').
// In practice, this might be a class (or a chain of classes) that decodes and then renders the incoming audio or video.
// Or it might be a "FileSink", for outputting the received data into a file (as is done by the "openRTSP" application).
// In this example code, however, we define a simple 'dummy' sink that receives incoming data, but does nothing with it.

class DummySink: public MediaSink {
public:
  static DummySink* createNew(UsageEnvironment& env,
			      MediaSubsession& subsession, // identifies the kind of data that's being received
			      char const* streamId = NULL); // identifies the stream itself (optional)

private:
  DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId);
    // called only by "createNew()"
  virtual ~DummySink();

  static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
				struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
			 struct timeval presentationTime, unsigned durationInMicroseconds);

private:
  // redefined virtual functions:
  virtual Boolean continuePlaying();

private:
  u_int8_t* fReceiveBuffer;
  MediaSubsession& fSubsession;
  char* fStreamId;
};

#define RTSP_CLIENT_VERBOSITY_LEVEL 1 // by default, print verbose output from each "RTSPClient"

static unsigned rtspClientCount = 0; // Counts how many streams (i.e., "RTSPClient"s) are currently in use.

void openURL(int index,UsageEnvironment& env, char const* progName, char const* rtspURL) {
  // Begin by creating a "RTSPClient" object.  Note that there is a separate "RTSPClient" object for each stream that we wish
  // to receive (even if more than stream uses the same "rtsp://" URL).
  RTSPClient* rtspClient = ourRTSPClient::createNew(env, rtspURL, RTSP_CLIENT_VERBOSITY_LEVEL, progName);
  rtspClient_copy[index] = rtspClient;
  if (rtspClient == NULL) {
    env << "Failed to create a RTSP client for URL \"" << rtspURL << "\": " << env.getResultMsg() << "\n";
    return;
  }

  ++rtspClientCount;

  // Next, send a RTSP "DESCRIBE" command, to get a SDP description for the stream.
  // Note that this command - like all RTSP commands - is sent asynchronously; we do not block, waiting for a response.
  // Instead, the following function call returns immediately, and we handle the RTSP response later, from within the event loop:
  rtspClient->sendDescribeCommand(continueAfterDESCRIBE); 
}


// Implementation of the RTSP 'response handlers':

void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString) {
  do {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to get a SDP description: " << resultString << "\n";
      delete[] resultString;
      break;
    }

    char* const sdpDescription = resultString;
    env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";

    // Create a media session object from this SDP description:
    scs.session = MediaSession::createNew(env, sdpDescription);
    delete[] sdpDescription; // because we don't need it anymore
    if (scs.session == NULL) {
      env << *rtspClient << "Failed to create a MediaSession object from the SDP description: " << env.getResultMsg() << "\n";
      break;
    } else if (!scs.session->hasSubsessions()) {
      env << *rtspClient << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
      break;
    }

    // Then, create and set up our data source objects for the session.  We do this by iterating over the session's 'subsessions',
    // calling "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command, on each one.
    // (Each 'subsession' will have its own data source.)
    scs.iter = new MediaSubsessionIterator(*scs.session);
    setupNextSubsession(rtspClient);
    return;
  } while (0);

  // An unrecoverable error occurred with this stream.
  shutdownStream(rtspClient);
}

// By default, we request that the server stream its data using RTP/UDP.
// If, instead, you want to request that the server stream via RTP-over-TCP, change the following to True:
#define REQUEST_STREAMING_OVER_TCP true

void setupNextSubsession(RTSPClient* rtspClient) {
  UsageEnvironment& env = rtspClient->envir(); // alias
  StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias
  
  scs.subsession = scs.iter->next();
  if (scs.subsession != NULL) {
    if (!scs.subsession->initiate()) {
      env << *rtspClient << "Failed to initiate the \"" << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
      setupNextSubsession(rtspClient); // give up on this subsession; go to the next one
    } else {
      env << *rtspClient << "Initiated the \"" << *scs.subsession << "\" subsession (";
      if (scs.subsession->rtcpIsMuxed()) {
	env << "client port " << scs.subsession->clientPortNum();
      } else {
	env << "client ports " << scs.subsession->clientPortNum() << "-" << scs.subsession->clientPortNum()+1;
      }
      env << ")\n";

      // Continue setting up this subsession, by sending a RTSP "SETUP" command:
	
      rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False, REQUEST_STREAMING_OVER_TCP);
    }
    return;
  }

  // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY" command to start the streaming:
  if (scs.session->absStartTime() != NULL) {
    // Special case: The stream is indexed by 'absolute' time, so send an appropriate "PLAY" command:
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY, scs.session->absStartTime(), scs.session->absEndTime());
  } else {
    scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
  }
}

void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString) {
  do {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias
	
    if (resultCode != 0) {
	cout<<"******resultCode*****"<<resultCode<<"\n"<<endl;
      env << *rtspClient << "Failed to set up the \"" << *scs.subsession << "\" subsession: " << resultString << "\n";
      break;
    }

    env << *rtspClient << "Set up the \"" << *scs.subsession << "\" subsession (";
    if (scs.subsession->rtcpIsMuxed()) {
      env << "client port " << scs.subsession->clientPortNum();
    } else {
      env << "client ports " << scs.subsession->clientPortNum() << "-" << scs.subsession->clientPortNum()+1;
    }
    env << ")\n";

    // Having successfully setup the subsession, create a data sink for it, and call "startPlaying()" on it.
    // (This will prepare the data sink to receive data; the actual flow of data from the client won't start happening until later,
    // after we've sent a RTSP "PLAY" command.)

    scs.subsession->sink = DummySink::createNew(env, *scs.subsession, rtspClient->url());
      // perhaps use your own custom "MediaSink" subclass instead
    if (scs.subsession->sink == NULL) {
      env << *rtspClient << "Failed to create a data sink for the \"" << *scs.subsession
	  << "\" subsession: " << env.getResultMsg() << "\n";
      break;
    }

    env << *rtspClient << "Created a data sink for the \"" << *scs.subsession << "\" subsession\n";
    scs.subsession->miscPtr = rtspClient; // a hack to let subsession handler functions get the "RTSPClient" from the subsession 
    scs.subsession->sink->startPlaying(*(scs.subsession->readSource()),
				       subsessionAfterPlaying, scs.subsession);
    // Also set a handler to be called if a RTCP "BYE" arrives for this subsession:
    if (scs.subsession->rtcpInstance() != NULL) {
      scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler, scs.subsession);
    }
  } while (0);
  delete[] resultString;

  // Set up the next subsession, if any:
  setupNextSubsession(rtspClient);
}

void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString) {
  Boolean success = False;

  do {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to start playing session: " << resultString << "\n";
      break;
    }

    // Set a timer to be handled at the end of the stream's expected duration (if the stream does not already signal its end
    // using a RTCP "BYE").  This is optional.  If, instead, you want to keep the stream active - e.g., so you can later
    // 'seek' back within it and do another RTSP "PLAY" - then you can omit this code.
    // (Alternatively, if you don't want to receive the entire stream, you could set this timer for some shorter value.)
    if (scs.duration > 0) {
      unsigned const delaySlop = 2; // number of seconds extra to delay, after the stream's expected duration.  (This is optional.)
      scs.duration += delaySlop;
      unsigned uSecsToDelay = (unsigned)(scs.duration*1000000);
      scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(uSecsToDelay, (TaskFunc*)streamTimerHandler, rtspClient);
    }
	
    env << *rtspClient << "Started playing session";
    if (scs.duration > 0) {
      env << " (for up to " << scs.duration << " seconds)";
    }
    env << "...\n";

    success = True;
  } while (0);
  delete[] resultString;

  if (!success) {
    // An unrecoverable error occurred with this stream.
    shutdownStream(rtspClient);
  }
}


// Implementation of the other event handlers:

void subsessionAfterPlaying(void* clientData) {
  MediaSubsession* subsession = (MediaSubsession*)clientData;
  RTSPClient* rtspClient = (RTSPClient*)(subsession->miscPtr);

  // Begin by closing this subsession's stream:
  Medium::close(subsession->sink);
  subsession->sink = NULL;

  // Next, check whether *all* subsessions' streams have now been closed:
  MediaSession& session = subsession->parentSession();
  MediaSubsessionIterator iter(session);
  while ((subsession = iter.next()) != NULL) {
    if (subsession->sink != NULL) return; // this subsession is still active
  }

  // All subsessions' streams have now been closed, so shutdown the client:
  shutdownStream(rtspClient);
}

void subsessionByeHandler(void* clientData) {
  MediaSubsession* subsession = (MediaSubsession*)clientData;
  RTSPClient* rtspClient = (RTSPClient*)subsession->miscPtr;
  UsageEnvironment& env = rtspClient->envir(); // alias

  env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession << "\" subsession\n";

  // Now act as if the subsession had closed:
  subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void* clientData) {
  ourRTSPClient* rtspClient = (ourRTSPClient*)clientData;
  StreamClientState& scs = rtspClient->scs; // alias

  scs.streamTimerTask = NULL;

  // Shut down the stream:
  shutdownStream(rtspClient);
}

void shutdownStream(RTSPClient* rtspClient, int exitCode) {
  UsageEnvironment& env = rtspClient->envir(); // alias
  StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

  // First, check whether any subsessions have still to be closed:
  if (scs.session != NULL) { 
    Boolean someSubsessionsWereActive = False;
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession* subsession;

    while ((subsession = iter.next()) != NULL) {
      if (subsession->sink != NULL) {
	Medium::close(subsession->sink);
	subsession->sink = NULL;

	if (subsession->rtcpInstance() != NULL) {
	  subsession->rtcpInstance()->setByeHandler(NULL, NULL); // in case the server sends a RTCP "BYE" while handling "TEARDOWN"
	}

	someSubsessionsWereActive = True;
      }
    }

    if (someSubsessionsWereActive) {
      // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the stream.
      // Don't bother handling the response to the "TEARDOWN".
      rtspClient->sendTeardownCommand(*scs.session, NULL);
    }
  }

  env << *rtspClient << "Closing the stream.\n";
  Medium::close(rtspClient);
    // Note that this will also cause this stream's "StreamClientState" structure to get reclaimed.

  if (--rtspClientCount == 0) {
    // The final stream has ended, so exit the application now.
    // (Of course, if you're embedding this code into your own application, you might want to comment this out,
    // and replace it with "eventLoopWatchVariable = 1;", so that we leave the LIVE555 event loop, and continue running "main()".)
    //exit(exitCode);
	;
  }
}


// Implementation of "ourRTSPClient":

ourRTSPClient* ourRTSPClient::createNew(UsageEnvironment& env, char const* rtspURL,
					int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum) {
  return new ourRTSPClient(env, rtspURL, verbosityLevel, applicationName, tunnelOverHTTPPortNum);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment& env, char const* rtspURL,
			     int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum)
  : RTSPClient(env,rtspURL, verbosityLevel, applicationName, tunnelOverHTTPPortNum, -1) {
}

ourRTSPClient::~ourRTSPClient() {
}


// Implementation of "StreamClientState":

StreamClientState::StreamClientState()
  : iter(NULL), session(NULL), subsession(NULL), streamTimerTask(NULL), duration(0.0) {
}

StreamClientState::~StreamClientState() {
  delete iter;
  if (session != NULL) {
    // We also need to delete "session", and unschedule "streamTimerTask" (if set)
    UsageEnvironment& env = session->envir(); // alias

    env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
    Medium::close(session);
  }
}


// Implementation of "DummySink":

// Even though we're not going to be doing anything with the incoming data, we still need to receive it.
// Define the size of the buffer that we'll use:
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 1000000

DummySink* DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) {
  return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId)
  : MediaSink(env),
    fSubsession(subsession) {
  fStreamId = strDup(streamId);
  fReceiveBuffer = new u_int8_t[DUMMY_SINK_RECEIVE_BUFFER_SIZE];
}

DummySink::~DummySink() {
  delete[] fReceiveBuffer;
  delete[] fStreamId;
}

void DummySink::afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes,
				  struct timeval presentationTime, unsigned durationInMicroseconds) {
  DummySink* sink = (DummySink*)clientData;
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
  
}

long int fps_ms_main = 0;
long int diff_main;
int fps_counter_main;

// If you don't want to see debugging output for each received frame, then comment out the following line:
//#define DEBUG_PRINT_EACH_RECEIVED_FRAME 1
void DummySink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
				  struct timeval presentationTime, unsigned /*durationInMicroseconds*/) {
  // We've just received a frame of data.  (Optionally) print out information about it:
#ifdef DEBUG_PRINT_EACH_RECEIVED_FRAME
  if (fStreamId != NULL) envir() << "Stream \"" << fStreamId << "\"; ";
  envir() << fSubsession.mediumName() << "/" << fSubsession.codecName() << ":\tReceived " << frameSize << " bytes\n";
  //envir() << "fReceiveBuffer: " << fReceiveBuffer << "\n";
  if (numTruncatedBytes > 0) envir() << " (with " << numTruncatedBytes << " bytes truncated)";
  char uSecsStr[6+1]; // used to output the 'microseconds' part of the presentation time
  sprintf(uSecsStr, "%06u", (unsigned)presentationTime.tv_usec);
  envir() << ".\tPresentation time: " << (int)presentationTime.tv_sec << "." << uSecsStr;
  if (fSubsession.rtpSource() != NULL && !fSubsession.rtpSource()->hasBeenSynchronizedUsingRTCP()) {
    envir() << "!"; // mark the debugging output to indicate that this presentation time is not RTCP-synchronized
  }
#ifdef DEBUG_PRINT_NPT
  envir() << "\tNPT: " << fSubsession.getNormalPlayTime(presentationTime);
#endif
  envir() << "\n";
#endif

#ifdef RTSP_DECODER
  /*if(is_init_sessionId && sessionId_count == 0)
  {
	rtsp_sessionId[sessionId_count] = fSubsession.sessionId();
	sessionId_count ++;
	cout << rtsp_sessionId[sessionId_count - 1] << endl;
	if(sessionId_count == line_count) is_init_sessionId = false;
  }
  else if(is_init_sessionId && sessionId_count < line_count)
  {
	bool isfind = true;
	for(int i=0;i < sessionId_count;i++)
	{
		if(rtsp_sessionId[i] == fSubsession.sessionId())
		{
			isfind = false;
			break;
		}
	}
	if(isfind)
	{
		rtsp_sessionId[sessionId_count] = fSubsession.sessionId();
		sessionId_count ++;
		cout << rtsp_sessionId[sessionId_count - 1] << endl;
		if(sessionId_count == line_count) is_init_sessionId = false;
	}
  }
  else if(!is_init_sessionId)
  {
	for(int i=0;i<line_count;i++)
	{
		if(rtsp_sessionId[i] == fSubsession.sessionId() && frameSize > 0)
		{
			//cout <<"i=" << i << ";frameSize = " << frameSize << endl;
			int flag = ndecoder[i].decode_h264(fReceiveBuffer,frameSize);
			break;
		}
		else if(rtsp_sessionId[i] == fSubsession.sessionId() && frameSize == 0)
		{
			break;
		}
	}
  }*/
  for(int i=0;i<line_count;i++)
  {
  	if(fStreamId == rtsp_url[i] && frameSize > 0)
	{
		//cout <<"i=" << i << ";frameSize = " << frameSize << endl;
		int flag = ndecoder[i].decode_h264(fReceiveBuffer,frameSize);
		break;
	}
	else if(fStreamId == rtsp_url[i] && frameSize == 0)
	{
		break;
	}
  }
  //usleep(1000);
#endif
  //cout << fSubsession.clientPortNum() << endl;
  
  if(frameSize > 100) fps_counter_main ++;
  long int now = presentationTime.tv_sec*1000 + presentationTime.tv_usec/1000; 		
  if ((diff_main = now - fps_ms_main) >= 1000) 
  {
    float fps = fps_counter_main / (diff_main*line_count / 1000.0);
    fps_counter_main = 0;
    fps_ms_main = now;
    cout << "fps_main: "<< fps << endl;
    //cout << fSubsession.sessionId() << endl;
    //cout << fStreamId << endl;
  }
  /*packet_buffer[packet_wpos++] = 0;
  packet_buffer[packet_wpos++] = 0;
  packet_buffer[packet_wpos++] = 0;
  packet_buffer[packet_wpos++] = 1;
  memcpy(packet_buffer + packet_wpos, fReceiveBuffer, frameSize);
  packet_wpos += frameSize;
  ndecoder[0].decoder_routine();*/
  //cout << "frameSize = " << frameSize << endl;
  //cout << "fStreamId:" << fStreamId << endl;
  // Then continue, to request the next frame of data:
  continuePlaying();
}

Boolean DummySink::continuePlaying() {
  if (fSource == NULL) return False; // sanity check (should not happen)

  // Request the next frame of data from our input source.  "afterGettingFrame()" will get called later, when it arrives:
  fSource->getNextFrame(fReceiveBuffer, DUMMY_SINK_RECEIVE_BUFFER_SIZE,
                        afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}
