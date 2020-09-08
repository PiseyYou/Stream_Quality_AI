#ifndef __ZMQ_REQ_HPP_INCLUDED__
#define __ZMQ_REQ_HPP_INCLUDED__

//#include <zmq.hpp>
//#include "zhelpers.hpp"
#include <time.h>
int req_queue(zmq::socket_t& Reqer)
{

        std::string name = "name=detector1;action=requestqueue";
        zmq::message_t message( strlen(name.c_str()) );
        memcpy(message.data(),name.c_str(),strlen(name.c_str()));
	Reqer.send(message);
        std::cout << "send " << name <<" "<< strlen(name.c_str()) <<std::endl;
        zmq::message_t reply;
        Reqer.recv(&reply);
        char* remsg = reinterpret_cast<char*>(reply.data());
        std::cout << remsg << std::endl;

        return 0;

}
int req_img(zmq::socket_t& Reqer)
{
        clock_t start = clock();
        char* name = "name=detector1;action=getnextframe";
        zmq::message_t message( strlen(name) );
        memcpy(message.data(),name,strlen(name));
	Reqer.send(message);
       // std::cout << "send " << name << std::endl;

        zmq::message_t reply;
        Reqer.recv(&reply);
        clock_t end = clock();
        std::cout << "zmq cost " << (double)(end - start)/CLOCKS_PER_SEC << std::endl;
        char* remsg = reinterpret_cast<char*>(reply.data());
        std::cout << remsg << std::endl;
        std::string msg(remsg);
        if (msg.substr(0,2) == "ok")return 1;
        //if (strcmp(remsg, "ok")==0)return 1;
        else return 0;

}

int send_result(zmq::socket_t& Reqer, long int timpstamp, char* result)
{   
    std::cout << "send result" << std::endl;
    clock_t start = clock();
    std::string res(result);
    std::string name = "name=detector1;";
    std::string timesp = "timestamp=" + std::to_string(timpstamp)+";";
    std::string event = "event=Plate_detect;";
    std::string plat_res = "plate_result=" + res + ";";
    std::string msg_res = name + timesp + event + plat_res;

    zmq::message_t message( strlen(msg_res.c_str()) );
    memcpy(message.data(),msg_res.c_str(),strlen(msg_res.c_str()));
    Reqer.send(message);

    zmq::message_t reply;
    Reqer.recv(&reply);
    char* remsg = reinterpret_cast<char*>(reply.data());
    clock_t end = clock();
    std::cout << "zmq cost " << (double)(end - start)/CLOCKS_PER_SEC << std::endl;

    std::cout << msg_res.c_str() << std::endl;
    std::cout << remsg << std::endl;
    

    return 1;

}

#endif
