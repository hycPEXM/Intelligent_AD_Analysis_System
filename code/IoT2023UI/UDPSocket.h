// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-07-06 17:55
// Email        : 2650982628@qq.com
// Filename     : UDPSocket.h
// Description  : 
// ******************************************************

#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <functional>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "MThread.hpp"


#define BUFFER_SIZE 1024

typedef struct Addr{
    Addr(const std::string& addr,uint16_t ip = 0):_ip(addr),_port(ip){};
    const std::string _ip;
    uint16_t _port;
}AddrStruct;


class UDPSocket: public MThread {

private:
    int m_sockfd;
    AddrStruct m_addr;
    bool m_flag_if_init;
    std::function<void(const std::string &)> m_func;

public:
    UDPSocket(const std::string & addr);
    ~UDPSocket(){close(m_sockfd);}

public:
    bool InitUdp(uint16_t port);
    void DeInitUdp();

public:
    static int sendTo(int socketfd,const std::string& addr, int port, const std::string& message);
    std::string receive();

    int  getSokcetfd(){return m_sockfd;}
    std::string getIp(){return m_addr._ip;}
    uint16_t getPort(){return m_addr._port;}

    void setFunction(std::function<void(const std::string&)> func){
        m_func =  func;
    }
    virtual void run();
};
#endif
