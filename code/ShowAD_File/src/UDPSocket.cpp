// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-07-06 10:07
// Email        : 2650982628@qq.com
// Filename     : UDPSocket.cpp
// Description  : 
// ******************************************************

#include "UDPSocket.h"
#include "Function.hpp"
#include <functional>
#include <iostream>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

UDPSocket::UDPSocket(const std::string & addr,int port):m_addr(addr, port),m_func(std::function<void(const std::string &)>{})
{
        // 创建UDP socket
        m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sockfd == -1) {
            std::cerr << "Failed to create socket." << std::endl;
            exit(EXIT_FAILURE);
        }

        // 设置接收超时时间为1秒
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        if (setsockopt(m_sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
            std::cerr << "Failed to set socket receive timeout: " << strerror(errno) << std::endl;
            close(m_sockfd);
            return ;
        }

        struct sockaddr_in Addr;
        // 设置服务器地址
        Addr.sin_family = AF_INET;
        Addr.sin_port = htons(m_addr._port);  // 让系统自动分配一个可用端口
        Addr.sin_addr.s_addr = inet_addr(m_addr._ip.c_str());

        // 将socket和地址绑定
        if (bind(m_sockfd, (struct sockaddr*)&Addr, sizeof(Addr)) == -1) {
            std::cerr << "Failed to bind socket." << std::endl;
            exit(EXIT_FAILURE);
        }
}

int UDPSocket::sendTo(int socketfd,const std::string& addr, int port, const std::string& message) {
        struct sockaddr_in destAddr;
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(port);
        destAddr.sin_addr.s_addr = inet_addr(addr.c_str());

        // 发送消息
        ssize_t bytesSent = sendto(socketfd, message.c_str(), message.length(), 
                0, (struct sockaddr*)&destAddr, sizeof(destAddr));
        if (bytesSent == -1) {
            std::cerr << "Failed to send message." << std::endl;
            return -1;
        } 
        else {
            std::cout << "Sent " << bytesSent << " bytes to " 
                << addr << ":" << port << std::endl;
            return 0;
        }
}

std::string UDPSocket::receive() {
        char buffer[BUFFER_SIZE];
        struct sockaddr_in srcAddr;
        socklen_t addrlen = sizeof(srcAddr);

        // 接收消息
        ssize_t bytesReceived = recvfrom(m_sockfd, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr*)&srcAddr, &addrlen);
        if (bytesReceived == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // std::cerr << "Receive timeout" << std::endl;
                return "";
            } else {
                std::cerr << "Failed to receive data: " << strerror(errno) << std::endl;
            }
            return "";
        }

        buffer[bytesReceived] = '\0';
        std::string receivedMessage(buffer);
        std::cout << "Received " << bytesReceived << " bytes from " << inet_ntoa(srcAddr.sin_addr) << ":" << ntohs(srcAddr.sin_port) << std::endl;
        return receivedMessage;
}


void UDPSocket::run(){
    m_flag_if_stop = false;
    while (!m_flag_if_stop) {
        auto msg = receive();
        if(m_func && !msg.empty()){
            m_func(msg);
        }
    }
}


int UDPmain(){

    UDPSocket sock("0.0.0.0",3001);
    sock.Start();
    int fd = sock.getSokcetfd();
    sock.sendTo(fd, "0.0.0.0", 3000, "absd\n");

    sock.setFunction([fd](const std::string &msg){
            UDPSocket::sendTo(fd, "0.0.0.0", 3000, "Focus:15,UnFocus:10,Total:25,Attractiveness:0.6");
            std::cout << "recv : " << msg << std::endl;
        });

    while (true) {
        if(Function::kbhit()&&(fgetc(stdin) == 'q')){
            std::cout << std::endl;
            sock.quit();
            break;
        }
        usleep(50*1000);
    }

    return 0;
}
