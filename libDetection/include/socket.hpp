#pragma once

#include <iostream>
#include <set>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <string>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
//inet需要包含下面头文件

#include <arpa/inet.h>
#include <netinet/in.h>


class Socket{

    public:
        Socket();
        /*初始化客户端和服务器的socket
        1.打开socket 2.设置地址和端口号 3.绑定 
        4.服务器进入监听模式，客户端连接目标地址
        */
        ~Socket();
        
        int server_init();
        int client_init(std::string address);
        

        int receive_data();
        //int send_data(const char* data);
        int send_data(float x,float y,float z);
        int send_data(const char* data);

    
    public:
        struct sockaddr_in client_addr;
        struct sockaddr_in server_addr;
        char receive_buff[1024];
        int sock_fd;
        int com_fd;
        std::string s1,s2,s3;

};

