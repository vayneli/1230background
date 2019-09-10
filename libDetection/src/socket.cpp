#include "socket.hpp"
using namespace std;
Socket::Socket(){
   
     
    //开启socket（）
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);

    if(sock_fd < 0){
        cout << "-------------Creat connection error--------------" << endl;
    }
    /*设置socket参数*/
    //使用ipv4地址
    server_addr.sin_family = AF_INET;   
    //设置端口号
    server_addr.sin_port = htons(6666);
    //ip地址由于客户端和服务器不同，在各自的初始化部分设置
    s1 = "#c6 ";
    s2=" $";
    
}

Socket::~Socket(){
    close(sock_fd);
}

int Socket::server_init(){
    //设置ipv4地址
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //绑定socket
    int ret = bind(sock_fd, (struct sockaddr*)&server_addr, sizeof(server_addr));

    if(ret < 0){
        cout << "Can not bind server socket" << endl;
        close(sock_fd);
        return -1;
    }
    //进入监听状态，等待用户发起请求
    ret = listen(sock_fd, 20);
  
    if(ret < 0){
        cout << "Can not listen socket" << endl;
        close(sock_fd);
        return -1;
    }

    socklen_t len = sizeof(client_addr);
    com_fd = accept(sock_fd, (struct sockaddr*)&client_addr, &len);

    if(com_fd < 0){
        cout << "Can not accept request" << endl;
        close(sock_fd);
        return -1;
    }
    
    return 0;
}

int Socket::client_init(string address){

    //设置ipv4地址,该地址为服务器的ip地址
    server_addr.sin_addr.s_addr = inet_addr(address.c_str());
    connect(sock_fd, (struct sockaddr*)&server_addr, sizeof(server_addr));
    //读取服务器传回的数据
    if(sock_fd < 0)
    {
        cout << "Can not connect to the server" << endl;
        return -1;
    }
    
        return 0; 
    }


int Socket::send_data(float x,float y,float z){

    //x=3*(x-240)/240;
    //y=3*(y-320)/320;
    s3=s1+to_string(x) +" "+to_string(y)+" "+to_string(z)+" "+to_string(0)+" "+to_string(0)+" "+to_string(0)+s2;
    const char* data=s3.c_str();
    int num = write(sock_fd, data, strlen(data));
    return num;
}

int Socket::send_data(const char* data){

    
    int num = write(sock_fd, data, strlen(data));
    return num;
}

int Socket::receive_data(){
    memset(receive_buff,0,1024);
    int num = read(sock_fd,receive_buff,sizeof(receive_buff)); 
    return num;
}

