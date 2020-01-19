////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      MainFun Code for robot
///ALL RIGHTS RESERVED
///@file:main.cpp
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-9-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "control_model.h"
#include "robot_model.h"
#include "serial_listen_thread.h"
#include "radio_listen_thread.h"
#include <stdlib.h>
#include <dji_linux_helpers.hpp>
#include <signal.h>
using  namespace std;

RobotModel robotModel;
void signal_exit_handler(int sig){
	robotModel.release();
	cout<<"hahahahahahahah"<<endl;
	exit(0);
}
int main(int argc, char** argv){
    //RobotModel robotModel;
    LinuxSetup linuxEnvironment(argc,argv);
   // Vehicle*   vehicle;
  //  vehicle = linuxEnvironment.getVehicle();
    
    cout<<"[robot init]robot model start to initialize!"<<endl;
   // RobotModel robotModel;
    robotModel.init(&linuxEnvironment);
    cout<<"[robot init]robot control model start to initializ!"<<endl;
    usleep(10000);
    ControlModel controlModel;
    controlModel.init(&robotModel);
    //数传线程
    cout<<"[robot init]radio start to listen!"<<endl;
    RadioInterface radio;
    

    cout<<"[robot init]robot serial port start to listen!"<<endl;
    //SerialListenThread serialListenThread;
    //serialListenThread.init(&robotModel,&controlModel);
    //serialListenThread.start();
    cout<<"serial listen ok!"<<endl;
    RadioListenThread radioListenThread;
    radioListenThread.init(&robotModel,&controlModel);
    radioListenThread.start();
   // cout<<"[robot init]robot init end!"<<endl;
    //debug模块
    //SerialPortDebug serialPortDebug;
    //serialPortDebug.init(robotModel.getpSerialInterface());
    //serialPortDebug.testSerialPort();
    //主逻辑
    signal(SIGINT,signal_exit_handler);
        signal(SIGABRT,signal_exit_handler);
    while(true){
        controlModel.processFSM();
	
    }
    //serialListenThread.join();
    cout<<"error end!"<<endl;
    //getchar();//防止监听线程意外结束直接退出。
    return 0;
}

