#include "serial_listen_thread.h"
SerialListenThread::SerialListenThread() {
    mExitFlag= false;
}
SerialListenThread::~SerialListenThread(){

}
void SerialListenThread::init(RobotModel *robotModel,ControlModel *controlModel){
    pRobotModel=robotModel;
    pControlModel=controlModel;
};

void SerialListenThread::run() {
    SerialPacket recvPacket;
    while(!mExitFlag){
        //是否需要考虑对监听数据构造队列，因为处理数据函数需要一定时间。
        if(pRobotModel->getpSerialInterface()->dataRecv(recvPacket)==0){
                //串口监听数据处理
                pControlModel->serialListenDataProcess(recvPacket);
        }

    }
}