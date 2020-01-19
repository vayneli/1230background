#include "radio_listen_thread.h"
RadioListenThread::RadioListenThread() {
    mExitFlag= false;
}
RadioListenThread::~RadioListenThread(){

}
void RadioListenThread::init(RobotModel *robotModel,ControlModel *controlModel){
    pRobotModel=robotModel;
    pControlModel=controlModel;
};

void RadioListenThread::run() {
    RadioPacket recvPacket;
    while(exist){
        //是否需要考虑对监听数据构造队列，因为处理数据函数需要一定时间。
        if(pRobotModel->getpRadioInterface()->dataRecv(recvPacket)==0){
                //串口监听数据处理
		//cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
                pControlModel->radioListenDataProcess(recvPacket);

        }

    }
}
