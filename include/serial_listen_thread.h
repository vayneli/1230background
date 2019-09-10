#ifndef SERIAL_LISTEN_THREAD_H
#define SERIAL_LISTEN_THREAD_H

#include "base_thread.h"
#include "control_model.h"

class SerialListenThread:public BaseThread{
public:
    SerialListenThread();
    ~SerialListenThread();

public:
    void init(RobotModel *robotModel,ControlModel *controlModel);
    void run();

private:
    RobotModel *pRobotModel;
    ControlModel *pControlModel;
    bool mExitFlag;

};



#endif
