#ifndef RADIO_LISTEN_THREAD_H
#define RADIO_LISTEN_THREAD_H

#include "base_thread.h"
#include "control_model.h"

class RadioListenThread:public BaseThread{
public:
    RadioListenThread();
    ~RadioListenThread();

public:
    void init(RobotModel *robotModel,ControlModel *controlModel);
    void run();

private:
    RobotModel *pRobotModel;
    ControlModel *pControlModel;
    bool mExitFlag;

};



#endif
