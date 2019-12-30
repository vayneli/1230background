#ifndef SENTRYDEMO_BASIC_TOOL_H
#define SENTRYDEMO_BASIC_TOOL_H
// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// Helpers
#include <sys/time.h>
#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
class BasicTool{
public:
    static struct timeval start;
    static struct timeval startInitGet();
    //获取当前时间距离程序启动时间的时间差
    static int currentTimeMsGet();
    Telemetry::Vector3f toEulerAngle(Quaternion quaternionData);
};

#endif //SENTRYDEMO_BASIC_TOOL_H
