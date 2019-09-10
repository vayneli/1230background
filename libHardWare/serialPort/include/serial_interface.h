#include "serial_packet.h"
#include <cmath>
// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
#include <iostream>
// Helpers
#include "basic_tool.h"
#ifndef RMDEMO_SERIAL_INTERFACE_H
#define RMDEMO_SERIAL_INTERFACE_H
#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
//协议

typedef enum:unsigned char {
    CMD_SERIAL_CHASSIS_STOP_SET = 0x01,
    CMD_SERIAL_YUNTAI_DELTA_SET=0x10,
    CMD_SERIAL_SHOOT=0x12,
    CMD_SERIAL_DATA_UPDATE=0xb0,
    CMD_SERIAL_MINIPC_SHUTDOWN=0xc1,
} SerialPortCMD;


class SerialInterface
{
public:
    SerialInterface();
    ~SerialInterface();

private:
    Vehicle*   mVehicle;
    BasicTool basictool;
public:
    /** 初始化函数
    *  @param:  std::string devPath :串口设备路径
    *  @return: int :错误号，0代表无错误，１代表发生错误。
    */
    int init(Vehicle* vehicle);
    //查询串口是否打开
    bool isOpen();

    /** 命令数据接收函数
     *根据协议数据帧格式，封装串口接收函数，包括协议帧的检查机制。
     *  @return: int :错误号，0代表无错误，１代表发生错误。
     *  @note:   数据包构成参考serial_packet.h/cpp
     */
    int dataRecv(SerialPacket &recvPacket);



    /** Monitored Takeoff
    This implementation of takeoff  with monitoring makes sure your aircraft
    actually took off and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
    */
    bool Takeoff( int timeout = 1);


    /** Very simple calculation of local NED offset between two pairs of GPS
    * coordinates.
    *
    * Accurate when distances are small.
    */
    bool Land( int timeout = 1);
    void movebyVelocity(float32_t Vx, float32_t Vy, float32_t Vz, float32_t yawRate);
    /** Position Control. Allows you to set an offset from your current
    *   location. The aircraft will move to that position and stay there.
    *   Typical use would be as a building block in an outer loop that does not
    *   require many fast changes, perhaps a few-waypoint trajectory. For smoother
    *   transition and response you should convert your trajectory to attitude
    *   setpoints and use attitude control or convert to velocity setpoints
    *   and use velocity control.
    */
    void moveByPositionOffset(float32_t x, float32_t y,float32_t z,float32_t Yaw);
    bool moveByPositionOffset_block(float xOffsetDesired,
                     float yOffsetDesired, float zOffsetDesired,
                     float yawDesired, float posThresholdInM,
                   float yawThresholdInDeg);
   

};



#endif
