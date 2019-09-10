#include "serial_packet.h"
#include "serial_interface.h"
#include <iostream>
#include <dji_linux_helpers.hpp>
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char** argv)
{
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle* vehicle;
    vehicle = linuxEnvironment.getVehicle();
    cout<<"start to test serial port,press 'c' to continue,press 'q' to quit!"<<endl;
    char c=getchar();
    cout<<"===========串口测试==========="<<endl;

    cout<<"w:           1m/s x轴"<<endl;
    cout<<"s:           -1m/s x轴"<<endl;
    cout<<"a:           1m/s  y轴"<<endl;
    cout<<"d:           -1m/s y轴"<<endl;
    cout<<"q:           1m/s  z轴"<<endl;
    cout<<"z:           -1m/s z轴"<<endl;
    cout<<"x:           获取数据  "<<endl;
    cout<<"i:            1m   x轴"<<endl;
    cout<<"k:            -1m  x轴"<<endl;
    cout<<"j:            1m   y轴"<<endl;
    cout<<"l:            -1m  y轴"<<endl;
    cout<<"u:             1m  z轴"<<endl;
    cout<<"m:             -1m z轴"<<endl;
    SerialInterface serial;
    SerialPacket packet;
    
    if(serial.init(vehicle)==0) {
         cout<<"[robot model init ]: RobotSerialInterface init successed!"<<endl;
     } else{
         cout<<"[robot model init ]: RobotSerialInterface init failed!"<<endl;
     }
    while(true){
        c = getchar();
        switch(c){
            case 'n':
                cout<<"takeoff"<<endl;
                serial.Takeoff(10);
                break;
            case 'w':
                cout<<"movebyVelocity(1,0,0,0)"<<endl;
                serial.movebyVelocity(1.0,0,0,0);
                cout<<"please input..."<<endl;
                break;
            case 's':
                cout<<"movebyVelocity(-1,0,0,0)"<<endl;
                serial.movebyVelocity(-1.0,0,0,0);
                cout<<"please input..."<<endl;
                break;
            case 'a':
                cout<<"movebyVelocity(0,1.0,0,0)"<<endl;
                serial.movebyVelocity(0,1.0,0,0);
                cout<<"please input..."<<endl;
                break;
            case 'd':
                cout<<"movebyVelocity(0,-1.0,0,0)"<<endl;
                serial.movebyVelocity(0,-1.0,0,0);
                cout<<"please input..."<<endl;
                break;           
            case 'q':
                cout<<"movebyVelocity(0,0.0,2,0)"<<endl;
                serial.movebyVelocity(0,0.0,2,0);
                cout<<"please input..."<<endl;
                break;     
            case 'x':
                cout<<"get message "<<endl;
                serial.dataRecv(packet);
                std::cout << "Flight Status                         = " << (int)packet.flightStatus<< "\n";
                std::cout << "Position              (LLA)           = " << packet.latLon.latitude
                << ", " << packet.latLon.longitude << ", " << packet.altitude << "\n";
                std::cout << "RC Commands           (r/p/y/thr)     = " << packet.rc.roll << ", "
                << packet.rc.pitch << ", " << packet.rc.yaw << ", " << packet.rc.throttle << "\n";
                std::cout << "Velocity              (vx,vy,vz)      = " << packet.velocity.data.x
                << ", " << packet.velocity.data.y << ", " << packet.velocity.data.z << "\n";
                std::cout << "Attitude Quaternion   (w,x,y,z)       = " << packet.quaternion.q0
                << ", " << packet.quaternion.q1 << ", " << packet.quaternion.q2 << ", "
                << packet.quaternion.q3 << "\n";

        }
    }
    return 0;
}
