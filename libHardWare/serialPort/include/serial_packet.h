#ifndef RMSERIALPORTDEMO_SERIAL_PACKET_H
#define RMSERIALPORTDEMO_SERIAL_PACKET_H
#ifndef DJIOSDK_TELEMETRYSAMPLE_HPP
#define DJIOSDK_TELEMETRYSAMPLE_HPP
#include <dji_vehicle.hpp>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class SerialPacket{
public:
    SerialPacket();
    ~SerialPacket();

public:
    TypeMap<TOPIC_STATUS_FLIGHT>::type     flightStatus;
    TypeMap<TOPIC_GPS_FUSED>::type         latLon;
    TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude;
    TypeMap<TOPIC_RC>::type                rc;
    TypeMap<TOPIC_VELOCITY>::type          velocity;
    TypeMap<TOPIC_QUATERNION>::type        quaternion;

};


#endif //RMSERIALPORTDEMO_SERIAL_PACKET_H
#endif