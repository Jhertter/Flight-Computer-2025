#include "XBee.h"

XBee::XBee()
{
    mission_time = 0;
    packet_count = 0;;
    status = PRE_LAUNCH;
    altitude = 0;
    tilt = 0;

    absolute_time = 0;
    gps_latitude = 0;
    gps_longitude = 0;
    gps_altitude = 0;
    acceleration = 0;
    temperature = 0;
    battery_voltage = 0;


}

XBee::~XBee()
{
}

void XBee::setMissionTime(char time)
{

}

void XBee::setPacketCount()
{
    this->packet_count++;
}

void XBee::setStatus(char status)
{
    this->status = status;
}
void XBee::setAltitude(char altitude)
{
    this->altitude = altitude;
}

void XBee::setTilt(char tilt)
{
    this->tilt = tilt;
}

void XBee::setGpsLatitude(char latitude)
{
    this->gps_latitude = latitude;
}

void XBee::setGpsLongitude(char longitude)
{
    this->gps_longitude = longitude;
}

void XBee::setGpsAltitude(char altitude)
{
    this->gps_altitude = altitude;
}

void XBee::setAcceleration(char acceleration)
{
    this->acceleration = acceleration;
}

void XBee::setTemperature(char temperature)
{
    this->temperature = temperature;
}

void XBee::setBatteryVoltage(char voltage)
{
    this->battery_voltage = voltage;
}

void sendPkt()
{
    
}

void parseMsg()
{
    
}