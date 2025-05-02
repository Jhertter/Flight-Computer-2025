#include "XBee.h"

XBee::XBee(long time)
{
    start_time = time;
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

/**
 * @brief sets elapsed time since start of the flight computer. Packet -> <hh:mm:ss>
 * 
 * 
 * @param time in ms 
 */
void XBee::setMissionTime(long time)
{
    long delta = (time - start_time)/1000;
    static int delta_seg = 0;
    static int delta_min = 0;
    static int delta_h = 0;
    
    delta_seg = delta%60;
    delta_min = (delta/60)%60;
    delta_h = (delta/(60*60))%60;
    
    this->mission_time = delta_seg + delta_min*100 + delta_h*10000;
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

void XBee::sendPkt()
{
    
}

void XBee::parseMsg()
{
    int d_point = DPOINT_NONE;
    int module = 0;
    
    float data = 0;
    
    for (int j=0; j<MSG_CANT; j++)
    {
        data = *data_arr[j];

        if ((((int)round(data*100))%10) > 0)
        {
            data *= 100;
            d_point = DPOINT_TWO;
        }
        else if (((int)(data*10))%10 > 0)
        {
            data *= 10;
            d_point = DPOINT_ONE;
        }
            
        for (int i=0; i<PACKET_SIZE; i++)
        {
            module = ((int)(data))%10;
            pkt[j*MSG_CANT + PACKET_SIZE-1-i] = module + '0';
            if (i == d_point)
            {
                pkt[j*MSG_CANT + PACKET_SIZE-1-i] = '.';
                i++;
            }
            data *= 0.1;
        }
    }
}