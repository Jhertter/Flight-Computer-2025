#include "XBee.h"

XBee::XBee(long time)
{
    start_time = time;
    mission_time = 0;
    packet_count = 0;;
    status = PRE_LAUNCH;
    battery_voltage = 0;

    imu_acceleration = 0;
    imu_tilt = 0;
    
    gnss_time = 0;
    gnss_latitude = 0;
    gnss_longitude = 0;
    gnss_altitude = 0;

    bme_pressure = 0;
    bme_altitude = 0;
    bme_temperature = 0;
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
    static int delta_seg = 0, delta_min = 0, delta_h = 0;
    
    delta_seg = delta%60;
    delta_min = (delta/60)%60;
    delta_h = (delta/(60*60))%60;
    
    mission_time = delta_seg + delta_min*100 + delta_h*10000;

    parseMsg(MISSION_TIME);
}

void XBee::setPacketCount()
{
    packet_count++;
    parseMsg(PACKET_COUNT);
}

void XBee::setStatus(MissionStatus_t m_status)
{
    status = m_status;
}

void XBee::setBatteryVoltage(char voltage)
{
    battery_voltage = voltage;
}

void XBee::setIMUAcceleration(char acceleration)
{
    imu_acceleration = acceleration;
}

void XBee::setIMUTilt(float tilt)
{
    imu_tilt = tilt;
}

void XBee::setGNSSTime(uint32_t time)
{
    gnss_time = time;
    parseMsg(GNSS_TIME);
}

#define ABS(x) ((x) < 0 ? -(x) : (x))

void XBee::setGNSSAltitude(int32_t altitude, int32_t altitudeMSL)
{
    gnss_altitude = altitude;
    // gnss_altitude_MSL = altitudeMSL;
    parseMsg(GNSS_ALTITUDE);
}

void XBee::setGNSSLatitude(int32_t latitude)
{
    uint8_t deg = ABS(latitude)/10000000;
    uint8_t min = (uint8_t)(((((float)ABS(latitude))/10000000)-deg)*60);
    uint8_t sec = (uint8_t)((((((float)ABS(latitude))/10000000)-deg)*60 - min)*60);

    gnss_latitude = deg * 10000 + min * 100 + sec;
    parseMsg(GNSS_LATITUDE);
}

void XBee::setGNSSLongitude(int32_t longitude)
{
    uint8_t deg = ABS(longitude)/10000000;
    uint8_t min = (uint8_t)(((((float)ABS(longitude))/10000000)-deg)*60);
    uint8_t sec = (uint8_t)((((((float)ABS(longitude))/10000000)-deg)*60 - min)*60);

    gnss_longitude = deg * 10000 + min * 100 + sec;
    parseMsg(GNSS_LONGITUDE);
}

void XBee::setBMEPressure(float pressure)
{
    bme_pressure = pressure;
    parseMsg(BME_PRESSURE);
}

void XBee::setBMEAltitude(float altitude)
{
    bme_altitude = altitude;
    parseMsg(BME_ALTITUDE);
}

void XBee::setBMETemperature(float temperature)
{
    bme_temperature = temperature;
    parseMsg(BME_TEMPERATURE, DPOINT_TWO);
}

void XBee::sendPkt()
{
    
}

/**
 * @brief 
 * 
 * @param packet index of the parameter to parse. Use the enum packet_index_t
 * @param d_point if needed, some parameters can use decimal point 
 */
void XBee::parseMsg(packet_index_t packet, dPoint_t d_point)
{
    int module = 0;
    
    uint32_t data = (*((uint32_t*)(data_arr[packet])));
    
    // Checks if param has decimal places
    if (d_point == DPOINT_TWO)
    {
        data *= 100;
        d_point = DPOINT_TWO;
    }
    else if (d_point == DPOINT_ONE)
    {
        data *= 10;
        d_point = DPOINT_ONE;
    }
        
    for (int i=0; i<SIZE_PARAM; i++)    
    {
        module = ((int)(data))%10;
        pkt[packet][SIZE_PARAM-1-i] = module + '0';
        if (i == d_point)
        {
            pkt[packet][SIZE_PARAM-1-i] = '.';
            i++;
        }
        data *= 0.1;
    }
}