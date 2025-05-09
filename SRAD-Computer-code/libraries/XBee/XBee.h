#ifndef     XBEE_H
#define     XBEE_H

#include "math.h"
#include <stdint.h>

#define SIZE_PARAM (6) // 6 bytes for each parameter
#define CANT_PARAM (13) // 12 parameters
#define PKT_SIZE (CANT_PARAM * SIZE_PARAM) 

// PACKET SHAPE: 
// <MISSION_TIME>,<PACKET_COUNT>,<STATUS>,<ALTITUDE>,<TILT>, <GPS_LATITUDE>,<GPS_LONGITUDE>,<GPS_ALTITUDE>,<ACCELERATION>,<TEMPERATURE>,<BATTERY_VOLTAGE>

typedef enum {
    PRE_LAUNCH = 0,
    LAUNCH,
    ASCENT,
    APOGEE,
    DESCENT,
    LANDING,
    RECOVERY,
    COMPLETE,
    FAILED
} MissionStatus_t;

typedef enum {
    DPOINT_NONE = -1,
    DPOINT_ONE = 0,
    DPOINT_TWO = 1
} dPoint_t;

typedef enum {
    MISSION_TIME = 0,
    PACKET_COUNT,
    STATUS,
    BATTERY_VOLTAGE,

    IMU_ACCELERATION,
    IMU_TILT,

    GNSS_TIME,
    GNSS_LATITUDE,
    GNSS_LONGITUDE,
    GNSS_ALTITUDE,
    
    BME_PRESSURE,
    BME_ALTITUDE,
    BME_TEMPERATURE,
} packet_index_t;

class XBee 
{
    private:
        char pkt[CANT_PARAM][SIZE_PARAM] = {0};
        char * ptr_pkt = &pkt[0][0];
        
        char * ptr_mission_time = ptr_pkt + (SIZE_PARAM *  MISSION_TIME);
        char * ptr_packet_count = ptr_pkt + (SIZE_PARAM *  PACKET_COUNT);
        char * ptr_status = ptr_pkt + (SIZE_PARAM *  STATUS);
        char * ptr_battery_voltage = ptr_pkt + (SIZE_PARAM * BATTERY_VOLTAGE);

        char * ptr_acceleration = ptr_pkt + (SIZE_PARAM * IMU_ACCELERATION);
        char * ptr_tilt = ptr_pkt + (SIZE_PARAM * IMU_TILT);
        
        char * ptr_gnss_time = ptr_pkt + (SIZE_PARAM * GNSS_TIME);
        char * ptr_gnss_latitude = ptr_pkt + (SIZE_PARAM * GNSS_LATITUDE); 
        char * ptr_gnss_longitude = ptr_pkt + (SIZE_PARAM * GNSS_LONGITUDE);
        char * ptr_gnss_altitude = ptr_pkt + (SIZE_PARAM * GNSS_ALTITUDE); 
        
        char * ptr_pressure = ptr_pkt + (SIZE_PARAM * BME_PRESSURE);
        char * ptr_altitude = ptr_pkt + (SIZE_PARAM * BME_ALTITUDE);
        char * ptr_temperature = ptr_pkt + (SIZE_PARAM * BME_TEMPERATURE);

        char start_time;
        
        // Dont know if i need the actual values, just in case
        uint32_t mission_time;
        uint16_t packet_count;
        uint8_t battery_voltage;
        uint16_t status;
        
        uint32_t imu_acceleration;
        uint32_t imu_tilt;

        uint32_t gnss_time;
        uint32_t gnss_altitude;
        uint32_t gnss_latitude;
        uint32_t gnss_longitude;

        uint32_t bme_pressure;
        uint32_t bme_altitude;
        uint32_t bme_temperature;

        void* data_arr[CANT_PARAM] = {
            &mission_time,
            &packet_count,
            &status,
            &battery_voltage,
            
            &imu_acceleration,
            &imu_tilt,
            
            &gnss_time,
            &gnss_latitude,
            &gnss_longitude,
            &gnss_altitude,
            
            &bme_pressure,
            &bme_altitude,
            &bme_temperature,
        };

    public:
        XBee(long time);
        ~XBee();

        void setMissionTime(long time);
        void setPacketCount();
        void setStatus(MissionStatus_t m_status);
        void setBatteryVoltage(char voltage); 
        
        void setIMUAcceleration(char acceleration);
        void setIMUTilt(float tilt);
        
        void setGNSSTime(uint32_t time);
        void setGNSSAltitude(int32_t altitude, int32_t altitudeMSL);
        void setGNSSLatitude(int32_t latitude);
        void setGNSSLongitude(int32_t longitude);
        // TODO: add satellite count

        void setBMEPressure(float pressure);
        void setBMETemperature(float temperature);
        void setBMEAltitude(float altitude);

        void parseMsg(packet_index_t packet, dPoint_t d_point = DPOINT_NONE);
        void sendPkt();
};

#endif // XBEE_H
