#ifndef     XBEE_H
#define     XBEE_H

#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include "hardware/uart.h"

#define SIZE_PARAM (6) // 6 bytes for each parameter
#define CANT_PARAM (14) // 12 parameters
#define PKT_SIZE (CANT_PARAM * SIZE_PARAM) 
#define PKT_TERMINATOR ('\n') // '\n' terminator

typedef struct {
    uart_inst_t * uart_id = uart1;
    uint32_t baud_rate = 115200;
    uint8_t stop_bit = 1; 
    uint8_t data_bits = 8;
    uart_parity_t parity = UART_PARITY_NONE;
} xbee_uart_cfg_t;

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
    FAILED,
} MissionStatus_t;

typedef enum {
    DPOINT_NONE = -1,
    DPOINT_ONE = 1,
    DPOINT_TWO = 2,
} dPoint_t;

typedef enum {
    MISSION_TIME = 0,
    PACKET_COUNT,
    STATUS,
    BATTERY_VOLTAGE,

    IMU_Y_VEL,
    IMU_ROLL,
    IMU_PITCH,

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
    public:
        XBee(xbee_uart_cfg_t cfg, uint32_t time = 0);
        ~XBee();

        void clearPkt(packet_index_t packet);
        
        void setMissionTime(uint32_t time);
        void setPacketCount();
        void setStatus(MissionStatus_t m_status);
        void setBatteryVoltage(char voltage); 
        
        void setIMUVerticalVel(int32_t vel);
        void setIMURoll(int16_t roll);
        void setIMUPitch(int16_t pitch);
        
        void setGNSSTime(uint8_t hour, uint8_t minute, uint8_t second);
        void setGNSSAltitude(int32_t altitude, int32_t altitudeMSL);
        void setGNSSLatitude(int32_t latitude);
        void setGNSSLongitude(int32_t longitude);
        // TODO: add satellite count
        
        void setBMEPressure(float pressure);
        void setBMETemperature(float temperature);
        void setBMEAltitude(float altitude);
        
        void parseMsg(packet_index_t packet, dPoint_t d_point = DPOINT_NONE);
        void sendParameter(packet_index_t packet);
        void sendPkt();
        
    private:
        char pkt[PKT_SIZE + 1] = {0}; // +1 for the '\n' terminator
    
        uint32_t start_time;
    
        xbee_uart_cfg_t uart_cfg;
        
        uint32_t mission_time = 0;
        uint16_t packet_count = 0;
        uint8_t battery_voltage = 0;
        uint16_t status = PRE_LAUNCH;
        
        uint32_t imu_y_vel = 0;
        int32_t imu_roll = 0;
        int32_t imu_pitch = 0;
    
        uint32_t gnss_time = 0;
        uint32_t gnss_altitude = 0;
        uint32_t gnss_latitude = 0;
        uint32_t gnss_longitude = 0;
    
        uint32_t bme_pressure = 0;
        uint32_t bme_altitude = 0;
        uint32_t bme_temperature = 0;
    
        void* data_arr[CANT_PARAM] = {
            &mission_time,
            &packet_count,
            &status,
            &battery_voltage,
            
            &imu_y_vel,
            &imu_roll,
            &imu_pitch,
            
            &gnss_time,
            &gnss_latitude,
            &gnss_longitude,
            &gnss_altitude,
            
            &bme_pressure,
            &bme_altitude,
            &bme_temperature,
        };

};    

#endif // XBEE_H
