#ifndef     XBEE_H
#define     XBEE_H

#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define DEBUG 0

#define PIN_UART_TX 24
#define PIN_UART_RX 25

#define SIZE_PARAM (6) // 6 bytes for each parameter
#define CANT_PARAM (14) // 12 parameters
#define PKT_SIZE (CANT_PARAM * SIZE_PARAM) 
#define PKT_TERMINATOR ('\n') // '\n' terminator
#define START_BYTE (0xED)


typedef struct {
    uart_inst_t * uart_id = uart1;
    uint32_t baud_rate = 115200;
    uint8_t stop_bit = 1; 
    uint8_t data_bits = 8;
    uart_parity_t parity = UART_PARITY_NONE;
} xbee_uart_cfg_t;

// PACKET SHAPE: 
// <MISSION_TIME>,<PACKET_COUNT>,<STATUS>,<ALTITUDE>,<TILT>, <GPS_LATITUDE>,<GPS_LONGITUDE>,<GPS_ALTITUDE>,<ACCELERATION>,<TEMPERATURE>,<battery_level>

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
    BATTERY_LEVEL,
    MISSION_STATUS,
    
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
        void setStatus(uint16_t m_status);
        void setBatteryVoltage(uint16_t voltage); 
        
        void setIMUVerticalVel(int32_t vel);
        void setIMURoll(int16_t roll);
        void setIMUPitch(int16_t pitch);
        
        void setGNSSTime(uint32_t time);
        void setGNSSAltitude(int32_t altitude, int32_t altitudeMSL);
        void setGNSSLatitude(int32_t latitude);
        void setGNSSLongitude(int32_t longitude);
        // TODO: add satellite count
        
        void setBMEPressure(uint32_t pressure);
        void setBMETemperature(int16_t emperature);
        void setBMEAltitude(int32_t altitude);
        
        void parseMsg(packet_index_t packet, dPoint_t d_point = DPOINT_NONE);
        void sendParameter(packet_index_t packet);
        void sendPkt();

        bool receiveStartSignal();
        
    private:
        char pkt[PKT_SIZE + 1] = {'0'}; // +1 for the '\n' terminator
    
        uint32_t start_time;
    
        xbee_uart_cfg_t uart_cfg;
        
        uint32_t mission_time = 0;
        // uint16_t packet_count = 0;
        // uint16_t status = LAUNCH;
        // uint16_t battery_level = 459;
        uint32_t packet_count = 0;
        uint32_t status = LAUNCH;
        uint32_t battery_level = 459;
        
        int32_t imu_y_vel = 0;
        int32_t imu_roll = 0;
        int32_t imu_pitch = 0;
    
        uint32_t gnss_time = 0;
        // uint16_t gnss_altitude = 0;
        uint32_t gnss_altitude = 0;
        uint32_t gnss_latitude = 0;
        uint32_t gnss_longitude = 0;
    
        uint32_t bme_pressure = 0;
        uint32_t bme_altitude = 0;
        uint32_t bme_temperature = 0;
    
        void* data_arr[CANT_PARAM] = {
            &mission_time,
            &packet_count,
            &battery_level,
            &status,
            
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