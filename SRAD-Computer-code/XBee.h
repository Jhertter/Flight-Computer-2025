#ifndef     XBEE_H
#define     XBEE_H


#define PACKET_SIZE (6) // 12 bytes for each sensor data
#define MSG_SIZE (12*PACKET_SIZE) 

// PACKET SHAPE: 
// <MISSION_TIME>,<PACKET_COUNT>,<STATUS>,<ALTITUDE>,<TILT>, <GPS_LATITUDE>,<GPS_LONGITUDE>,<GPS_ALTITUDE>,<ACCELERATION>,<TEMPERATURE>,<BATTERY_VOLTAGE>

enum MissionStatus
{
    PRE_LAUNCH = 0,
    LAUNCH,
    ASCENT,
    APOGEE,
    DESCENT,
    LANDING,
    RECOVERY,
    MISSION_COMPLETE,
    MISSION_FAILED
};

class XBee 
{
    private:
        char pkt[MSG_SIZE] = {0};
        
        char * ptr_mission_time = pkt;
        char * ptr_packet_count = pkt + PACKET_SIZE;
        char * ptr_status = pkt + (PACKET_SIZE *  2);
        char * ptr_altitude = pkt + (PACKET_SIZE * 3);
        char * ptr_tilt = pkt + (PACKET_SIZE * 4);
        char * ptr_absolute_time = pkt + (PACKET_SIZE * 5);
        char * ptr_gps_latitude = pkt + (PACKET_SIZE * 6); 
        char * ptr_gps_longitude = pkt + (PACKET_SIZE * 7);
        char * ptr_gps_altitude = pkt + (PACKET_SIZE * 8); 
        char * ptr_acceleration = pkt + (PACKET_SIZE * 9);
        char * ptr_temperature = pkt + (PACKET_SIZE * 10);
        char * ptr_battery_voltage = pkt + (PACKET_SIZE * 11);

        char mission_time;
        char packet_count;
        char status;
        char altitude;
        char tilt;
        char absolute_time;
        char gps_latitude;
        char gps_longitude;
        char gps_altitude;
        char acceleration;
        char temperature;
        char battery_voltage;

    public:
        XBee();
        ~XBee();

        void setMissionTime(char time);
        void setPacketCount();
        void setStatus(char status);
        void setAltitude(char altitude);
        void setTilt(char tilt);
        void setGpsLatitude(char latitude);
        void setGpsLongitude(char longitude);
        void setGpsAltitude(char altitude);
        void setAcceleration(char acceleration);
        void setTemperature(char temperature);
        void setBatteryVoltage(char voltage);

        void parseMsg();
        void sendPkt();
};

#endif // XBEE_H
