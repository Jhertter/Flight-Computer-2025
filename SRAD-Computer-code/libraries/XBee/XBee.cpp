#include "XBee.h"

XBee::XBee(xbee_uart_cfg_t cfg, uint32_t time)
{
    uart_cfg = cfg;
    // Initialize UART
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    uart_init(uart_cfg.uart_id, uart_cfg.baud_rate);
    uart_set_hw_flow(uart_cfg.uart_id, false, false);
    uart_set_format(uart_cfg.uart_id, uart_cfg.data_bits, uart_cfg.stop_bit, uart_cfg.parity);
    pkt[PKT_SIZE] = '\n'; // Add the terminator to the end of the packet

    start_time = time;
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
void XBee::setMissionTime(uint32_t time)
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

void XBee::setStatus(uint16_t m_status)
{
    status = m_status;
    parseMsg(MISSION_STATUS);
}

void XBee::setBatteryVoltage(uint16_t voltage)
{
    battery_level = voltage;
    parseMsg(BATTERY_LEVEL, DPOINT_TWO);
}

void XBee::setIMUVerticalVel(int32_t vel)
{
    imu_y_vel = vel;
    parseMsg(IMU_Y_VEL);
}

void XBee::setIMUPitch(int16_t pitch)
{
    imu_pitch = pitch;
    parseMsg(IMU_PITCH, DPOINT_ONE);
}

void XBee::setIMURoll(int16_t roll)
{
    imu_roll = roll;
    parseMsg(IMU_ROLL, DPOINT_ONE);
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

/**
 * @brief Form
 * 
 * @param latitude 
 */
void XBee::setGNSSLatitude(int32_t latitude)
{
    uint8_t deg = ABS(latitude)/10000000;
    uint8_t min = (uint8_t)(((((float)ABS(latitude))/10000000)-deg)*60);
    uint8_t sec = (uint8_t)((((((float)ABS(latitude))/10000000)-deg)*60 - min)*60);

    gnss_latitude = deg * 10000 + min * 100 + sec;
    parseMsg(GNSS_LATITUDE);
}

/**
 * @brief Formats decimal degrees to degrees, minutes and seconds.
 * 
 * @param longitude 
 */
void XBee::setGNSSLongitude(int32_t longitude)
{
    uint8_t deg = ABS(longitude)/10000000;
    uint8_t min = (uint8_t)(((((float)ABS(longitude))/10000000)-deg)*60);
    uint8_t sec = (uint8_t)((((((float)ABS(longitude))/10000000)-deg)*60 - min)*60);

    gnss_longitude = deg * 10000 + min * 100 + sec;
    parseMsg(GNSS_LONGITUDE);
}

void XBee::setBMEPressure(uint32_t pressure)
{
    bme_pressure = pressure;
    parseMsg(BME_PRESSURE);
}

void XBee::setBMEAltitude(int32_t altitude)
{
    bme_altitude = altitude;
    parseMsg(BME_ALTITUDE);
}

void XBee::setBMETemperature(int16_t temperature)
{
    bme_temperature = temperature;
    parseMsg(BME_TEMPERATURE, DPOINT_TWO);
}

void XBee::sendPkt()
{
    for (int i=0; i<=PKT_SIZE; i++)
        uart_putc(uart_cfg.uart_id , pkt[i]);

#if DEBUG
    for(int i=0; i<=PKT_SIZE; i++)
        printf("%c", pkt[i]);
    printf("\n");
#endif
}

/**
 * @brief if needed to send a specific parameter, use this function
 * 
 * @param packet packet to send
 */
void XBee::sendParameter(packet_index_t packet)
{
    for (int i=0; i<SIZE_PARAM; i++)
    {
        // uart_putc(uart_cfg.uart_id, pkt[packet*SIZE_PARAM + i]);
        printf("%c", pkt[packet*SIZE_PARAM + i]);
    }
    // uart_putc(uart_cfg.uart_id, '\n');
    printf(" ");
}

/**
 * @brief   Parses data to char array format. User should verify that the desired
 *          significant figures fit (including decimal point and sign if needed)
 *          fit in the SIZE_PARAM.
 * 
 * @param packet index of the parameter to parse. Use the enum packet_index_t
 * @param d_point if needed, some parameters can use decimal point 
 */
void XBee::parseMsg(packet_index_t packet, dPoint_t d_point)
{
    clearPkt(packet);

    int32_t module = 0;
    bool negative = false;

    module = (*((int32_t*)(data_arr[packet])));
    uint32_t data = 0;
    
    if (module < 0)
    {
        negative = true;
        data = (uint32_t)(-module);
    }
    else
    {
        data = (*((uint32_t*)(data_arr[packet])));
        // printf("enum:%d:%d - ", packet, data);
    }
    
    for (int i=0; i<SIZE_PARAM; i++)    
    {
        module = ((int)(data))%10;
        pkt[packet*SIZE_PARAM + SIZE_PARAM-1-i] = module + '0';
        if (i == d_point)
            pkt[packet*SIZE_PARAM + SIZE_PARAM-1-i] = '.';
        else
            data *= 0.1;
    }

    if (negative)
        pkt[packet*SIZE_PARAM] = '-';
}

void XBee::clearPkt(packet_index_t packet)
{
    for (int i=0; i<SIZE_PARAM; i++)
        pkt[packet*SIZE_PARAM] = 0;
}

bool XBee::receiveStartSignal()
{
    static char c = '0';
    // printf("Waiting for start signal...\n");
    // if (uart_is_readable(uart_cfg.uart_id))
    // if (uart_is_readable_within_us(uart_cfg.uart_id, 1000)) // Wait for 1ms for a character
    // {
        c = uart_getc(uart_cfg.uart_id);
        printf("Received: %c\n", c);
        if(c == START_BYTE)
        {
            uart_puts(uart_cfg.uart_id, "ACK\n");
            return true;
        }
        return false;
    // }
    // else
    //     return false;
}

