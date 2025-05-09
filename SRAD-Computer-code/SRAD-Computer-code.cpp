#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
// #include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
// #include "hardware/uart.h"
#include "hardware/flash.h" //para programar y borrar flash
#include "hardware/sync.h"  //para interrupciones
#include "libraries/icm_20948/pico-icm20948.h"
#include "libraries/MadgwickAHRS/MadgwickAHRS.h"
#include "libraries/sam_m8q_v2/sam_m8q_v2.h"
#include "libraries/bmp_280/bmp280.h"
#include "libraries/XBee/XBee.h"

#define I2C_FREQUENCY 100000

#define PIN_LED_COM 11
#define PIN_LED_ERROR 10
#define PIN_LED_ALTITUDE 9

#define PIN_I2C_SDA 16
#define PIN_I2C_SCL 17

#define PIN_UART_TX 24
#define PIN_UART_RX 25

#define UART_ID uart1
#define UART_BAUD 9600
#define UART_STOP_BITS 1
#define UART_DATA_BITS 8
#define UART_PARITY UART_PARITY_NONE

// #define PIN_SD_CLK 20
// #define PIN_SD_CMD 21
// #define PIN_SD_DAT0 19
// #define PIN_SD_DAT1 25
// #define PIN_SD_DAT2 23
// #define PIN_SD_DAT3 22

#define FLASH_OFFSET (0x01000000) //definimos un offset para no pisar nuestro código

const static int gpio_size = 3;
const static int gpio_array[] = {
    PIN_LED_COM,
    PIN_LED_ALTITUDE,
    PIN_LED_ERROR};

i2c_inst_t i2c_setUp = {i2c0_hw, false};
icm20948_config_t IMU_config = {0x69, 0x0C, &i2c_setUp};
icm20984_data_t data;
madgwick_ahrs_t filter = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};

BMP280 BME(&i2c_setUp);

SFE_UBLOX_GNSS GNSS;

xbee_uart_cfg_t uart_cfg = {UART_ID, UART_BAUD, UART_STOP_BITS, UART_DATA_BITS, UART_PARITY};
XBee xbee(uart_cfg, to_ms_since_boot(get_absolute_time()));

const uint8_t * flashBase = (const uint8_t *)(XIP_BASE + FLASH_OFFSET); //deprecated

typedef struct{
    float pressure = 0;
    float temperature = 0;
    float altitudeBME = 0;
    
    uint32_t GNSS_time = 0;
    uint8_t satellite_count = 0;
    int32_t latitude = 0;     // latitude +-90ª
    int32_t longitude = 0;   // longitude +-180ª
    int32_t altitude = 0;
    int32_t altitude_MSL = 0;
    
    uint32_t imu_acceleration = 0;
    uint32_t imu_tilt = 0;
} packet;

#define BUFFER_SIZE ((uint8_t)(FLASH_PAGE_SIZE/sizeof(packet)))
#define FLASH_SIZE ((uint32_t)(16 * 1024 * 1024))

packet parameters;
packet test;

bool read_imu(repeating_timer_t *rt);
bool read_gnss(repeating_timer_t *rt);
bool read_bme(repeating_timer_t *rt);

void gpio_toggle(int pin);
void init_leds();

packet buffer_flash[BUFFER_SIZE] = {0};
uint32_t saveData(packet data);

int main()
{
    test.altitude = 99;
    test.altitude_MSL = 48;
    test.altitudeBME = 1.5f;
    test.GNSS_time = 33;
    test.imu_acceleration = 100;
    test.imu_tilt = 87;
    test.latitude = 66;
    test.longitude = 77;
    test.pressure = 88;
    test.satellite_count = 5;
    test.temperature = 34;
    
    stdio_init_all();
    init_leds();
    
    // Initialize I2C Bus
    gpio_init(PIN_I2C_SCL);
    gpio_init(PIN_I2C_SDA);
    i2c_init(&i2c_setUp, I2C_FREQUENCY); //320kHz
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);
    
    sleep_ms(5000);

    // Initialize ICM-20948
    // printf("Initializing IMU...\n");
    // if (!icm20948_init(&IMU_config))
    // {
    //     gpio_put(PIN_LED_ERROR, 1);
    //     printf("IMU not detected at I2C address. Freezing");
    // }
    // else 
    // {
    //     gpio_put(PIN_LED_COM, 1);
    //     printf("IMU initialized.\n");
    // }
    
    // icm20948_cal_gyro(&IMU_config, &data.gyro_bias[0]);
    // printf("calibrated gyro: %d %d %d\n", data.gyro_bias[0], data.gyro_bias[1], data.gyro_bias[2]);

    // icm20948_cal_accel(&IMU_config, &data.accel_bias[0]);
    // printf("calibrated accel: %d %d %d\n", data.accel_bias[0], data.accel_bias[1], data.accel_bias[2]);
    
    
    // // Initialize BME280
    // printf("Initializing ESU...\n");
    // if(BME.begin(0x76, 0x60) == false)
    // {
    //     printf("ESU not detected at default I2C address. Please check wiring. Freezing.");
    //     gpio_put(PIN_LED_ERROR, 1);
    //     while (1);
    // }
    // else
    // {
    //     gpio_put(PIN_LED_ALTITUDE, 1);
    //     printf("ESU initialized.\n");
    // }
    
    // Initialize SAM_M8Q
    // printf("Initializing GNSS...\n");
    // if (GNSS.begin(i2c_setUp, 0x42) == false) // Connect to the Ublox module using Wire port
    // {
    //     printf("GNSS not detected at default I2C address. Please check wiring. Freezing.");
    //     gpio_put(PIN_LED_ERROR, 1);
    //     while (1);
    // }
    // else
    // {
    //     // printf("GNSS initialized.\n");
    //     GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS, 11000);
    // }
    // printf("All sensors initialized without errors.\n");
    gpio_put(PIN_LED_ALTITUDE, 1);
    
    // static repeating_timer_t timer_imu;
    // add_repeating_timer_ms(30, &read_imu, &parameters, &timer_imu);
    
    // static repeating_timer_t timer_gnss;
    // add_repeating_timer_ms(200, &read_gnss, &parameters, &timer_gnss);
    
    // static repeating_timer_t timer_bme;
    // add_repeating_timer_ms(30, &read_bme, &parameters, &timer_bme);
    
    for(uint8_t i = 0; i<19; i++)
    {
        saveData(test);
    }

    packet* test_ptr = (packet*)(XIP_BASE + (FLASH_SIZE) - FLASH_PAGE_SIZE);
    for(uint8_t j = 0; j < 19; j++)
    {
        if(j % BUFFER_SIZE == 0)
        {
            test_ptr = (packet*)((FLASH_SIZE) - FLASH_PAGE_SIZE - FLASH_PAGE_SIZE * j/BUFFER_SIZE);
        }
        printf("Packet %d: ", j);
        printf("Packet %d: ", test_ptr->GNSS_time);
        printf("Lat: %ld - ", test_ptr->latitude);
        printf("Long: %ld - ", test_ptr->longitude);
        printf("Alt: %d (mm) - ", test_ptr->altitude);
        printf("AltMSL: %d (mm) - ", test_ptr->altitude_MSL);
        printf("Pressure: %f (hPa) - ", test_ptr->pressure/100);
        printf("Temperature: %f (C) - ", test_ptr->temperature);
        printf("Alt BME: %f (m)\n", test_ptr->altitudeBME);
        test_ptr += j;
    }

    while (true)
    {
        static long lastTime = 0;
        static char byte = 0;

        if (to_ms_since_boot(get_absolute_time()) - lastTime > 500)
        {
            lastTime = to_ms_since_boot(get_absolute_time()); // Update the timer

            xbee.setMissionTime(to_ms_since_boot(get_absolute_time()));
            xbee.setGNSSTime(GNSS.getHour(), GNSS.getMinute(), GNSS.getSecond());
            xbee.setGNSSLatitude(GNSS.getLatitude());

            xbee.sendPkt(MISSION_TIME);
            // xbee.sendPkt(GNSS_LATITUDE);
            // xbee.sendPkt(GNSS_TIME);
        }
    }
}

void gpio_toggle(int pin)
{
    if (gpio_get(pin))
        gpio_put(pin, 0);
    else
        gpio_put(pin, 1);
}

bool read_imu(repeating_timer_t *rt)
{
    static int16_t accel_raw[3] = {0};
    static int16_t gyro_raw[3] = {0};
    static int16_t mag_raw[3] = {0};
    static int16_t temp_raw = 0;
    static float accel_g[3] = {0};
    static float gyro_dps[3] = {0};
    static float mag_ut[3] = {0};

    icm20948_read_cal_accel(&IMU_config, &data.accel_raw[0], &data.accel_bias[0]);
    icm20948_read_cal_gyro(&IMU_config, &data.gyro_raw[0], &data.gyro_bias[0]);
    icm20948_read_cal_mag(&IMU_config, &data.mag_raw[0], &data.mag_bias[0]);
    // icm20948_read_temp_c(&IMU_config, &data.temp_c);

    accel_g[0] = (float)data.accel_raw[0] / 16384.0f;
    accel_g[1] = (float)data.accel_raw[1] / 16384.0f;
    accel_g[2] = (float)data.accel_raw[2] / 16384.0f;
    gyro_dps[0] = (float)data.gyro_raw[0] / 131.0f;
    gyro_dps[1] = (float)data.gyro_raw[1] / 131.0f;
    gyro_dps[2] = (float)data.gyro_raw[2] / 131.0f;
    mag_ut[0] = (float)data.mag_raw[1];
    mag_ut[1] = (float)-data.mag_raw[0];
    mag_ut[2] = (float)-data.mag_raw[2];

    MadgwickAHRSupdate(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8, mag_ut[0], mag_ut[1], mag_ut[2]);

    return true;
}

bool read_gnss(repeating_timer_t *rt)
{
    parameters.GNSS_time = GNSS.getHour() * 10000 + GNSS.getMinute() * 100 + GNSS.getSecond();
    parameters.satellite_count = GNSS.getSIV();
    parameters.latitude = GNSS.getLatitude();     // latitude +-90ª
    parameters.longitude = GNSS.getLongitude();   // longitude +-180ª
    parameters.altitude = GNSS.getAltitude();
    parameters.altitude_MSL = GNSS.getAltitudeMSL();

    
    return true;
}

bool read_bme(repeating_timer_t *rt)
{
    parameters.pressure = BME.readPressure();
    parameters.temperature = BME.readTemperature();
    parameters.altitudeBME = BME.readAltitude(1013.25);
    
    return true;
}

void init_leds()
{
    for (int i = 0; i < gpio_size; i++)
    {
        gpio_init(gpio_array[i]);
        gpio_set_dir(gpio_array[i], GPIO_OUT);
        gpio_put(gpio_array[i], 0);
    }

    for (int i = 0; i < 6; i++)
    {
        gpio_toggle(PIN_LED_ALTITUDE);
        sleep_ms(100);
        gpio_toggle(PIN_LED_ERROR);
        sleep_ms(100);
        gpio_toggle(PIN_LED_COM);
        sleep_ms(100);
    }
}
/*
Función que guarda estructuras en un buffer, y cuando este se llena, guarda este
buffer en memoria flash.
Recibe: La estructura con los datos actuales del sistema (Posición, altura, etc)
Devuelve: La cantidad de veces que se escribió un sector en memoria
*/
uint32_t saveData(packet data)
{
    static uint8_t buff_count = 0;
    static uint32_t saves = 0;

    printf("Saving data...\n");
    sleep_ms(100);
    printf("Buffer count: %d    ", buff_count);
    sleep_ms(100);
    printf("Saves: %d\n", saves);
    sleep_ms(100);

    if(buff_count < BUFFER_SIZE)
    {
        buffer_flash[buff_count] = data;
        buff_count++;
    }
    else
    {
        //La idea es borrar los sectores a medida que necesitemos. Entran 16 páginas 
        //(escrituras), por lo que cada 16 iteraciones borramos el siguiente sector
        if(saves % 16 == 0) 
        {
            printf("Erasing sector %d\n", saves / 16);
            sleep_ms(100);
            uint32_t erase_addr = (FLASH_SIZE) - (uint32_t)(FLASH_SECTOR_SIZE * ((saves / 16) + 1));

            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(erase_addr, FLASH_SECTOR_SIZE);
            restore_interrupts (ints);
        }
        //Después de borrar (si fue necesario), guardamos el buffer en memoria
        printf("Writing buffer %d\n", saves);
        sleep_ms(100);
        uint32_t prog_addr = (FLASH_SIZE) - (FLASH_PAGE_SIZE * saves);

        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(prog_addr, (uint8_t*)(buffer_flash), FLASH_PAGE_SIZE);
        restore_interrupts (ints);

        saves++;
        buff_count = 0;
    }
    return saves;
} //TODO: Fijarse que no sobreescriba la sección de código (contemplar ese caso)
    //TODO: hardcodear el archivo build/pico_flash_region.ld