#ifndef SRAD_computer 
#define SRAD_computer

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/flash.h" //para programar y borrar flash
#include "hardware/sync.h"  //para interrupciones
#include "libraries/icm_20948/pico-icm20948.h"
#include "libraries/MadgwickAHRS/MadgwickAHRS.h"
#include "libraries/sam_m8q_v2/sam_m8q_v2.h"
#include "libraries/bmp_280/bmp280.h"
#include "libraries/XBee/XBee.h"

#define PRINT_DEBUG_PKT 0

#define I2C_FREQUENCY 400000

#define PIN_LED_ON 11
#define PIN_LED_ERROR 10
#define PIN_LED_ALTITUDE 9

#define PIN_I2C_SDA 16
#define PIN_I2C_SCL 17

#define PIN_UART_TX 24
#define PIN_UART_RX 25

#define PIN_AIRBRAKE 29

#define UART_ID uart1
#define UART_BAUD 115200
#define UART_STOP_BITS 1
#define UART_DATA_BITS 8
#define UART_PARITY UART_PARITY_NONE

#define FLASH_OFFSET (0x01000000) // definimos un offset para no pisar nuestro código
#define FLASH_CODE_END ((uint32_t)(XIP_BASE + FLASH_OFFSET)) //Hardcodear cuando tengamos el código final

const static int gpio_size = 3;
const static int gpio_array[] = {
    PIN_LED_ON,
    PIN_LED_ALTITUDE,
    PIN_LED_ERROR};

const static int16_t accel_bias [3] = {96, -94, 5};
static int16_t altitude_bias = 0;

i2c_inst_t i2c_setUp = {i2c0_hw, false};
icm20948_config_t IMU_config = {0x69, 0x0C, &i2c_setUp};
icm20984_data_t data;
madgwick_ahrs_t filter = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};
BMP280 BME(&i2c_setUp);
SFE_UBLOX_GNSS GNSS;


xbee_uart_cfg_t uart_cfg = {UART_ID, UART_BAUD, UART_STOP_BITS, UART_DATA_BITS, UART_PARITY};

const uint8_t *flashBase = (const uint8_t *)(XIP_BASE + FLASH_OFFSET);

typedef struct
{
    uint8_t status;
    uint8_t battery_level = 0;

    uint32_t bme_pressure = 0;
    uint32_t bme_temperature = 0;
    uint32_t bme_altitude = 0;

    uint8_t satellite_count = 0;
    uint32_t gnss_time = 0;
    int32_t gnss_latitude = 0;  // latitude +-90ª
    int32_t gnss_longitude = 0; // longitude +-180ª
    int16_t gnss_altitude = 0;
    int32_t gnss_altitude_MSL = 0;

    int16_t imu_roll = 0;
    int16_t imu_pitch = 0;
    int32_t imu_accel_y = 0;
    int32_t imu_vel_y = 0;
} packet;

typedef struct {
    uint8_t last_status;
    int32_t intial_lat;
    int32_t intial_long;
    int32_t final_lat;
    int32_t final_long;
    uint32_t mission_duration;
    int16_t max_altitude;
    uint32_t cant_pages;
} flash_global_vars_t;

// typedef enum {
// 	RESET_MISSION = 0,
// 	MOVE,
// 	STAY,
// } fsm_actions_t;

#define BUFFER_SIZE ((uint8_t)(FLASH_PAGE_SIZE / sizeof(packet)))
#define FLASH_SIZE ((uint32_t)(16 * 1024 * 1024))



void read_data();
void update_xbee_parameters(uint32_t last_time);

void calibrate_bme(void);

void read_imu();
void read_gnss();
void read_bme();
static float ground_hP = 0;

void gpio_toggle(int pin);
void init_leds();

packet buffer_flash[BUFFER_SIZE];
uint32_t saveData(packet data);
void readData(uint32_t n_buffers);
void saveGlobalData(flash_global_vars_t data);

/*******************
 *  State machine
 *******************/
void waitForStart(void);
void calibrateRocket(void);
void waitForLaunch(void);
void telemetry(void);
void ascentRoutine(void);
void apogeeRoutine(void);
void descentRoutine(void);
void recoverySignal(void);
void standByMode(void); 
void airbrake_payload(bool payload = false);

#endif 