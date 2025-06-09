#ifndef SRAD_computer 
#define SRAD_computer

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/flash.h" //para programar y borrar flash
#include "hardware/sync.h"  //para interrupciones
#include "hardware/adc.h"

#include "libraries/icm_20948/pico-icm20948.h"
#include "libraries/MadgwickAHRS/MadgwickAHRS.h"
#include "libraries/sam_m8q_v2/sam_m8q_v2.h"
#include "libraries/bmp_280/bmp280.h"
#include "libraries/XBee/XBee.h"

#define NDEBUG_SAT          1
#define PRINT_DEBUG_PKT     0

#define I2C_FREQUENCY       400000

#define PIN_LED_ON          11
#define PIN_LED_ERROR       10
#define PIN_LED_ALTITUDE    9

#define PIN_I2C_SDA         16
#define PIN_I2C_SCL         17

#define PIN_AIRBRAKE        29
#define PIN_ADC_BATTERY     28

#define UART_ID             uart1
#define UART_BAUD           115200
#define UART_STOP_BITS      1
#define UART_DATA_BITS      8
#define UART_PARITY         UART_PARITY_NONE

#define FLASH_OFFSET        (0x01000000) // definimos un offset para no pisar nuestro código
#define FLASH_CODE_END      ((uint32_t)(XIP_BASE + FLASH_OFFSET)) //Hardcodear cuando tengamos el código final

#define AIRBRAKE_HEIGHT     2000    // Height in meters

#define ABS(x)              ((x) < 0 ? -(x) : (x))
#define RAD_TO_DEG          (180.0f / 3.14159265358979323846f)
#define DEG_TO_RAD          (1 / RAD_TO_DEG)
#define GRAVITY             (9.80665f)
#define IMU_SAMPLING_RATE   (200.0f)                     // Hz
#define IMU_SAMPLING_PERIOD (1.0f / IMU_SAMPLING_RATE) // seconds
#define STATIC_ACCELEROMETER_THRESHOLD (0.12f)         // m/s^2
#define STATIC_GYROSCOPE_THRESHOLD (3.0f)              // m/s

const int gpio_size = 3;
const int gpio_array[] = {
    PIN_LED_ON,
    PIN_LED_ALTITUDE,
    PIN_LED_ERROR};

const int16_t accel_bias [3] = {96, -94, 5};
int16_t altitude_bias = 0;

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
    uint32_t status;
    uint32_t battery_level = 0;

    uint32_t bme_pressure = 0;
    int32_t bme_temperature = 0;
    int32_t bme_altitude = 0;

    uint8_t satellite_count = 0;
    uint32_t gnss_time = 0;
    int32_t gnss_latitude = 0;  // latitude +-90ª
    int32_t gnss_longitude = 0; // longitude +-180ª
    int32_t gnss_altitude = 0;
    int32_t gnss_altitude_MSL = 0;
    int32_t gnss_velocity = 0;

    int32_t imu_roll = 0;
    int32_t imu_pitch = 0;
    int32_t imu_vel_y = 0;  // cm/s
    int32_t imu_accel_y = 0; // cm/s^2
} packet;

typedef struct {
    uint8_t last_status;
    int32_t initial_lat;
    int32_t initial_long;
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

void readData();
void updateXbeeParameters(uint32_t last_time);

void readIMU();
void readGNSS();
void readBME();
void calibrateBME(void);
static float ground_hP = 0;

void gpio_toggle(int pin);
void init_leds();
void error_led(void);

packet buffer_flash[BUFFER_SIZE];
uint32_t flashSaveData(packet data);
void flashRead(uint32_t n_pages);
void flashSaveGlobalData(flash_global_vars_t data);
void flashReadGlobalData(void);
void flashResetGlobalData(void);
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
void calculateIMUvelocity(float accel_g_y, float gyro_dps_y);

#endif 