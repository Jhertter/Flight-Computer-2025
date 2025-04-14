#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
// #include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "libraries/icm_20948/pico-icm20948.h"
#include "libraries/MadgwickAHRS/MadgwickAHRS.h"
#include "libraries/sam_m8q/sam_m8q.h"

#define I2C_FREQUENCY 320000

#define PIN_LED_COM 11
#define PIN_LED_ERROR 10
#define PIN_LED_ALTITUDE 9

#define PIN_I2C_SDA 16
#define PIN_I2C_SCL 17

#define PIN_UART_TX 26
#define PIN_UART_RX 27

#define PIN_SD_CLK 20
#define PIN_SD_CMD 21
#define PIN_SD_DAT0 19
#define PIN_SD_DAT1 25
#define PIN_SD_DAT2 23
#define PIN_SD_DAT3 22

const static int gpio_size = 3;
const static int gpio_array[] = {
    PIN_LED_COM,
    PIN_LED_ALTITUDE,
    PIN_LED_ERROR};

i2c_inst_t i2c_setUp = {i2c0_hw, false};
icm20948_config_t IMU_config = {0x69, 0x0C, &i2c_setUp};
icm20984_data_t data;
madgwick_ahrs_t filter = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};

SFE_UBLOX_GPS GNSS;

bool dataflag = false;

bool read_icm20948(repeating_timer_t *rt);
void gpio_toggle(int pin);
void init_leds();

int main()
{
    stdio_init_all();
    sleep_ms(10000);
    init_leds();

    i2c_init(&i2c_setUp, I2C_FREQUENCY); //320kHz
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // Initialize ICM-20948

    // if (!icm20948_init(&IMU_config))
    // {
    //     // gpio_put(PIN_LED_ERROR, 1);
    //     printf("icm20948 IMU not detected at I2C address. Freezing");
    // }

    // icm20948_cal_gyro(&IMU_config, &data.gyro_bias[0]);
    // printf("calibrated gyro: %d %d %d\n", data.gyro_bias[0], data.gyro_bias[1], data.gyro_bias[2]);

    // icm20948_cal_accel(&IMU_config, &data.accel_bias[0]);
    // printf("calibrated accel: %d %d %d\n", data.accel_bias[0], data.accel_bias[1], data.accel_bias[2]);

    // int16_t accel_raw[3] = {0}, gyro_raw[3] = {0}, mag_raw[3] = {0}, temp_raw = 0;
    // float accel_g[3] = {0}, gyro_dps[3] = {0}, mag_ut[3] = {0}, temp_c = 0;

    // static repeating_timer_t timer;
    // add_repeating_timer_ms(-10, &read_icm20948, NULL, &timer);

    // Initialize SAM_M8Q
    if (GNSS.begin(i2c_setUp, 0x42) == false) // Connect to the Ublox module using Wire port
    {
        printf("Ublox GPS not detected at default I2C address. Please check wiring. Freezing.");
        gpio_put(PIN_LED_ERROR, 1);
        while (1);
    }

    while (true)
    {
        static long lastTime = 0;

        if (to_ms_since_boot(get_absolute_time()) - lastTime > 1000)
        {
            lastTime = to_ms_since_boot(get_absolute_time()); // Update the timer

            long latitude = GNSS.getLatitude();
            printf("Lat: %f,", latitude);

            long longitude = GNSS.getLongitude();
            printf(" Long: %f (degrees * 10^-7), ", longitude);

            long altitude = GNSS.getAltitude();
            printf(" Alt: %d (mm), ", altitude);

            long altitudeMSL = GNSS.getAltitudeMSL();
            printf(" AltMSL: %d (mm)\n", altitudeMSL);
        }
        // if (dataflag)
        // {
        //     dataflag = false;
        //     // 0: x, 1: y, 2: z
        //     accel_g[0] = (float)data.accel_raw[0] / 16384.0f;
        //     accel_g[1] = (float)data.accel_raw[1] / 16384.0f;
        //     accel_g[2] = (float)data.accel_raw[2] / 16384.0f;
        //     gyro_dps[0] = (float)data.gyro_raw[0] / 131.0f;
        //     gyro_dps[1] = (float)data.gyro_raw[1] / 131.0f;
        //     gyro_dps[2] = (float)data.gyro_raw[2] / 131.0f;
        //     mag_ut[0] = (float)data.mag_raw[1];
        //     mag_ut[1] = (float)-data.mag_raw[0];
        //     mag_ut[2] = (float)-data.mag_raw[2];

        //     MadgwickAHRSupdate(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8, mag_ut[0], mag_ut[1], mag_ut[2]);

        //     // printf("(data->q[0]) %0.1f, (data->q[1]) %0.1f, (data->q[2]) %0.1f, (data->q[3]) %0.1f\n", (data->q[0]), (data->q[1]), (data->q[2]), (data->q[3]));
        //     // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler[0] * 57.29578f, euler[1] * 57.29578f, euler[2] * 57.29578f + 180.0f);

        //     // accel(g)   = raw_value / (65535 / full_scale)
        //     // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value / 8192
        //     // gyro(dps)  = raw_value / (65535 / full_scale)
        //     // ex) if full_scale == +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131
        //     // mag(uT)    = raw_value / (32752 / 4912) = (approx) raw_value / 20 * 3
        //     printf("accel. x: %+2.5f, y: %+2.5f, z:%+2.5f\n", accel_g[0], accel_g[1], accel_g[2]);
        //     printf("gyro.  x: %+2.5f, y: %+2.5f, z:%+2.5f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        //     printf("mag.   x: %+2.5f, y: %+2.5f, z:%+2.5f\n", mag_ut[0], mag_ut[1], mag_ut[2]);
        //     // printf("temp: %+2.5f\n", data.temp_c);
        // }
    }
}

void gpio_toggle(int pin)
{
    if (gpio_get(pin))
        gpio_put(pin, 0);
    else
        gpio_put(pin, 1);
}

bool read_icm20948(repeating_timer_t *rt)
{
    icm20948_read_cal_accel(&IMU_config, &data.accel_raw[0], &data.accel_bias[0]);
    icm20948_read_cal_gyro(&IMU_config, &data.gyro_raw[0], &data.gyro_bias[0]);
    icm20948_read_cal_mag(&IMU_config, &data.mag_raw[0], &data.mag_bias[0]);
    icm20948_read_temp_c(&IMU_config, &data.temp_c);
    dataflag = true;
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
        gpio_toggle(PIN_LED_COM);
        sleep_ms(100);
        gpio_toggle(PIN_LED_ALTITUDE);
        sleep_ms(100);
        gpio_toggle(PIN_LED_ERROR);
        sleep_ms(100);
    }
}