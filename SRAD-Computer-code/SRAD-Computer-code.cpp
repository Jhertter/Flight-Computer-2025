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
// TODO: Uncomment when XBee branch is merged
// #include "libraries/XBee/XBee.h"
#include "math.h" // remove en above is uncommented

#include "libraries/FSM/fsm.h"
#include "libraries/FSM/fsmtable.h"

#define I2C_FREQUENCY 100000

#define PIN_LED_ON 11
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

#define TRANSMISSION_PERIOD 40 // ms

// #define PIN_SD_CLK 20
// #define PIN_SD_CMD 21
// #define PIN_SD_DAT0 19
// #define PIN_SD_DAT1 25
// #define PIN_SD_DAT2 23
// #define PIN_SD_DAT3 22

const static int gpio_size = 3;
const static int gpio_array[] = {
    PIN_LED_ON,
    PIN_LED_ALTITUDE,
    PIN_LED_ERROR};
    
i2c_inst_t i2c_setUp = {i2c0_hw, false};
icm20948_config_t IMU_config = {0x69, 0x0C, &i2c_setUp};
icm20984_data_t data;
madgwick_ahrs_t filter = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};
BMP280 BME(&i2c_setUp);
SFE_UBLOX_GNSS GNSS;

STATE *p_fsm = NULL;

// TODO: Uncomment when XBee branch is merged
// xbee_uart_cfg_t uart_cfg = {UART_ID, UART_BAUD, UART_STOP_BITS, UART_DATA_BITS, UART_PARITY};
// XBee xbee(uart_cfg, to_ms_since_boot(get_absolute_time()));

typedef struct
{
    uint8_t mission_state = 0;  // TODO: Use enum instead of uint8_t

    uint32_t bme_pressure = 0;
    uint32_t bme_temperature = 0;
    uint32_t bme_altitude = 0;
    
    uint8_t satellite_count = 0;
    uint32_t gnss_time = 0;
    int32_t gnss_latitude = 0;  // latitude +-90ª
    int32_t gnss_longitude = 0; // longitude +-180ª
    int32_t gnss_altitude = 0;
    int32_t gnss_altitude_MSL = 0;
    
    int16_t imu_roll = 0;
    int16_t imu_pitch = 0;
    int32_t imu_xvel = 0;
    int32_t imu_yvel = 0;
} packet;

#define FLASH_OFFSET (0x01000000) // definimos un offset para no pisar nuestro código
#define BUFFER_SIZE ((uint8_t)(FLASH_PAGE_SIZE / sizeof(packet)))
#define FLASH_SIZE ((uint32_t)(16 * 1024 * 1024))

const uint8_t *flashBase = (const uint8_t *)(XIP_BASE + FLASH_OFFSET);
packet buffer_flash[BUFFER_SIZE] = {0};
uint32_t saveData(packet data);

packet parameters;

void read_data();
void update_xbee_parameters(uint32_t last_time);

void calibrate_bme(void);

void read_imu();
void read_gnss();
void read_bme();
static float ground_hP = 0;

void gpio_toggle(int pin);
void init_leds();

int main()
{
    stdio_init_all();
    init_leds();

    // Initialize I2C Bus
    gpio_init(PIN_I2C_SCL);
    gpio_init(PIN_I2C_SDA);
    i2c_init(&i2c_setUp, I2C_FREQUENCY); // 320kHz
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    // Initialize ICM-20948
    if (icm20948_init(&IMU_config) != 0)
    {
        printf("IMU not detected at I2C address. Freezing");
        gpio_put(PIN_LED_ERROR, 1);
        while (true)
            ;
    }
    icm20948_cal_gyro(&IMU_config, &data.gyro_bias[0]);
    icm20948_cal_accel(&IMU_config, &data.accel_bias[0]);
    
    gpio_toggle(PIN_LED_ALTITUDE);

    // Initialize BME280
    if (BME.begin(0x76, 0x60) == false)
    {
        printf("BME not detected at default I2C address. Freezing.");
        gpio_put(PIN_LED_ERROR, 1);
        while (1)
            ;
    }
    calibrate_bme(); //Actualizamos la presión actual en el suelo, así calculamos la altura a partir de esto

    gpio_toggle(PIN_LED_ON);
    
    // Initialize SAM_M8Q
    // if (GNSS.begin(i2c_setUp, 0x42) == false) // Connect to the Ublox module using Wire port
    // {
        //     printf("GNSS not detected at default I2C address. Freezing.");
        //     gpio_put(PIN_LED_ERROR, 1);
        //     while (1)
        //         ;
        // }
        // else
        // {
    //     // printf("GNSS initialized.\n");
    //     GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS, 11000);
    // }

    init_leds();
    gpio_put(PIN_LED_ON, 1);

    p_fsm = FSM_GetInitState();

    while (true)
    {

        static uint32_t last_time = 0;

        if (to_ms_since_boot(get_absolute_time()) - last_time > TRANSMISSION_PERIOD)
        {
            last_time = to_ms_since_boot(get_absolute_time()); // Update the timer
            
            read_data();
            update_xbee_parameters(last_time);

            p_fsm = fsm(p_fsm, parameters.gnss_altitude); // Call the FSM with the current event
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

// TODO: Uncomment when XBee branch is merged
void update_xbee_parameters(uint32_t last_time)
{
//     // Mission data
//     xbee.setMissionTime(last_time);
//     // xbee.setPacketCount();
//     // xbee.setBatteryVoltage(parameters.battery_voltage);

//     // xbee.setIMUAcceleration();
//     xbee.setIMURoll(parameters.imu_roll);
//     xbee.setIMUPitch(parameters.imu_pitch);

//     // // GNSS data
//     if (parameters.satellite_count > 0)
//     {
//         xbee.setGNSSTime(GNSS.getHour(), GNSS.getMinute(), GNSS.getSecond());
//         xbee.setGNSSLatitude(parameters.gnss_latitude);
//         xbee.setGNSSLongitude(parameters.gnss_longitude);
//         xbee.setGNSSAltitude(parameters.gnss_altitude, parameters.gnss_altitude_MSL);
//     }
    
//     // ESU data
//     xbee.setBMEPressure(parameters.bme_pressure);
//     xbee.setBMEAltitude(parameters.bme_altitude);
//     xbee.setBMETemperature(parameters.bme_temperature);
    
//     // IMU data
//     // xbee.setIMUAcceleration(data.accel_raw[0]);
//     // xbee.setIMUTilt(data.gyro_raw[0]);
}

// bool read_data(repeating_timer_t *rt)
void read_data()
{
    static uint8_t times = 0;

    read_imu();
    read_bme();

    // if ((times % 7) == 0)
    //     read_gnss();

    times++;
}

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#define DEG_TO_RAD (1/RAD_TO_DEG)
#define GRAVITY (9.80665f)
#define IMU_SAMPLING_RATE (25.0f) // Hz
#define STATIC_ACCELEROMETER_THRESHOLD (0.12f) // m/s^2
#define STATIC_GYROSCOPE_THRESHOLD (0.15f) // m/s

void read_imu()
{
    static float accel_g[3] = {0};
    static float gyro_dps[3] = {0};
    static float mag_ut[3] = {0};

    icm20948_read_cal_accel(&IMU_config, &data.accel_raw[0], &data.accel_bias[0]);
    icm20948_read_cal_gyro(&IMU_config, &data.gyro_raw[0], &data.gyro_bias[0]);

    accel_g[0] = (float)data.accel_raw[0] / 16384.0f;
    accel_g[1] = (float)data.accel_raw[1] / 16384.0f;
    accel_g[2] = (float)data.accel_raw[2] / 16384.0f;

    gyro_dps[0] = (float)data.gyro_raw[0] / 131.0f;
    gyro_dps[1] = (float)data.gyro_raw[1] / 131.0f;
    gyro_dps[2] = (float)data.gyro_raw[2] / 131.0f;

    MadgwickAHRSupdate(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * GRAVITY, accel_g[1] * GRAVITY, accel_g[2] * GRAVITY, mag_ut[0], mag_ut[1], mag_ut[2]);

    parameters.imu_xvel += (int16_t)(accel_g[0] / IMU_SAMPLING_RATE * 1000);
    parameters.imu_yvel += (int16_t)(accel_g[1] / IMU_SAMPLING_RATE * 1000);
    parameters.imu_roll = (int16_t)(atan2(-accel_g[0], accel_g[2]) * RAD_TO_DEG * 10);
    parameters.imu_pitch = (int16_t)(atan(accel_g[1] / sqrt(accel_g[0] * accel_g[0] + accel_g[2] * accel_g[2])) * RAD_TO_DEG * 10);

    // printf("Accel_raw: %+2.2f %+2.2f %+2.2f - ", accel_g[0], accel_g[1], accel_g[2]);
    // printf("Gyro_raw: %+2.2f %+2.2f %+2.2f - ", gyro_dps[0], gyro_dps[1], gyro_dps[2]);

    static float accel_y = 0;
    static float prev_accel_y = 0;

    static float vel_y = 0;
    static float prev_vel_y = 0;

    static float pos_y = 0;
    static float prev_pos_y = 0;

    accel_y = (accel_g[1] - cos((90 - parameters.imu_pitch / 10) * DEG_TO_RAD)) * GRAVITY; // m/s^2

    // ZUPT -> Zero Velocitu Update
    if ((ABS(accel_g[1]) > STATIC_ACCELEROMETER_THRESHOLD) && (ABS(gyro_dps[1]) > STATIC_GYROSCOPE_THRESHOLD))
        vel_y +=  0.5 * (accel_y + prev_accel_y) * (1/IMU_SAMPLING_RATE);
    else
        vel_y = 0;
    
    pos_y += vel_y * (1/IMU_SAMPLING_RATE);

    prev_vel_y = vel_y;
    prev_accel_y = accel_y;

    // printf("Accel y: %+2.3f(m/s^2) - ", accel_y);
    // printf("Vel y: %2.3f(m/s)\n", vel_y);
    printf("vel:%f, pos:%f\n", vel_y, pos_y);
}

void read_gnss()
{
    parameters.satellite_count = GNSS.getSIV();

    if (parameters.satellite_count > 0)
    {
        parameters.gnss_time = GNSS.getHour() * 10000 + GNSS.getMinute() * 100 + GNSS.getSecond();
        parameters.gnss_latitude = GNSS.getLatitude();   // latitude +-90ª
        parameters.gnss_longitude = GNSS.getLongitude(); // longitude +-180ª
        parameters.gnss_altitude = GNSS.getAltitude();
        parameters.gnss_altitude_MSL = GNSS.getAltitudeMSL();
    }
}

void read_bme()
{
    parameters.bme_temperature = (uint32_t)(BME.readTemperature() * 100);
    parameters.bme_pressure = (uint32_t)BME.readPressure();
    parameters.bme_altitude = (uint32_t)BME.readAltitude(ground_hP);

    // printf("Pressure: %f(hPa) - ", BME.readPressure()/100);
    // printf("Altitude: %f(m) - ", BME.readAltitude(ground_hP));
    // printf("Temperature: %f(ºC) - ", BME.readTemperature());
    // printf("Ground pressure: %f(hPa)\n", ground_hP);
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
        gpio_toggle(PIN_LED_ON);
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

    if (buff_count < BUFFER_SIZE)
    {
        buffer_flash[buff_count] = data;
        buff_count++;
    }
    else
    {
        // La idea es borrar los sectores a medida que necesitemos. Entran 16 páginas
        //(escrituras), por lo que cada 16 iteraciones borramos el siguiente sector
        if (saves % 16 == 0)
        {
            printf("Erasing sector %d\n", saves / 16);
            sleep_ms(100);
            uint32_t erase_addr = (FLASH_SIZE) - (uint32_t)(FLASH_SECTOR_SIZE * (saves / 16 + 1));

            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(erase_addr, FLASH_SECTOR_SIZE);
            restore_interrupts(ints);
        }
        // Después de borrar (si fue necesario), guardamos el buffer en memoria
        printf("Writing buffer %d\n", saves);
        sleep_ms(100);
        uint32_t prog_addr = (FLASH_SIZE) - (FLASH_PAGE_SIZE * (saves + 1));

        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(prog_addr, (uint8_t *)(buffer_flash), FLASH_PAGE_SIZE);
        restore_interrupts(ints);

        saves++;
        buff_count = 0;
    }
    return saves;
} // TODO: Fijarse que no sobreescriba la sección de código (contemplar ese caso)
  // TODO: hardcodear el archivo build/pico_flash_region.ld

// Returns the current pressure. Handy to have for calibration processes  
void calibrate_bme(void)
{
    float pressure = BME.readPressure();
    pressure /= 100;
    ground_hP = pressure;
}