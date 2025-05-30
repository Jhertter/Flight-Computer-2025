#include "SRAD-Computer-code.h"

XBee xbee(uart_cfg, to_ms_since_boot(get_absolute_time()));

flash_global_vars_t global_vars = {0};

packet parameters = {PRE_LAUNCH, 0};

int main()
{
    stdio_init_all();
    init_leds();

    // Init airbrake&payload pin
    gpio_init(PIN_AIRBRAKE);
    gpio_set_dir(PIN_AIRBRAKE, GPIO_OUT);
    gpio_put(PIN_AIRBRAKE, 0);

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

    // Initialize ADC for battery voltage measurement
    // adc_init();
    // adc_gpio_init(PIN_ADC_BATTERY); // ADC0
    // adc_select_input(PIN_ADC_BATTERY); // Select ADC0 (GPIO26) for battery voltage measurement

    // Set global variables as they were on last boot (just in case SRAD resets middle flight)
    // flashReadGlobalData();

    // Initialize ICM-20948
    if (icm20948_init(&IMU_config) != 0)
    {
        printf("IMU not detected at I2C address. Freezing");
        error_led();
        while (true)
            ;
    }
    gpio_toggle(PIN_LED_ALTITUDE);

    data.accel_bias[0] = accel_bias[0];
    data.accel_bias[1] = accel_bias[1];
    data.accel_bias[2] = accel_bias[2];

    // Initialize BME280
    if (BME.begin(0x76, 0x60) == false)
    {
        printf("BME not detected at default I2C address. Freezing.");
        error_led();
        while (1)
            ;
    }

    gpio_toggle(PIN_LED_ON);

    // Initialize SAM_M8Q
    if (GNSS.begin(i2c_setUp, 0x42) == false) // Connect to the Ublox module using Wire port
    {
        printf("GNSS not detected at default I2C address. Freezing.");
        error_led();
        while (1)
            ;
    }
    else
    {
        sleep_ms(200);
        GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
        sleep_ms(200);
        GNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
        sleep_ms(200);
        GNSS.setNavigationFrequency(20);
        sleep_ms(200);
        GNSS.setAutoPVTrate(20);
        sleep_ms(200);
        GNSS.setHNRNavigationRate(15);
        sleep_ms(200);

        gpio_put(PIN_LED_ALTITUDE, 1);

        // if(parameters.status == PRE_LAUNCH)
        // {
        //     do {
        //         parameters.satellite_count = GNSS.getSIV();
        //         printf("Waiting for GNSS satellites: %d\n", parameters.satellite_count);
        //     } while (parameters.satellite_count < 1);
        // }
    }
    init_leds();

    gpio_put(PIN_LED_ON, 1);

    while (true)
    {
        // static uint32_t last_time = 0;

        // if (to_ms_since_boot(get_absolute_time()) - last_time > 5) // 200Hz
        // {
        //     last_time = to_ms_since_boot(get_absolute_time());
        //     readData();
        //     telemetry();
        // }

        switch (parameters.status)
        {
        case PRE_LAUNCH:
            waitForStart();
            break;
        case LAUNCH:
            waitForLaunch();
            break;
        case ASCENT:
            ascentRoutine();
            break;
        case APOGEE:
            apogeeRoutine();
            break;
        case DESCENT:
            descentRoutine();
            break;
        // case LANDING:
        //     break;
        case RECOVERY:
            recoverySignal();
            break;
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

void error_led()
{
    gpio_put(PIN_LED_ERROR, 1);
    gpio_put(PIN_LED_ALTITUDE, 0);
    gpio_put(PIN_LED_ON, 0);
}

void updateXbeeParameters(uint32_t last_time)
{
    static bool si = true;
    // Mission data
    xbee.setMissionTime(last_time);
    xbee.setPacketCount();
    xbee.setStatus(parameters.status);
    // parameters.battery_level = (uint8_t)(adc_read() * 8.4f / 4096.0f * 100); // Convert ADC value to percentage
    parameters.battery_level = (uint32_t)(735);
    xbee.setBatteryVoltage(parameters.battery_level);
    
    // IMU data
    xbee.setIMUVerticalVel(parameters.imu_vel_y);
    xbee.setIMURoll(parameters.imu_roll);
    xbee.setIMUPitch(parameters.imu_pitch);
    
    // GNSS data
    xbee.setGNSSTime(parameters.gnss_time);
    xbee.setGNSSLatitude(parameters.gnss_latitude);
    xbee.setGNSSLongitude(parameters.gnss_longitude);
    xbee.setGNSSAltitude(parameters.gnss_altitude, parameters.gnss_altitude_MSL);
    
    // ESU data
    xbee.setBMEPressure(parameters.bme_pressure);
    xbee.setBMEAltitude(parameters.bme_altitude);
    xbee.setBMETemperature(parameters.bme_temperature);
}

void readData()
{
    static uint8_t times = 0;
    readIMU();

    if ((times % 5) == 0)
        readBME();

    if ((times % 20) == 0)
        readGNSS();

    times++;
    if (times > 240)
        times = 0;
}

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#define DEG_TO_RAD (1 / RAD_TO_DEG)
#define GRAVITY (9.80665f)
#define IMU_SAMPLING_RATE (200.0f)                     // Hz
#define IMU_SAMPLING_PERIOD (1.0f / IMU_SAMPLING_RATE) // seconds
#define STATIC_ACCELEROMETER_THRESHOLD (0.12f)         // m/s^2
#define STATIC_GYROSCOPE_THRESHOLD (3.0f)              // m/s

void readIMU()
{
    static float accel_g[3] = {0};
    static float gyro_dps[3] = {0};

    icm20948_read_cal_accel(&IMU_config, &data.accel_raw[0], &data.accel_bias[0]);
    icm20948_read_cal_gyro(&IMU_config, &data.gyro_raw[0], &data.gyro_bias[0]);

    accel_g[0] = (float)data.accel_raw[0] / 4096.0f;
    accel_g[1] = (float)data.accel_raw[1] / 4096.0f;
    accel_g[2] = (float)data.accel_raw[2] / 4096.0f;

    gyro_dps[0] = (float)data.gyro_raw[0] / 131.0f;
    gyro_dps[1] = (float)data.gyro_raw[1] / 131.0f;

    parameters.imu_roll = (int16_t)(atan2(-accel_g[0], accel_g[2]) * RAD_TO_DEG * 10);
    parameters.imu_pitch = (int16_t)(atan(accel_g[1] / sqrt(accel_g[0] * accel_g[0] + accel_g[2] * accel_g[2])) * RAD_TO_DEG * 10);

    parameters.imu_accel_y = (accel_g[1] * (-GRAVITY));                                                                   // m/s^2
    parameters.imu_accel_y = (ABS(parameters.imu_accel_y) < STATIC_ACCELEROMETER_THRESHOLD) ? 0 : parameters.imu_accel_y; // m/s^2

    calculateVelocity(accel_g[1], gyro_dps[1]);
}

/**
 * @brief Integrates Acceleration to obtain velicty with ZUPT correction.
 *
 * @param accel_g_y
 * @param gyro_dps_y
 */
void calculateVelocity(float accel_g_y, float gyro_dps_y)
{
    static float prev_accel_y = 0; // in graviy units
    static float delta_vel_y = 0;  // in m/s

    static float vel_y = 0;

    // ZUPT -> Zero Velocitu Update
    if ((ABS(accel_g_y - 1) < STATIC_ACCELEROMETER_THRESHOLD) && (ABS(gyro_dps_y) < STATIC_GYROSCOPE_THRESHOLD))
        delta_vel_y = 0;
    else
        delta_vel_y = 0.5 * (accel_g_y + prev_accel_y) * GRAVITY * IMU_SAMPLING_PERIOD;

    vel_y += delta_vel_y; // m/s
    prev_accel_y = accel_g_y;

    parameters.imu_vel_y = (int32_t)(vel_y * 100);
}

void readGNSS()
{
    // static bool first_time = true;

    // if (first_time)
    // {
    parameters.satellite_count = GNSS.getSIV();

    //     if (parameters.satellite_count > 20)
    //         parameters.satellite_count = 0;

    //     if (parameters.satellite_count > 0)
    //         first_time = false;
    // }

    if (parameters.satellite_count > 0)
    {
        parameters.gnss_time = (uint32_t)(GNSS.getHour() * 10000 + GNSS.getMinute() * 100 + GNSS.getSecond());
        parameters.gnss_latitude = GNSS.getLatitude();   // latitude +-90ª
        parameters.gnss_longitude = GNSS.getLongitude(); // longitude +-180ª
        parameters.gnss_altitude = (GNSS.getAltitude() - altitude_bias) / 1000;
        parameters.gnss_altitude = GNSS.getAltitudeMSL();
    }
}

// Returns the current pressure. Handy to have for calibration processes
void calibrateBME(void)
{
    float pressure = BME.readPressure();
    pressure /= 100;
    ground_hP = pressure;
}

void readBME()
{
    parameters.bme_temperature = (int32_t)(BME.readTemperature() * 100);
    parameters.bme_pressure = (uint32_t)BME.readPressure();
    parameters.bme_altitude = (int32_t)BME.readAltitude(1013.25f); 
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

Si se quedó sin memoria, va a devolver siempre la misma cantidad de veces que se
guardó en el buffer la estructura, pero no se va a poder guardar más información.
En este caso, todo lo que esté en el buffer, se va a perder si se apaga el sistema.
*/
uint32_t flashSaveData(packet data)
{
    static uint8_t buff_count = 0;
    static uint32_t saves = 0;
    static bool out_of_memory = false;

    if (buff_count < BUFFER_SIZE)
    {
        buffer_flash[buff_count] = data;
        buff_count++;
    }
    else if (!out_of_memory) // Si el buffer se llenó y no se quedó sin memoria
    {
        // La idea es borrar los sectores a medida que necesitemos. Entran 16 páginas
        //(escrituras), por lo que cada 16 iteraciones borramos el siguiente sector
        if (saves % 16 == 0)
        {
            uint32_t erase_addr = (FLASH_SIZE) - (uint32_t)(FLASH_SECTOR_SIZE * ((saves / 16) + 2)); //+2 para reservar el último para las variables globales
            if (erase_addr <= FLASH_CODE_END)                                                        // No borrar el sector de código
            {
                out_of_memory = true;
                return saves;
            }

            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(erase_addr, FLASH_SECTOR_SIZE);
            restore_interrupts(ints);
        }
        // Después de borrar (si fue necesario), guardamos el buffer en memoria
        uint32_t prog_addr = (FLASH_SIZE)-(FLASH_SECTOR_SIZE*2) - (FLASH_PAGE_SIZE * saves); // notar que el ultimo sector es para las variables globales

        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(prog_addr, (uint8_t *)(buffer_flash), FLASH_PAGE_SIZE);
        restore_interrupts(ints);

        saves++;
        buff_count = 0;
    }
    else if (global_vars.last_status == RECOVERY)
    {
        uint32_t prog_addr = (FLASH_SIZE)-(FLASH_SECTOR_SIZE) - (FLASH_PAGE_SIZE * (saves+1)); // notar que el ultimo sector es para las variables globales

        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(prog_addr, (uint8_t *)(buffer_flash), FLASH_PAGE_SIZE);
        restore_interrupts(ints);

        saves++;
    }
    return saves;
}

void flashRead(uint32_t n_pages)
{
    static packet *data_ptr = (packet *)(XIP_BASE + (FLASH_SIZE) - FLASH_SECTOR_SIZE -FLASH_PAGE_SIZE);

    for (uint32_t j = 0; j < n_pages; j++)
    {
        for (uint8_t i = 0; i < BUFFER_SIZE; i++) // este for mandaría todas las estructuras, una por una, por el puerto serie
        {
            for (uint8_t k = 0; k < sizeof(packet); k++) // este for mandaría todos los bytes de la estructura, uno por uno, por el puerto serie
            {
                printf("%02X", data_ptr[j]);
            }
            printf("\n");
            data_ptr++;
        }
        data_ptr = (packet *)(XIP_BASE + (FLASH_SIZE) - FLASH_SECTOR_SIZE - (FLASH_PAGE_SIZE * (j + 2)));
    }
}

void flashSaveGlobalData(flash_global_vars_t data)
{
    static const uint32_t erase_addr = (FLASH_SIZE) - (FLASH_SECTOR_SIZE);
    static const uint32_t prog_addr = (FLASH_SIZE) - (FLASH_PAGE_SIZE * 2); // No guardamos en la última página para dejar lugar a futuras implementaciones y tests
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(erase_addr, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    ints = save_and_disable_interrupts();
    flash_range_program(prog_addr, (uint8_t *)&global_vars, sizeof(flash_global_vars_t));
    restore_interrupts(ints);
}

void flashReadGlobalData(void)
{
    static flash_global_vars_t *flash_variables = (flash_global_vars_t *)(XIP_BASE + (FLASH_SIZE) - (FLASH_PAGE_SIZE * 2));
    global_vars = *flash_variables;
}

void flashResetGlobalData(void)
{
    static const uint32_t erase_addr = (FLASH_SIZE) - (FLASH_SECTOR_SIZE);
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(erase_addr, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

bool checkAccel(void)
{
    // if (parameters.imu_accel_y > 0)
    //     return true;
    // else
    //     return false;

    return false;
}

/**
 * @brief   Waits for START signal from ground station. Sends ACK.
 *          If no signal is received, double checks vertical acceleration.
 *
 */
void waitForStart(void)
{
    // leemos uart hasta que llegue el char de inicio (START)
    //  Manda ACK
    // si nunca llega START y empieza a acelerar el weon, pasamos a estado LAUNCH
    static uint32_t last_time = 0;

    if (xbee.receiveStartSignal() || parameters.imu_accel_y > 0)
    {
        calibrateRocket();
        parameters.status = LAUNCH;
        global_vars.last_status = parameters.status;
        flashSaveGlobalData(global_vars); // Guardado de parámetros globales en flash
        return;
    }

    if (to_ms_since_boot(get_absolute_time()) - last_time > 100) // para leer aceleración vertical, por si no llega el START
    {
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer

        readData();
    }
}

/**
 * @brief   Calibrate rocket parameters (GNSS altitude,
 *          pressure sensor offset & imu bias) and
 *          starts payload.
 *
 */
void calibrateRocket(void)
{
    // Y manda el telegrama a la payload con el pin que va a la ESP32
    // ASEGURARSE QUE VAN A PASAR X SEGUNDOS PARA QUE SE PRENDA LA PAYLOAD
    // dejamos de leer UART, seteamos el puerto para sólo enviarle chars

    icm20948_cal_gyro(&IMU_config, &data.gyro_bias[0]);
    calibrateBME();                     // Actualizamos la presión actual en el suelo, así calculamos la altura a partir de esto
    altitude_bias = GNSS.getAltitude(); // :+1:
    global_vars.initial_lat = GNSS.getLatitude();
    global_vars.initial_long = GNSS.getLongitude();
    airbrake_payload(true);
}

/**
 * @brief NOP unless accel > 0
 *
 */
void waitForLaunch(void)
{
    static uint32_t last_time = 0;
    // se queda en este estado hasta que la accel_y > 0
    // Podria trasmitir telemetria tambien, TBD
    // si accel > 0:
    // 	event_action = MOVE;
    // 	parameters.status = ASCENT;
    //  save_falsh = true;

    if (to_ms_since_boot(get_absolute_time()) - last_time > 5)
    {
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer

        readData();

        if (parameters.imu_accel_y > 0)
        {
            parameters.status = ASCENT;
            global_vars.last_status = ASCENT;
            flashSaveGlobalData(global_vars); // Guardado de parámetros globales en flash
        }
    }

    return;
}

void ascentRoutine(void)
{
    static uint32_t last_time = 0;

    if (parameters.gnss_altitude > 2000)
    {
        parameters.status = APOGEE;
        global_vars.last_status = APOGEE;
        flashSaveGlobalData(global_vars); // Guardado de parámetros globales en flash

        return;
    }

    if (to_ms_since_boot(get_absolute_time()) - last_time > 5)
    {
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer

        readData();

        global_vars.cant_pages = flashSaveData(parameters); // guardado de datos actuales en flash
    }

    telemetry();

    return;
}

/**
 * @brief Transmits telemetry through xbee
 *
 */
void telemetry(void)
{
    static uint32_t last_time_xbee = 0;

    if (to_ms_since_boot(get_absolute_time()) - last_time_xbee > 100)
    {
        last_time_xbee = to_ms_since_boot(get_absolute_time());
        updateXbeeParameters(last_time_xbee);
        xbee.sendPkt();

#if PRINT_DEBUG_PKT

        printf("%d - ", parameters.satellite_count);

        xbee.sendParameter(MISSION_TIME);
        xbee.sendParameter(PACKET_COUNT);
        xbee.sendParameter(MISSION_STATUS);
        xbee.sendParameter(BATTERY_LEVEL);

        printf("- ");

        xbee.sendParameter(IMU_ROLL);
        xbee.sendParameter(IMU_PITCH);
        xbee.sendParameter(IMU_Y_VEL);

        printf("- ");

        if (parameters.satellite_count > 0)
        {
            xbee.sendParameter(GNSS_TIME);
            xbee.sendParameter(GNSS_LATITUDE);
            xbee.sendParameter(GNSS_LONGITUDE);
            xbee.sendParameter(GNSS_ALTITUDE);
        }

        printf("- ");

        xbee.sendParameter(BME_PRESSURE);
        xbee.sendParameter(BME_ALTITUDE);
        xbee.sendParameter(BME_TEMPERATURE);

        printf("\n");
#endif
    }
}

/**
 * @brief 	When 2000m are reached, starts airbrake control.
 * 			Stores max height.
 *
 */
void apogeeRoutine(void)
{
    static uint32_t last_time = 0;
    // empiezan los cálculos del airbrake
    // busca altura máxima
    // cuando accel = 0, almacena la altura máxima --> LAMENTABLEMENTE FALSO
    // event_action = MOVE;
    // parameters.status = DESCENT;

    if (parameters.imu_vel_y < 0) // hasta que no funcione del todo bien la velocidad, calcular altura
    {
        parameters.status = DESCENT;
        global_vars.last_status = DESCENT;
        global_vars.max_altitude = parameters.gnss_altitude;
        flashSaveGlobalData(global_vars); // Guardado de parámetros globales en flash
        return;
    }

    if (to_ms_since_boot(get_absolute_time()) - last_time > 5)
    {
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer

        readData();
        global_vars.cant_pages = flashSaveData(parameters); // Guardado de datos actuales en flash

        airbrake_payload();
    }
    telemetry();
}

void descentRoutine(void)
{
    static uint32_t last_time = 0;

    if (((parameters.gnss_altitude < 100) && (parameters.imu_vel_y > -2 && parameters.imu_vel_y < 2)) || parameters.gnss_altitude < 5) // Si está descendiendo lo suficientemente lento como para decir que aterrizó
    {
        parameters.status = RECOVERY;
        global_vars.last_status = RECOVERY;
        flashSaveGlobalData(global_vars); // Guardado de parámetros globales en flash
        return;
    }

    if (to_ms_since_boot(get_absolute_time()) - last_time > 5)
    {
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer

        readData();                                    // :+1:
        global_vars.cant_pages = flashSaveData(parameters); // Guardado de datos actuales en flash
    }
    telemetry();
}

/**
 * @brief 	Sends absolute coordinates and time to GS.
 * 			Starts max altitude protocol.
 *
 */
void recoverySignal(void)
{
    static uint32_t last_time = 0;

    if (to_ms_since_boot(get_absolute_time()) - last_time > 5000)
    {
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer
        telemetry();
    }
}

/**
 * @brief 	Shuts down sensors -> enters low energy mode.
 *
 */
void standByMode(void)
{
    // TODO: Si la bateria no llegase a alcanzar, apagar todos los sensores
}

/**
 * @brief   PIN_AIRBRAKE = 1 : airbrake closed
 *          PIN_AIRBRAKE = 0 : airbrake open
 *
 * @param payload
 */
void airbrake_payload(bool payload)
{
    static bool pin_state = false;
    static int16_t poly_curve_1 = 0;
    static int16_t poly_curve_3 = 0;

    pin_state = gpio_get(PIN_AIRBRAKE);
    poly_curve_1 = (int16_t)(-1.209e-7 * (float)pow(parameters.imu_vel_y, 4) + 1.479e-4 * pow(parameters.imu_vel_y, 3) - 5.82e-2 * pow(parameters.imu_vel_y, 2) + 3053);
    poly_curve_3 = (int16_t)(1.088e-7 * (float)pow(parameters.imu_vel_y, 4) + 3.179e-5 * pow(parameters.imu_vel_y, 3) - 5.009e-2 * pow(parameters.imu_vel_y, 2) + 3047);

    if (payload)
        gpio_put(PIN_AIRBRAKE, 1);
    else
    {
        if ((parameters.gnss_altitude > poly_curve_3) && pin_state)
            gpio_put(PIN_AIRBRAKE, 0); // Open airprake
        else if ((parameters.gnss_altitude < poly_curve_1) && !pin_state)
            gpio_put(PIN_AIRBRAKE, 1); // Close airbrake
    }
}


