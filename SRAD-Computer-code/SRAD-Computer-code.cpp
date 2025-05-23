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

    sleep_ms(500);

    // Initialize ICM-20948
    if (icm20948_init(&IMU_config) != 0)
    {
        printf("IMU not detected at I2C address. Freezing");
        gpio_put(PIN_LED_ERROR, 1);
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
        gpio_put(PIN_LED_ERROR, 1);
        while (1)
            ;
    }
    

    gpio_toggle(PIN_LED_ON);

    // Initialize SAM_M8Q
    if (GNSS.begin(i2c_setUp, 0x42) == false) // Connect to the Ublox module using Wire port
    {
        printf("GNSS not detected at default I2C address. Freezing.");
        gpio_put(PIN_LED_ERROR, 1);
        while (1)
            ;
    }
    else
    {
        // sleep_ms(500);
        // if(!GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS) || !GNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS))
        //     printf("GPS not enabled\n");
        // sleep_ms(500);
        // if (GNSS.setNavigationFrequency(20))
        //     printf("GNSS nav Frequency: %d\n", GNSS.getNavigationFrequency());
        // else
        //     printf("Skill issue GNSS nav Frequency\n");
        // sleep_ms(500);
        // if (!GNSS.setAutoPVTrate(20))
        //     printf("Skill issue AUTO PVT rate\n");
        // sleep_ms(500);
        // if (!GNSS.setHNRNavigationRate(15))
        //     printf("Skill issue HNR Navigation rate\n");
        // sleep_ms(500);
        sleep_ms(500);
        GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
        sleep_ms(500);
        GNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
        sleep_ms(500);
        GNSS.setNavigationFrequency(20);
        sleep_ms(500);
        GNSS.setAutoPVTrate(20);
        sleep_ms(500);
        GNSS.setHNRNavigationRate(15);
    }
    init_leds();
    gpio_put(PIN_LED_ON, 1);

    while (true)
    {
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

void update_xbee_parameters(uint32_t last_time)
{
    // Mission data
    xbee.setMissionTime(last_time);
    // xbee.setBatteryVoltage(parameters.battery_level);

    // GNSS data
    if (parameters.satellite_count > 0)
    {
        xbee.setGNSSTime(parameters.gnss_time);
        xbee.setGNSSLatitude(parameters.gnss_latitude);
        xbee.setGNSSLongitude(parameters.gnss_longitude);
        xbee.setGNSSAltitude(parameters.gnss_altitude, parameters.gnss_altitude_MSL);
    }

    // ESU data
    xbee.setBMEPressure(parameters.bme_pressure);
    xbee.setBMEAltitude(parameters.bme_altitude);
    xbee.setBMETemperature(parameters.bme_temperature);

    // IMU data
    xbee.setIMUPitch(parameters.imu_pitch);
    xbee.setIMURoll(parameters.imu_roll);
    xbee.setIMUVerticalVel(parameters.imu_vel_y);
}

void read_data()
{
    static uint8_t times = 0;
    read_imu();

    if ((times % 5) == 0)
        read_bme();

    if ((times % 20) == 0)
        read_gnss();

    times++;

    if (times > 240)
        times = 0;
}

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#define DEG_TO_RAD (1 / RAD_TO_DEG)
#define GRAVITY (9.80665f)
#define IMU_SAMPLING_RATE (200.0f)             // Hz
#define STATIC_ACCELEROMETER_THRESHOLD (0.12f) // m/s^2
#define STATIC_GYROSCOPE_THRESHOLD (0.15f)     // m/s

void read_imu()
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

    // ZUPT -> Zero Velocitu Update
    // if ((ABS(accel_g[1]) < STATIC_ACCELEROMETER_THRESHOLD) && (ABS(gyro_dps[1]) < STATIC_GYROSCOPE_THRESHOLD))
    //     delta_vel_y = 0;
    // else
    //     delta_vel_y += 0.5 * (accel_y + prev_accel_y) * (1 / IMU_SAMPLING_RATE);

    // vel_y += delta_vel_y; // m/s

    // prev_accel_y = accel_y;

    // printf("Accel :%2.3f %2.3f %2.3f ,", accel_g[0], accel_g[1], accel_g[2]);
    // printf("pitch:%f\n", (float)parameters.imu_pitch / 10);
    // printf("accel_y:%f,vel:%f,pos:%f,pitch:%f\n", accel_y, vel_y, pos_y, (float)parameters.imu_pitch / 10);
}

void read_gnss()
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
void calibrate_bme(void)
{
    float pressure = BME.readPressure();
    pressure /= 100;
    ground_hP = pressure;
}

void read_bme()
{
    parameters.bme_temperature = (uint32_t)(BME.readTemperature() * 100);
    parameters.bme_pressure = (uint32_t)BME.readPressure();
    parameters.bme_altitude = (uint32_t)BME.readAltitude(ground_hP);
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
uint32_t saveData(packet data)
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
            if (erase_addr <= FLASH_CODE_END) // No borrar el sector de código
            {
                out_of_memory = true;
                return saves;
            }

            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(erase_addr, FLASH_SECTOR_SIZE);
            restore_interrupts(ints);
        }
        // Después de borrar (si fue necesario), guardamos el buffer en memoria
        uint32_t prog_addr = (FLASH_SIZE) - FLASH_SECTOR_SIZE - (FLASH_PAGE_SIZE * saves); //notar que el ultimo sector es para las variables globales

        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(prog_addr, (uint8_t *)(buffer_flash), FLASH_PAGE_SIZE);
        restore_interrupts(ints);

        saves++;
        buff_count = 0;
    }
    return saves;
}

void readData(uint32_t n_pages)
{
    static packet *data_ptr = (packet *)(XIP_BASE + (FLASH_SIZE)-FLASH_PAGE_SIZE);

    packet data[BUFFER_SIZE] = {0};
    for (uint32_t j = 0; j < n_pages; j++)
    {
        for (uint8_t i = 0; i < BUFFER_SIZE; i++) // este for mandaría todas las estructuras, una por una, por el puerto serie
        {
            fwrite(data_ptr, sizeof(packet), 1, stdout); // TODO: a checkear que ande. Spoiler: no anda
            printf("Packet %d: \n", i);
        }
    }
}

void saveGlobalData(flash_global_vars_t data)
{
    static const uint32_t erase_addr = (FLASH_SIZE) - (FLASH_SECTOR_SIZE);
    static const uint32_t prog_addr = (FLASH_SIZE) - (FLASH_PAGE_SIZE * 2); //notar que el ultimo sector es para las variables globales 
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(erase_addr, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    ints = save_and_disable_interrupts();
    flash_range_program(prog_addr, (uint8_t *)&global_vars, sizeof(flash_global_vars_t));
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
 * @brief Waits for START signal from ground station. Sends ACK.
 *
 */
void waitForStart(void)
{
	//leemos uart hasta que llegue el char de inicio (START)
	// Manda ACK
	//si nunca llega START y empieza a acelerar el weon, pasamos a estado LAUNCH
    static uint32_t last_time = 0;

	if (xbee.receiveStartSignal() || parameters.imu_accel_y > 0)
    {
        parameters.status = LAUNCH;
        global_vars.last_status = parameters.status;
        saveGlobalData(global_vars);                //Guardado de parámetros globales en flash
        calibrateRocket();
        return;
    }
        
	if (to_ms_since_boot(get_absolute_time()) - last_time > 100)
	{
		last_time = to_ms_since_boot(get_absolute_time()); // Update the timer
		
		read_data();
		update_xbee_parameters(last_time);
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
	//dejamos de leer UART, seteamos el puerto para sólo enviarle chars

    icm20948_cal_gyro(&IMU_config, &data.gyro_bias[0]);
    calibrate_bme(); // Actualizamos la presión actual en el suelo, así calculamos la altura a partir de esto
    altitude_bias = GNSS.getAltitude(); // :+1:

    // startPayload();
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
		
		read_data();
		update_xbee_parameters(last_time);
        if(parameters.imu_accel_y > 0)
        {
            parameters.status = ASCENT;
            global_vars.last_status = ASCENT;
            saveGlobalData(global_vars);                //Guardado de parámetros globales en flash
            //save_falsh = true;            //TODO: ver si implementamos este flag
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
        saveGlobalData(global_vars);                //Guardado de parámetros globales en flash
    }
    
	if (to_ms_since_boot(get_absolute_time()) - last_time > 5)
	{
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer
		
		read_data();
		update_xbee_parameters(last_time);
        saveData(parameters); //guardado de datos actuales en flash
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
		xbee.sendPkt();
		xbee.setPacketCount();

#if PRINT_DEBUG_PKT

		printf("%d - ", parameters.satellite_count);

		xbee.sendParameter(MISSION_TIME);
		xbee.sendParameter(PACKET_COUNT);
		// xbee.sendPkt(STATUS);
		// xbee.sendPkt(BATTERY_VOLTAGE);

		printf("- ");

		xbee.sendParameter(IMU_ROLL);
		xbee.sendParameter(IMU_PITCH);
		// printf("%ld(10^-3 m/s) ", parameters.imu_xvel);
		// printf("%ld(10^-3 m/s) ", parameters.imu_yvel);
		// xbee.sendPkt(IMU_ACCELERATION);
		// xbee.sendPkt(IMU_TILT);

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
	// cuando accel = 0, almacena la altura máxima
	// event_action = MOVE;
	// parameters.status = DESCENT;

    if (parameters.imu_vel_y < 0)   // hasta que no funcione del todo bien la velocidad, calcular altura
    {
        parameters.status = DESCENT;
        global_vars.last_status = DESCENT;
        saveGlobalData(global_vars);                //Guardado de parámetros globales en flash
    }

	if (to_ms_since_boot(get_absolute_time()) - last_time > 5)
	{
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer
		
		read_data();
		update_xbee_parameters(last_time);

        saveData(parameters);                               //Guardado de datos actuales en flash
  
        // airbrake();
	}
    telemetry();
}

void descentRoutine(void)
{
    static uint32_t last_time = 0;
	// cheque si hubo landing
	// telemetry();
	// si hubo landing:
	// event_action = MOVE;
	// parameters.status = RECOVERY;
    // standByMode();

    if (parameters.imu_vel_y > -0.05 || parameters.imu_vel_y < 0.05) //Si está descendiendo lo suficientemente lento como para decir que aterrizó
    {
        parameters.status = RECOVERY;
        global_vars.last_status = RECOVERY;
        saveGlobalData(global_vars);                //Guardado de parámetros globales en flash
    }

	if (to_ms_since_boot(get_absolute_time()) - last_time > 5)
	{
        last_time = to_ms_since_boot(get_absolute_time()); // Update the timer
		
		read_data();
		update_xbee_parameters(last_time);        
        saveData(parameters);                               //Guardado de datos actuales en flash

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

	if (to_ms_since_boot(get_absolute_time()) - last_time > 3000)
        telemetry();
}

/**
 * @brief 	Shuts down sensors -> enters low energy mode.
 *
 */
void standByMode(void)
{

}


/**
 * @brief   PIN_AIRBRAKE = 1 : airbrake closed
 *          PIN_AIRBRAKE = 0 : airbrake open
 * 
 * @param payload 
 */
void airbrake_payload(bool payload = false)
{
    static bool pin_state = false;
    static int16_t poly_curve_1 = 0;
    static int16_t poly_curve_3 = 0;

    pin_state = gpio_get(PIN_AIRBRAKE);
    poly_curve_1 = (int16_t)(-1.209e-7 * (float)pow(parameters.imu_vel_y, 4) + 1.479e-4 * pow(parameters.imu_vel_y,3) - 5.82e-2
* pow(parameters.imu_vel_y,2) + 3053);
    poly_curve_3 = (int16_t)(1.088e-7 * (float)pow(parameters.imu_vel_y, 4) + 3.179e-5 * pow(parameters.imu_vel_y,3) - 5.009e-2
* pow(parameters.imu_vel_y,2) + 3047);


    if (payload)
        gpio_put(PIN_AIRBRAKE, 1);
    else
    {
        if ((parameters.gnss_altitude > poly_curve_3) && pin_state)
            gpio_put(PIN_AIRBRAKE, 0);  // Open airprake 
        else if ((parameters.gnss_altitude < poly_curve_1) && !pin_state)
            gpio_put(PIN_AIRBRAKE, 1);  // Close airbrake
    }

}