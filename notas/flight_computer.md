# ITBA Rocketry - A&Rocketry
## Flight Computer

### <u>Edición 2025</u>

Si llega a haber freno de aire, va a tener una cpu distinta

La placa SRAD tranquilamente puede ser de 90x150mm (el 90 está muy justo, el otro se puede ajustar). Si la tenemos lista para finales de febrero podriamos mandar a fabricarla.

Tener BOM para 28/01. Armar un GANT con final el 10/04

### Minutas reuniones

#### 10/03

2 compu/altimetros ccomercciales (RRC3 ni idea) 

ver si podemos activar la SRAD FC a mas de 2km o ver si se puede mantener prendido 10hs. Cambiaron el sensor de presion y temperatura

MPU 9050

puertos
RP4020 x3
flash x2
M8Q x1
bateria holder
18450  x4

Consideraciones de la placa
- dejar marcas de tornillos y bulones
- equilibrar la placa
- 90x120mm, puede ser hasta 150mm
- no poner todo de un lado y dejar el resto vacio
  
Investigar un poco sobre la placa para BMS

#### 27/03

- ~~usar shielding de GND para nets sensibles (SAM + flash)~~
- ver alternativas de capacitores para no usar 7 para la rpi -> miguel o pañol
- poner leds (pueden ser smd, mejor de colores distintos)
- ~~si es doble capa se puede usar nets como caps para VCC~~
- ~~conexión a xbee~~
- ~~poner opción de conectar una bornera y que un PMOS haga de llave driveado por el micro (camara), usa latch~~?
- ~~el MPU viene con placa integrada, es mejor sacarlo o dejarlo y conectar eso?~~
- poner pulsador por las dudas (ej: al tocar se envia la mem por el usb)
- no usar plano de masa general del otro lado (acople)
- ~~asegurarse que los caps de desacople esten antes de las entradas~~
- no exponer el BME280 a mucha luz (es fotosensible)
- a~~l gps va a haber que aisalrlo en la placa, dejarlo en una esquina separado y tiene que estar en un plano de GND de 50x50 mm ~~
- ~~ver SAM-M8Q Hardware Integration Manual, UBX-16018358 pag 12~~
- ~~usar conexión troncal para VCC y GND, dejar al final el gps~~
- va a haber que hacerle un case con tapa a la placa para evitar corrientes de aire
- usar pulsera antiestática
- a nivel bahía analizar la posición para maximizar el diagrama de radiación
- el regulador lo pondría arriba del usb, sería reg-usbc-jumper-bornera3


### Componentes

#### Micro
-   RP2040
    -   Cores: 2
    -   Clk: 133MHz
    -   GPIO: 34
    -   DAC: nada
    -   ADC: 4 canales, 12-bits
    -   Protocolos: UART, SPI, I2C, I2S, PIO    
    -   Consumo: es menor que el de Xtensa LX6
    -   Precio: ~$1000
    -   Comentarios: Como no vamos a usar los módulos wifi y bluetooth (las telemetrías se van a hacer por algún módulo LoRa), va a ser mejor usar el rp2040 para poder controlar mejor los modulos. Xtensa LX6 tiene limitaciones para operaciones en tiempo real (medio que ya lo vimos para el final de labo3). El RP2040 dicen que está diseñado especificamente para operaciones de baja latencia.
    -   PIO block: modulo para operaciones I/O de hasta 133MHz (en la esp llega a 40MHz masomenos)

#### Periféricos
- <u>Memoria:</u>
    - W25Q80DV:
        - Protocolo com: SPI
        - Precio: $477
-   <u>Sensor de presión:</u>
    -   BMP390. Podría usarse además un altimetro GPS para más precisión.
-   <u>GPS altimetro:</u>  
    -   GP-02
        -   Protocolo com: UART
        -   GNSS: BDS, GPS, GLONASS
        -   Precision: <2m
        -   Alimentacon: 3.3V
        -   Precio: $9600 + envio
    -   GlobalTop FGPMMOPA6H
    -   SparkFun GPS-RTK-SMA (ZED-F9P)
    -   u-blox NEO 8M:
        -   Protocolo com: UART, I²C, SPI
        -   GNSS: GPS, Galileo, BeiDou, GLONASS
        -   Precision: 1cm (RTK)
        -   Alimentación: 3.3V
        -   Precio: U$D 129
    -   Adafruit Ultimate GPS Module (MTK3339)
        -   Protocolo com: UART
- <u>Sensor de temperatura:</u>
    - HS3001: 
      - Protocolo com: I²C
      - Alimentación: 2.3V a 5.5V
      - Precisión: 0.25ºC
      - Rango: -10 a 80ºC
      - Precio: $3095
    - MAX31865
      - Protocolo com: 
      - Alimentación:
      - Precisión:
      - Rango: 
      - Precio: 
    - HTU21D      
      - Protocolo com: 
      - Alimentación:
      - Precisión:
      - Rango: 
      - Precio:
- <u>Acelerómetro y giroscopio:</u>
  - MPU6050:
      - Protocolo com:
      - Alimentación:
- <u>GPS:</u>

cargas pitorecnicas (despliegue nose cone y paracaidas)

### <u>Edición 2024</u>
#### Micro
-   ESP32 (Xtensa LX6):
    -   Cores: 2
    -   Clk: 240MHz
    -   GPIO: 34
    -   DAC: 2 canales, 8-bit
    -   ADC: 18 canales, 12-bits
    -   Protocolos: UART, SPI, I2C, I2S, CAN
    -   Precio: 
#### Perifericos
-   <u>Sensor de presión:</u> BMP390.
    -   Protocolo de com: I²C y SPI
    -   Precio unitario:
    -   Para medir altitud. Cons. Requiere calibración con la presión atmosférica del lugar y es muy sensible a cambios climaticos.
-   <u>Memoria:</u> W25Q80DV. 
    -   Serial flash memory. 
    -   Protocolo de com: SPI.
    -   Precio unitario: 
-   <u>Acelerometro:</u> MPU-6050. 
    -   Protocolo de com: I²C. *
    -   Precio unitario:
    -   Tiene giroscopio. 
-   <u>Pyro channels:</u>   
-   <u>Regulador de tensión:</u>
    -   Precio unitario: *

### 2023
