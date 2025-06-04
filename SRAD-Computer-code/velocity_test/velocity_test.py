import os
import serial
import serial.tools.list_ports as ls_ports
import time

YELLOW = "\033[93m"
GREEN = "\033[92m"
RED = "\033[91m"
WHITE = "\033[0m"

PORT_PATH = '/dev/ttyUSB0'

uart = 0

connected = True
velocity = 0
last_vel = 0

state = 'DISCONNECTED'

while (True):
    if (state == 'CONNECTED'):
        try:
            # velocity = float(((uart.readline().decode('utf-8').strip()).split(',')[0]).split(':')[-1])
            # vel_km_h = round(velocity * 3.6, 2)

            line = uart.readline().decode('utf-8').strip()
            if line:
                os.system('clear')
                print(line)

            # if velocity:
            #     os.system('clear')
            #     if velocity == 0:   
            #         # print(f"{YELLOW}{vel_km_h}{WHITE}\n")
            #         print(f"{YELLOW}{velocity}{WHITE}")
            #     elif velocity > 0:
            #         # print(f"{GREEN}{vel_km_h}{WHITE}\n")
            #         print(f"{GREEN}{velocity}{WHITE}")
            #     elif velocity < 0:
            #         # print(f"{RED}{vel_km_h}{WHITE}\n")
            #         print(f"{RED}{velocity}{WHITE}")
                    
        except serial.SerialException: 
            state = 'DISCONNECTED'
            uart.close()
            time.sleep(1)
        except ValueError:
            os.system('clear')
            print(f"{GREEN}Connected{WHITE} - Invalid data received")
            time.sleep(1)

    elif (state == 'DISCONNECTED'):
        os.system('clear')
        print(f"RP-2040 {RED}Disconnected{WHITE}")

        ports = [p.device for p in ls_ports.comports()]

        if PORT_PATH in ports:
            try:
                uart = serial.Serial(PORT_PATH, 115200, timeout=1)
                if uart.is_open:
                    state = 'CONNECTED'
                elif not uart.is_open:
                    uart.open()
                    state = 'CONNECTED'
            except:
                pass
        
        time.sleep(1)