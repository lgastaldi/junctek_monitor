#MQTT Server Info
MQTT_HOST='IP'
MQTT_USER='user'
MQTT_PASS='pass'
# Mac Address of the Juntec BT
JUNTEC_ADDR="XX:XX:XX:XX:XX:XX"
# Battery Bank Capacity in Ah
BATT_CAP=200
# Port where the RS485 is located
RS485 = '/dev/ttyUSB0'

"""

https://38-3d.co.uk/blogs/blog/using-the-max485-rs485-module-with-the-raspberry-pi?srsltid=AfmBOorWuryoW232dp_ySrTtwgTtq2_W6958YyT9leFCnuUQoWLMBls3

RS485 = '/dev/serial0'

Wiring the MAX485 to the Raspberry Pi

MAX485 Pin    Raspberry Pi Pin   Function
----------    ----------------   -------------------------------
VCC           5V (Pin 2)         Power Supply
GND           GND (Pin 6)        Ground
RO            GPIO15 (Pin 10)    RS485 Data Receive (RX)
DI            GPIO14 (Pin 8)     RS485 Data Transmit (TX)
RE            GPIO18 (Pin 12)    Receive Enable (LOW to receive)
DE            GPIO18 (Pin 12)    Driver Enable (HIGH to send)
A             RS485 A Line       Connect to RS485 Device
B             RS485 B Line       Connect to RS485 Device

Note: The RE and DE pins are controlled together by GPIO18, enabling or disabling transmission.

Basic Python Code to Send and Receive Data via RS485

import serial
import RPi.GPIO as GPIO
import time

# Define GPIO pin for RE/DE control
RS485_CONTROL = 18  

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(RS485_CONTROL, GPIO.OUT)

# Configure the serial connection
ser = serial.Serial(
    port='/dev/serial0',  # Raspberry Pi UART port
    baudrate=9600,        # Set baud rate to match RS485 device
    timeout=1
)

def send_data(data):
    GPIO.output(RS485_CONTROL, GPIO.HIGH)  # Enable transmission
    time.sleep(0.01)  # Small delay before sending
    ser.write(data.encode())  # Send data as bytes
    time.sleep(0.01)  # Small delay to ensure data is sent
    GPIO.output(RS485_CONTROL, GPIO.LOW)  # Enable receiving

def receive_data():
    GPIO.output(RS485_CONTROL, GPIO.LOW)  # Enable reception
    data = ser.readline().decode('utf-8').strip()
    return data

try:
    while True:
        send_data("Hello RS485 Device!\n")
        print("Data sent!")

        # Wait for a response
        response = receive_data()
        if response:
            print(f"Received: {response}")

        time.sleep(2)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    GPIO.cleanup()

"""    
    
    


