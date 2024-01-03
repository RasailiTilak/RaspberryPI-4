

import paho.mqtt.client as mqtt
import json
import time
import datetime
import RPi.GPIO as GPIO
import minimalmodbus
import requests
import threading

# URL of the PHP script
url = 'http://ip-of-server/mini-farm/rsp-php-14.php'  # Replace with your actua                                                                                                                                                             l URL

# Modbus RTU device settings
environment_sensor = minimalmodbus.Instrument('/dev/ttyUSB0', 2)  # Port and add                                                                                                                                                             ress configuration
nutrient_sensor = minimalmodbus.Instrument('/dev/ttyUSB0', 7)  # Port and addres                                                                                                                                                             s configuration

environment_sensor.serial.baudrate = 9600
environment_sensor.serial.bytesize = 8
environment_sensor.serial.parity = minimalmodbus.serial.PARITY_NONE
environment_sensor.serial.stopbits = 1
environment_sensor.serial.timeout = 0.1

nutrient_sensor.serial.baudrate = 9600
nutrient_sensor.serial.bytesize = 8
nutrient_sensor.serial.parity = minimalmodbus.serial.PARITY_NONE
nutrient_sensor.serial.stopbits = 1
nutrient_sensor.serial.timeout = 0.1

# Sensor addresses
Co2_register_address = 0
TEMP_register_address = 1
HUMI_register_address = 2


EC_register_address = 0
PH_register_address = 1
WT_register_address = 2

data_collection_interval = 10  # Data collection interval in seconds

# GPIO pin numbers for devices
light_pin = 5  # Replace with the actual GPIO pin number for the light
pump_pin = 6  # Replace with the actual GPIO pin number for the pump
fan_pin = 13    # Replace with the actual GPIO pin number for the fan
heater_pin = 19 # Replace with the actual GPIO pin number for the heater
cooler_pin = 26 # Replace with the actual GPIO pin number for the cooler

# Set up GPIO mode and initial states
GPIO.setmode(GPIO.BCM)
GPIO.setup(light_pin, GPIO.OUT)
GPIO.setup(pump_pin, GPIO.OUT)
GPIO.setup(fan_pin, GPIO.OUT)
GPIO.setup(heater_pin, GPIO.OUT)
GPIO.setup(cooler_pin, GPIO.OUT)

# MQTT Broker Configuration
broker_address = "your-MQTT-BROKER-IP"
publish_topic = "MQTT-topic" # topic to publish from the gateway and subscribe in web
subscribe_topic = "MQTT-TOIC"  # topic to subscribe in gatewy and publish from the web

# Button and input mappings
button_mappings = {
    "button1": "autMan",
    "button2": "light",
    "button3": "pump",
    "button4": "fan",
    "button5": "heater",
    "button6": "cooler",
}

input_mappings = {
    "inputNumber1": "light_on_h",
    "inputNumber2": "light_on_m",
    "inputNumber3": "light_off_h",
    "inputNumber4": "light_off_m",
    "inputNumber5": "pump_on",
    "inputNumber6": "pump_off",
    "inputNumber7": "fan_on",
    "inputNumber8": "fan_off",
    "inputNumber9": "temp_set",
}

# Variables to store values
autMan, light, pump, fan, heater, cooler = [None] * 6
light_on_h, light_on_m, light_off_h, light_off_m = [None] * 4
pump_on, pump_off, fan_on, fan_off, temp_set = [None] * 5


time1 = time2 = time3 = time4 = time5 = time6 = None
F_time_set_1 = F_time_set_2 = P_time_set_1 = P_time_set_2 = 0
F_time_set = F_time = P_time_set = P_time = False
start_time = None
RTC=None


def read_sensor_data(sensor):
    try:
        temp_value = environment_sensor.read_register(TEMP_register_address, functioncode=3) / 100
        humi_value = environment_sensor.read_register(HUMI_register_address, functioncode=3) / 100
        co2_value = environment_sensor.read_register(Co2_register_address, functioncode=3)
        EC_value = nutrient_sensor.read_register(EC_register_address, functioncode=3) /100
        PH_value = nutrient_sensor.read_register(PH_register_address, functioncode=3) /10
        WT_value = (nutrient_sensor.read_register(WT_register_address, functioncode=3) -550) /10

        # temp = temp_value
        # humi = humi_value
        # co2 = co2_value
        # EC = EC_value
        # PH = PH_value
        # WT = WT_value


        return {
            'co2': co2_value,
            'temperature': temp_value,
            'humidity': humi_value,
            'ec': EC_value,
            'ph': PH_value,
            'wt': WT_value,
            'inputNumber1': light_on_h,
            'inputNumber2': light_on_m,
            'inputNumber3': light_off_h,
            'inputNumber4': light_off_m,
            'inputNumber5': pump_on,
            'inputNumber6': pump_off,
            'inputNumber7': fan_on,
            'inputNumber8': fan_off,
            'inputNumber9': temp_set,

            'button1': autMan,
            'button2': light,
            'button3': pump,
            'button4': fan,
            'button5': heater,
            'button6': cooler,
            'time':RTC,


        }
    except Exception as e:
        print(f"Error reading sensor data: {e}")
        return None

    except minimalmodbus.ModbusException as e:
        print(f"Modbus Exception: {e}")

def send_sensor_data(data):
    try:
        response = requests.post(url, data=data)
        print("Data sent - Response:", response.text)
    except Exception as e:
        print(f"Error sending data: {e}")

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe(subscribe_topic)
    else:
        print("Connection failed with code", rc)

def on_message(client, userdata, msg):
    received_message = msg.payload.decode()
    print("Received message:", received_message)

    # Parse the received JSON message
    data = json.loads(received_message)

    # Check conditions and store values in variables
    for key, variable in button_mappings.items():
        if "button" in data and data["button"] == key:
            globals()[variable] = data["status"]
            print(f"{variable.capitalize()} Status:", globals()[variable])

    for key, variable in input_mappings.items():
        if "inputNumber" in data and data["inputNumber"] == key:
            globals()[variable] = data["value"]
            print(f"{variable.capitalize()}: {globals()[variable]}")

# MQTT Client Setup
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
print("Connecting to MQTT broker...")
client.connect(broker_address, 1883, 60)

# Run the client loop to process callbacks
client.loop_start()

def Mode_S():
    print("MODE S START")
    global autMan, light, pump, fan, heater, cooler
    if autMan == "ON" and light_on_h is not None and light_off_h is not None and light_off_m is not None and light_on_m is not None and light_on_h is not 00 and light_off_h is not 00 :
        Auto_Light()
        auto_pump()
        auto_fan()
        # Auto_Cooling()
        # Auto_Heating()
    elif autMan == "OFF":
        Manual_Start()
        F_time_set_1 = 0
        F_time_set_2 = 0
        P_time_set_1 = 0
        P_time_set_2 = 0

def millis():
    return int(round(time.time() * 1000))

# Function to control devices based on their status
def Manual_Start():
    print("Manual Start")
    GPIO.output(light_pin, GPIO.HIGH if light == "ON" else GPIO.LOW)
    GPIO.output(pump_pin, GPIO.HIGH if pump == "ON" else GPIO.LOW)
    GPIO.output(fan_pin, GPIO.HIGH if fan == "ON" else GPIO.LOW)
    GPIO.output(heater_pin, GPIO.HIGH if heater == "ON" else GPIO.LOW)
    GPIO.output(cooler_pin, GPIO.HIGH if cooler == "ON" else GPIO.LOW)

# Auto Mode Start
def Auto_Light():
    print("Auto Light")
    now = datetime.datetime.now()
    RTC = now.time()  # Get only time information, excluding year-month-day
    print("Current Time:", RTC)
    global Web_L_ON, Web_L_OFF
    Web_L_ON = datetime.datetime.strptime(f"{light_on_h}:{light_on_m}", "%H:%M").time()
    Web_L_OFF = datetime.datetime.strptime(f"{light_off_h}:{light_off_m}", "%H:%M").time()
    print("Setting Time:", Web_L_ON)
    print("Setting Time:", Web_L_OFF)

    # Web_L_ON_time, Web_L_OFF_time in time format and compare
    if Web_L_ON is not None and Web_L_OFF is not None:
        if Web_L_ON <= RTC <= Web_L_OFF and autMan == 'ON':
            GPIO.output(light_pin, GPIO.HIGH)
            print("A_L_ON")
        else:
            GPIO.output(light_pin, GPIO.LOW)
            print("A_L_OFF")


start_time = millis()

def auto_pump():
    global P_time_set, P_time_set_1, P_time_set_2, P_time

    if P_time == False:
        P_time_set_2 = millis() + int(pump_off) * 60000

    if P_time_set == False:
        P_time_set_1 = millis() + int(pump_on) * 1000

    if pump_on is not None and pump_off is not None:
        if int(pump_on) >= 1 and autMan == 'ON':
            P_time_set = True
            GPIO.output(pump_pin, GPIO.HIGH)
            print("pump_ON")
            if P_time_set_1 < millis():
                P_time = True
                GPIO.output(pump_pin, GPIO.LOW)
                print("pump_OFF")
            if P_time_set_2 < millis():
                P_time_set = False
                P_time = False
                P_time_set_1 = 0
                P_time_set_2 = 0
                return

def auto_fan():
    global F_time_set, F_time_set_1, F_time_set_2, F_time

    if F_time == False:
        F_time_set_2 = millis() + int(fan_off) * 60000

    if F_time_set == False:
        F_time_set_1 = millis() + int(fan_on) * 60000

    if fan_on is not None and fan_off is not None:
        if int(fan_on) >= 1  and autMan == 'ON':
            F_time_set = True
            GPIO.output(fan_pin, GPIO.HIGH)
            print("Fan_ON")
            if F_time_set_1 < millis():
                F_time = True
                print("Fan_OFF")
                GPIO.output(fan_pin, GPIO.LOW)
            if F_time_set_2 < millis():
                F_time_set = False
                F_time = False
                F_time_set_1 = 0
                F_time_set_2 = 0
                return
def send_sensor_data_php(data):
    try:
        response = requests.post(url, data=data)
        print("Data sent - Response:", response.text)
    except Exception as e:
        print(f"Error sending data: {e}")


def collect_sensor_data():
    while True:
        environment_data = read_sensor_data(environment_sensor)  # Read environment sensor data
        nutrient_data = read_sensor_data(nutrient_sensor)  # Read nutrient sensor data

        if environment_data and nutrient_data:
            combined_data = {**environment_data, **nutrient_data}  # Combine both sensor data
            client.publish(publish_topic, json.dumps(combined_data))  # Publish combined data

        Mode_S()

        time.sleep(2)

def send_combined_data():
    while True:
        environment_data = read_sensor_data(environment_sensor)
        nutrient_data = read_sensor_data(nutrient_sensor)

        if environment_data and nutrient_data:
            combined_data = {**environment_data, **nutrient_data}
            print("Generated Data:", combined_data)
            send_sensor_data_php(combined_data)
        else:
            print("Error reading sensor data")

        time.sleep(60)

if __name__ == "__main__":
    thread1 = threading.Thread(target=collect_sensor_data)
    thread2 = threading.Thread(target=send_combined_data)

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()
