import time
import Adafruit_DHT
import smbus2
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import blynklib
from bmp280 import BMP280

# HiveMQ MQTT Broker Settings
MQTT_BROKER = "801c2d100ffe4339adcf39bdcda6f79b.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_USER = “miteshmurthy”
MQTT_PASSWORD = “Mitesh12”
MQTT_TOPIC = "sensor/data"

# Blynk Authentication Token
BLYNK_AUTH = "74HhZMC1MtjaFjKqQkvF55ndtYymyhF_"

# GPIO Pin Definitions
DHT_PIN = 4
DHT_SENSOR = Adafruit_DHT.DHT11
MQ135_PIN = 17
MQ2_PIN = 27
DUST_SENSOR_LED_PIN = 22
DUST_SENSOR_VOUT_PIN = 23

# Initialize Blynk and MQTT clients
blynk = blynklib.Blynk(BLYNK_AUTH)
mqtt_client = mqtt.Client()

# BMP280 Setup
bus = smbus2.SMBus(1)
bmp280 = BMP280(i2c_dev=bus)

# MQTT Setup
def setup_mqtt():
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()

# Function to read sensor data
def read_sensors():
    humidity, temperature_dht = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
    temperature_bmp = bmp280.get_temperature()
    pressure = bmp280.get_pressure()
    
    # Add your MQ135, MQ2, and dust sensor readings here
    dust_value = read_dust_sensor()
    mq135_value = GPIO.input(MQ135_PIN)
    mq2_value = GPIO.input(MQ2_PIN)
    aqi = calculate_aqi(mq135_value, mq2_value, dust_value)
    
    return temperature_dht, humidity, temperature_bmp, pressure, mq135_value, mq2_value, dust_value, aqi

def read_dust_sensor():
    GPIO.output(DUST_SENSOR_LED_PIN, GPIO.LOW)  # Turn on LED
    time.sleep(0.00028)
    dust_value = GPIO.input(DUST_SENSOR_VOUT_PIN)
    time.sleep(0.00004)
    GPIO.output(DUST_SENSOR_LED_PIN, GPIO.HIGH)  # Turn off LED
    time.sleep(0.00968)
    return dust_value

def calculate_aqi(mq135, mq2, dust):
    # Simplified AQI calculation
    mq135_contrib = mq135 / 1024.0 * 100.0
    mq2_contrib = mq2 / 1024.0 * 100.0
    dust_contrib = dust / 1024.0 * 100.0
    return (mq135_contrib + mq2_contrib + dust_contrib) / 3.0

# Function to send data to Blynk and MQTT broker
def send_data(temperature_dht, humidity, temperature_bmp, pressure, mq135_value, mq2_value, dust_value, aqi):
    payload = {
        "temperature_dht": temperature_dht,
        "humidity": humidity,
        "temperature_bmp": temperature_bmp,
        "pressure": pressure,
        "mq135": mq135_value,
        "mq2": mq2_value,
        "dust": dust_value,
        "aqi": aqi
    }
    mqtt_client.publish(MQTT_TOPIC, str(payload))

    # Send data to Blynk
    blynk.virtual_write(0, temperature_dht)
    blynk.virtual_write(1, humidity)
    blynk.virtual_write(2, temperature_bmp)
    blynk.virtual_write(3, pressure)
    blynk.virtual_write(4, dust_value)
    blynk.virtual_write(5, aqi)

# Main loop
if _name_ == "_main_":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MQ135_PIN, GPIO.IN)
    GPIO.setup(MQ2_PIN, GPIO.IN)
    GPIO.setup(DUST_SENSOR_LED_PIN, GPIO.OUT)
    GPIO.setup(DUST_SENSOR_VOUT_PIN, GPIO.IN)

    setup_mqtt()
    
    while True:
        temperature_dht, humidity, temperature_bmp, pressure, mq135_value, mq2_value, dust_value, aqi = read_sensors()
        send_data(temperature_dht, humidity, temperature_bmp, pressure, mq135_value, mq2_value, dust_value, aqi)
        blynk.run()
        time.sleep(2)  # Delay between readings