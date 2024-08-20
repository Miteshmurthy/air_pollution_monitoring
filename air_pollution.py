import adafruit_dht
import board
import smbus2
import blynklib
import paho.mqtt.client as mqtt
import time

# Blynk and MQTT setup
BLYNK_AUTH_TOKEN = "74HhZMC1MtjaFjKqQkvF55ndtYymyhF_"
blynk = blynklib.Blynk(BLYNK_AUTH_TOKEN)
MQTT_BROKER = "801c2d100ffe4339adcf39bdcda6f79b.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_TOPIC = "your/mqtt/topic"

# Initialize sensors
dht_device = adafruit_dht.DHT11(board.D4)
bus = smbus2.SMBus(1)

# BMP280 setup
BMP280_I2C_ADDR = 0x76
def read_bmp280():
    # Read BMP280 data
    data = bus.read_i2c_block_data(BMP280_I2C_ADDR, 0x88, 24)
    # Process and calculate temperature and pressure
    # Your logic for BMP280 temperature and pressure reading goes here
    temperature_bmp = 25.0  # Placeholder value
    pressure = 1013.25      # Placeholder value
    return temperature_bmp, pressure

# MQ and Dust Sensor Setup
def read_mq135():
    # Placeholder function for MQ135 sensor
    return 400  # Replace with actual reading logic

def read_mq2():
    # Placeholder function for MQ2 sensor
    return 300  # Replace with actual reading logic

def read_dust_sensor():
    # Placeholder function for dust sensor
    return 35  # Replace with actual reading logic

# Calculate AQI
def calculate_aqi(mq135_value, mq2_value, dust_value):
    # Combine sensor values for a simplified AQI calculation
    aqi = (mq135_value + mq2_value + dust_value) / 3
    return aqi

# MQTT setup
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker with result code " + str(rc))
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Main loop
def read_sensors():
    try:
        # Read DHT11 sensor data
        temperature_dht = dht_device.temperature
        humidity = dht_device.humidity

        # Read BMP280 sensor data
        temperature_bmp, pressure = read_bmp280()

        # Read MQ135, MQ2, and dust sensor data
        mq135_value = read_mq135()
        mq2_value = read_mq2()
        dust_value = read_dust_sensor()

        # Calculate AQI
        aqi = calculate_aqi(mq135_value, mq2_value, dust_value)

        return temperature_dht, humidity, temperature_bmp, pressure, mq135_value, mq2_value, dust_value, aqi

    except RuntimeError as error:
        print(f"Error reading sensors: {error}")
        return None, None, None, None, None, None, None, None

while True:
    # Read sensor values
    temperature_dht, humidity, temperature_bmp, pressure, mq135_value, mq2_value, dust_value, aqi = read_sensors()

    if temperature_dht is not None:
        # Display on Blynk
        blynk.virtual_write(1, temperature_dht)
        blynk.virtual_write(2, humidity)
        blynk.virtual_write(3, temperature_bmp)
        blynk.virtual_write(4, pressure)
        blynk.virtual_write(5, mq135_value)
        blynk.virtual_write(6, mq2_value)
        blynk.virtual_write(7, dust_value)
        blynk.virtual_write(8, aqi)

        # Publish to MQTT
        mqtt_client.publish(MQTT_TOPIC + "/temperature", temperature_dht)
        mqtt_client.publish(MQTT_TOPIC + "/humidity", humidity)
        mqtt_client.publish(MQTT_TOPIC + "/bmp_temperature", temperature_bmp)
        mqtt_client.publish(MQTT_TOPIC + "/pressure", pressure)
        mqtt_client.publish(MQTT_TOPIC + "/mq135", mq135_value)
        mqtt_client.publish(MQTT_TOPIC + "/mq2", mq2_value)
        mqtt_client.publish(MQTT_TOPIC + "/dust", dust_value)
        mqtt_client.publish(MQTT_TOPIC + "/aqi", aqi)

    blynk.run()
    mqtt_client.loop_start()
    time.sleep(5)  # Delay between readings
