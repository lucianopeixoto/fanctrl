#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import sys
import paho.mqtt.client as mqtt
import json

# Configuration file path
CONFIG_FILE = "config.json"

# Load MQTT and fan configuration from JSON config file
def load_config():
    with open(CONFIG_FILE) as f:
        config = json.load(f)
        mqtt_config = config["mqtt"]
        fan_config = config["fan"]
        return mqtt_config, fan_config

# Get MQTT and fan configuration
mqtt_config, fan_config = load_config()

# Extract MQTT variables from the config
MQTT_BROKER = mqtt_config["broker"]
MQTT_PORT = mqtt_config["port"]
MQTT_TOPIC_TEMPERATURE = mqtt_config["topic_temperature"]
MQTT_TOPIC_FAN_SPEED = mqtt_config["topic_fan_speed"]
MQTT_TOPIC_CONTROL = mqtt_config["topic_control"]
MQTT_TOPIC_MANUAL_SPEED = mqtt_config["topic_manual_speed"]

# Extract fan variables from the config
FAN_PIN = fan_config["pin"]
WAIT_TIME = fan_config["wait_time"]
PWM_FREQ = fan_config["pwm_freq"]
tempSteps = fan_config["temp_steps"]
speedSteps = fan_config["speed_steps"]
hyst = fan_config["hysteresis"]

# Setup GPIO pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)
fan = GPIO.PWM(FAN_PIN, PWM_FREQ)
fan.start(0)

i = 0
cpuTemp = 0
fanSpeed = 100
cpuTempOld = 0
fanSpeedOld = 0

# MQTT client instance
mqtt_client = None

# Check if MQTT configuration is available
if MQTT_BROKER and MQTT_PORT:
    # Create MQTT client instance
    mqtt_client = mqtt.Client()

    # Connect to MQTT broker
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT)

# Publish temperature and fan speed (only if MQTT is available)
def publish_values(temperature, fan_speed):
    if mqtt_client:
        mqtt_client.publish(MQTT_TOPIC_TEMPERATURE, str(temperature))
        mqtt_client.publish(MQTT_TOPIC_FAN_SPEED, str(fan_speed))

# Callback function for MQTT message received
def on_message(client, userdata, message):
    global fanSpeed, fanSpeedOld

    if message.topic == MQTT_TOPIC_CONTROL:
        control_command = message.payload.decode("utf-8")

        if control_command == "auto":
            # Enable automatic control
            fanSpeed = calculate_fan_speed(cpuTemp)
        elif control_command == "manual":
            # Disable automatic control
            fanSpeed = get_manual_fan_speed()

        if fanSpeed != fanSpeedOld:
            fan.ChangeDutyCycle(fanSpeed)
            fanSpeedOld = fanSpeed

    elif message.topic == MQTT_TOPIC_MANUAL_SPEED:
        if control_command == "manual":
            # Set manual fan speed
            manual_speed = float(message.payload.decode("utf-8"))
            fanSpeed = manual_speed

            if fanSpeed != fanSpeedOld:
                fan.ChangeDutyCycle(fanSpeed)
                fanSpeedOld = fanSpeed

# Subscribe to control and manual speed topics (only if MQTT is available)
if mqtt_client:
    mqtt_client.subscribe(MQTT_TOPIC_CONTROL)
    mqtt_client.subscribe(MQTT_TOPIC_MANUAL_SPEED)
    mqtt_client.on_message = on_message
    mqtt_client.loop_start()
else:
    # Automatic control by default without MQTT connection
    fanSpeed = calculate_fan_speed(cpuTemp)
    fan.ChangeDutyCycle(fanSpeed)
    fanSpeedOld = fanSpeed

try:
    while True:
        # Read CPU temperature
        cpuTempFile = open("/sys/class/thermal/thermal_zone0/temp", "r")
        cpuTemp = float(cpuTempFile.read()) / 1000
        cpuTempFile.close()

        # Calculate desired fan speed
        if abs(cpuTemp - cpuTempOld) > hyst:
            # Below first value, fan will not run.
            if cpuTemp < tempSteps[0]:
                fanSpeed = 0
            # Above last value, fan will run at max speed
            elif cpuTemp >= tempSteps[-1]:
                fanSpeed = speedSteps[-1]
            # If temperature is between 2 steps, fan speed is calculated by linear interpolation
            else:
                for i in range(len(tempSteps) - 1):
                    if tempSteps[i] <= cpuTemp < tempSteps[i + 1]:
                        fanSpeed = round((speedSteps[i + 1] - speedSteps[i]) / (tempSteps[i + 1] - tempSteps[i]) * (cpuTemp - tempSteps[i]) + speedSteps[i], 1)

            if fanSpeed != fanSpeedOld:
                fan.ChangeDutyCycle(fanSpeed)
                fanSpeedOld = fanSpeed

            cpuTempOld = cpuTemp

        # Check if enough time has passed since the last publish (only if MQTT is available)
        if mqtt_client and (time.time() - last_publish_time >= publish_interval):
            # Publish temperature and fan speed
            publish_values(cpuTemp, fanSpeed)
            last_publish_time = time.time()

        # Wait until next refresh
        time.sleep(WAIT_TIME)

# If a keyboard interrupt occurs (ctrl + c), the GPIO is set to 0 and the program exits.
except KeyboardInterrupt:
    print("Fan control interrupted by keyboard")
    GPIO.cleanup()
    if mqtt_client:
        mqtt_client.loop_stop()
    sys.exit()