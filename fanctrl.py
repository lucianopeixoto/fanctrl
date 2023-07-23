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
        pid_config = fan_config["pid"]
        return mqtt_config, fan_config, pid_config

# Get MQTT, fan, and PID configuration
mqtt_config, fan_config, pid_config = load_config()

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
hyst = fan_config["hysteresis"]

# Extract PID controller variables from the config
KP = pid_config["kp"]
KI = pid_config["ki"]
KD = pid_config["kd"]
SETPOINT = pid_config["setpoint"]
MIN_SPEED = pid_config["min_speed"]
MAX_SPEED = pid_config["max_speed"]

# Setup GPIO pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)
fan = GPIO.PWM(FAN_PIN, PWM_FREQ)
fan.start(0)

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
    global fanSpeed

    if message.topic == MQTT_TOPIC_CONTROL:
        control_command = message.payload.decode("utf-8")

        if control_command == "auto":
            # Enable automatic control
            fanSpeed = calculate_fan_speed(cpuTemp)
        elif control_command == "manual":
            # Disable automatic control
            fanSpeed = get_manual_fan_speed()

        # Update fan speed
        set_fan_speed(fanSpeed)

    elif message.topic == MQTT_TOPIC_MANUAL_SPEED:
        if control_command == "manual":
            # Set manual fan speed
            manual_speed = float(message.payload.decode("utf-8"))
            fanSpeed = manual_speed

            # Update fan speed
            set_fan_speed(fanSpeed)

# Subscribe to control and manual speed topics (only if MQTT is available)
if mqtt_client:
    mqtt_client.subscribe(MQTT_TOPIC_CONTROL)
    mqtt_client.subscribe(MQTT_TOPIC_MANUAL_SPEED)
    mqtt_client.on_message = on_message
    mqtt_client.loop_start()
else:
    # Automatic control by default without MQTT connection
    fanSpeed = calculate_fan_speed(cpuTemp)
    set_fan_speed(fanSpeed)

# PID controller variables
lastError = 0.0
integral = 0.0
last_publish_time = time.time()
publish_interval = pid_config["publish_interval"]

# Calculate fan speed using PID controller
def calculate_fan_speed(cpu_temp):
    global lastError, integral

    error = SETPOINT - cpu_temp
    integral += error * WAIT_TIME
    derivative = (error - lastError) / WAIT_TIME
    fan_speed = KP * error + KI * integral + KD * derivative
    fan_speed = min(max(fan_speed, MIN_SPEED), MAX_SPEED)

    lastError = error
    return fan_speed

# Set fan speed
def set_fan_speed(speed):
    fan.ChangeDutyCycle(speed)

try:
    while True:
        # Read CPU temperature
        cpuTempFile = open("/sys/class/thermal/thermal_zone0/temp", "r")
        cpuTemp = float(cpuTempFile.read()) / 1000
        cpuTempFile.close()

        # Calculate fan speed using PID controller
        fanSpeed = calculate_fan_speed(cpuTemp)

        # Update fan speed
        set_fan_speed(fanSpeed)

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