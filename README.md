# fanctrl - RPi Advanced Fan Control

This is a Python-based application for controlling the fan speed of a Raspberry Pi based on the CPU temperature. The fan control is implemented using the Raspberry Pi's GPIO pins and can be controlled automatically based on the temperature or manually via MQTT commands.

## Prerequisites
- Raspberry Pi with Raspbian OS (or compatible)
- Python 3.x
- RPi.GPIO library (RPi.GPIO)
- Paho MQTT library (paho.mqtt.client)

## Installation
1. Clone the repository to your Raspberry Pi:
   git clone https://github.com/your-username/fan-control-application.git

2. Install the required Python libraries:
   pip install RPi.GPIO paho-mqtt

## Configuration
The application uses a configuration file (config.json) to specify MQTT and fan-related settings. Before running the application, make sure to update the config.json file with your specific configuration:

```json
{
  "mqtt": {
    "broker": "your_broker_ip",
    "port": 1883,
    "topic_temperature": "temperature",
    "topic_fan_speed": "fan_speed",
    "topic_control": "fan_control",
    "topic_manual_speed": "manual_fan_speed"
  },
  "fan": {
    "pin": 12,
    "wait_time": 1,
    "pwm_freq": 60,
    "temp_steps": [65, 70],
    "speed_steps": [30, 100],
    "hysteresis": 1
  }
}
```

- "broker": IP address or hostname of the MQTT broker.
- "port": Port number of the MQTT broker.
- "topic_temperature": MQTT topic for publishing CPU temperature.
- "topic_fan_speed": MQTT topic for publishing fan speed.
- "topic_control": MQTT topic for controlling fan mode (auto/manual).
- "topic_manual_speed": MQTT topic for setting manual fan speed.
- "pin": BCM pin used to drive the transistor's base for fan control.
- "wait_time": Time to wait between each refresh in seconds.
- "pwm_freq": Frequency of the PWM signal for the fan control.
- "temp_steps": Temperature steps for fan speed calculation.
- "speed_steps": Corresponding fan speed steps for the temperature steps.
- "hysteresis": Temperature difference at which the fan speed will change.

## Usage
1. Run the fan control application:
   python fan_control.py

2. Monitor the fan control and temperature updates:
   - Subscribe to the MQTT topic specified in "topic_temperature" for CPU temperature updates.
   - Subscribe to the MQTT topic specified in "topic_fan_speed" for fan speed updates.

3. Control the fan mode:
   - To switch to automatic mode, publish the command "auto" to the MQTT topic specified in "topic_control".
   - To switch to manual mode, publish the command "manual" to the MQTT topic specified in "topic_control".

4. Set the manual fan speed:
   - Publish the desired fan speed value as a float number to the MQTT topic specified in "topic_manual_speed". The value represents the fan speed as a percentage (e.g., 50.0 for 50% speed).

## License
This application is licensed under the GNU General Public License v3.0.

Please make sure to include the LICENSE file in your repository and update the documentation accordingly to reflect the correct license information.

Feel free to customize the documentation further and add any additional sections or information as needed.
