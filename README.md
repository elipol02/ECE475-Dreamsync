# Dreamsync

## Project Overview
Our product Dreamsync is designed to enhance health monitoring by integrating sensors that track various metrics related to sleep quality and environmental conditions. Data collected from the sensors are transmitted to a companion mobile application via Bluetooth for analysis and visualization. This will make it easier for our users to track their sleeping patterns.

### Features
- **Sleep Movement Monitoring**: Tracks body movements during sleep using an MPU6050 accelerometer and gyroscope.
- **Heart Rate Monitoring**: Continuously monitors heart rate using a photoplethysmography (PPG) sensor.
- **Environmental Light Exposure**: Measures light levels in the sleep environment using light sensors.
- **Ambient Temperature Monitoring**: Tracks the ambient temperature using a DS18B20 temperature sensor.
- **Battery Monitoring**: Measures battery voltage, percentage, and charging status.
- **Bluetooth Connectivity**: Transmits real-time sensor data to a companion app via BLE (Bluetooth Low Energy).

---

## Components and Dependencies

### Hardware Components
- **MPU6050**: Accelerometer and gyroscope for motion tracking.
- **DS18B20**: Temperature sensor.
- **PPG Sensor**: For heart rate monitoring.
- **Light Sensors**: Measures light levels (connected to analog pins A0 and A1).
- **nRF52840 Feather**: For Bluetooth communication.
- **LED**: Visual feedback for heart rate peaks.

### Software Libraries
- **Wire**: I2C communication with MPU6050.
- **OneWire**: Communication protocol for DS18B20.
- **DallasTemperature**: Interface for DS18B20 temperature sensor.
- **Adafruit Bluefruit**: Bluetooth communication.
- **Adafruit LittleFS**: Internal file system for configuration storage.

---

## Setup Instructions

### Hardware Setup
1. Connect the **MPU6050** to the I2C pins (SDA, SCL).
2. Attach the **DS18B20** sensor to `TEMP_SENSOR_PIN` (A2).
3. Connect **PPG Sensor** to `HEART_RATE_PIN` (A3).
4. Connect light sensors to `LIGHT_SENSOR_1` (A0) and `LIGHT_SENSOR_2` (A1).
5. Connect the LED to `LED_PIN` (3).

### Software Setup
1. You'll need to install the libraries we listed above.
2. Upload the firmware code provided to your nRF52840 development board via Arduino IDE.

### Mobile Application
1. Ensure the Bluetooth is enabled on your mobile device.
2. Pair with the "SleepMask" device via BLE.
3. Use the companion app to visualize and analyze the collected data.

---

## Code Highlights

### Key Functions
- **`initializeMPU6050()`**: Initializes the MPU6050 for motion tracking.
- **`readLightAndTemp()`**: Reads ambient light and temperature values.
- **`readHeartRateSensor()`**: Reads and processes heart rate data.
- **`calculateBPM()`**: Calculates beats per minute (BPM) based on PPG data.
- **`sendEnvironmentalData()`**: Sends light and temperature data over BLE.
- **`checkBattery()`**: Monitors battery voltage and percentage.
- **`startAdv()`**: Configures and starts BLE advertising.

### BLE Services
- **Device Information Service**: Provides metadata about the device.
- **UART Service**: Transmits sensor data to the mobile app.
- **Battery Service**: Sends battery status updates.

---

## Sensor Data Format

The sensor data is transmitted to the companion app in the following format:
1. **Environmental Data**: `E,light1,light2,temperature`
2. **Heart Rate Data**: `H,heartRate`
3. **Motion Data**: `M,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,pitch,roll,motionIntensity`
4. **Battery Data**: `B,batteryVoltage,batteryPercent,isCharging`

---

## Contributors

Chukwuemeka Mordi, Chu-en Chang, Elijah Pollack, Kinner Parikh
