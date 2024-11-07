#include <SoftwareSerial.h>
#include <Wire.h>

// Pin definitions for Adafruit Feather nRF52840
#define LIGHT_SENSOR_1_PIN A0  // Use A0 (P0.26)
#define LIGHT_SENSOR_2_PIN A1  // Use A1 (P0.27)
#define TEMP_SENSOR_PIN A2     // Use A2 (P0.28)
#define HEART_RATE_PIN A3      // Use A3 (P0.29)

// Bluetooth module pins (if using SoftSerial)
#define BT_RX 2  // Use GPIO2 (P0.02)
#define BT_TX 3  // Use GPIO3 (P0.03)

// Battery monitoring stuff
#define BATTERY_PIN A7      // feather's battery pin
#define VBAT_DIVIDER 2      // the nRF52840's built-in voltage divider
#define VBAT_MAX 4.25       // max voltage for 472228 battery
#define VBAT_MIN 2.75       // min voltage for 472228 battery

#define MC3419_ADDR 0x4C    // I2C address

// MC3419 Registers
#define REG_MODE 0x07       // Mode control
#define REG_SR 0x08         // Sample rate
#define REG_RANGE 0x20      // Range control
#define REG_XOUT_LSB 0x0D   // Data registers
#define REG_XOUT_MSB 0x0E
#define REG_YOUT_LSB 0x0F
#define REG_YOUT_MSB 0x10
#define REG_ZOUT_LSB 0x11
#define REG_ZOUT_MSB 0x12

// MC3419 settings
#define MODE_STANDBY 0x00
#define MODE_WAKE 0x01
#define SR_125HZ 0x03       // 125Hz output rate
#define RANGE_2G 0x00       // ±2g range

// Create Bluetooth serial connection
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Timing variables (all in milliseconds)
const unsigned long LIGHT_TEMP_INTERVAL = 300000;  // 5 minutes
const unsigned long HEART_RATE_INTERVAL = 120000;  // 2 minutes
const unsigned long ACCEL_INTERVAL = 100;          // 100ms (10Hz)
const unsigned long BATTERY_INTERVAL = 1000;       // 1 second

unsigned long lastLightTempReading = 0;
unsigned long lastHeartRateReading = 0;
unsigned long lastAccelReading = 0;
unsigned long lastBatteryReading = 0;

// Sensor state management
enum SensorState {
    LIGHT_TEMP,
    HEART_RATE,
    ACCELEROMETER,
    BATTERY_CHECK,
    IDLE
};

SensorState currentState = IDLE;

// Buffer for sensor readings
struct SensorData {
    int light1;
    int light2;
    float temperature;
    int heartRate;
    int16_t accelRawX;
    int16_t accelRawY;
    int16_t accelRawZ;
    float accelG_X;
    float accelG_Y;
    float accelG_Z;
    float motionIntensity;
    float batteryVoltage;
    int batteryPercent;
    bool isCharging;
} sensorData;

const int SAMPLES = 100;
int ppgBuffer[SAMPLES];
int bufferIndex = 0;

// Heart rate detection variables
float threshold = 0.5;  // Threshold for peak detection
int peakCount = 0;
unsigned long lastPeakTime = 0;
float accelSensitivity = 16384.0f;  // LSB/g for ±2g range
bool accelInitialized = false;

// Write helper for the MC3419
void writeAccelRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MC3419_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Read accelerometer
void readAccelerometer() {
    if (!accelInitialized) return;

    Wire.beginTransmission(MC3419_ADDR);
    Wire.write(REG_XOUT_LSB);
    Wire.endTransmission(false);
    Wire.requestFrom(MC3419_ADDR, 6);

    if (Wire.available() >= 6) {
        uint8_t xL = Wire.read();
        uint8_t xH = Wire.read();
        uint8_t yL = Wire.read();
        uint8_t yH = Wire.read();
        uint8_t zL = Wire.read();
        uint8_t zH = Wire.read();

        sensorData.accelRawX = (xH << 8) | xL;
        sensorData.accelRawY = (yH << 8) | yL;
        sensorData.accelRawZ = (zH << 8) | zL;

        // Converting to g (16384 LSB/g for ±2g range)
        sensorData.accelG_X = sensorData.accelRawX / 16384.0;
        sensorData.accelG_Y = sensorData.accelRawY / 16384.0;
        sensorData.accelG_Z = sensorData.accelRawZ / 16384.0;

        sensorData.motionIntensity = sqrt(
            sensorData.accelG_X * sensorData.accelG_X +
            sensorData.accelG_Y * sensorData.accelG_Y +
            sensorData.accelG_Z * sensorData.accelG_Z
        );
    }
}

bool initializeMC3419() {
    Wire.begin();

    // Initialization sequence
    writeAccelRegister(REG_MODE, MODE_STANDBY);
    delay(10);

    writeAccelRegister(REG_RANGE, RANGE_2G);
    writeAccelRegister(REG_SR, SR_125HZ);

    writeAccelRegister(REG_MODE, MODE_WAKE);
    delay(3);

    return true;
}

void setup() {
    // Initialize serial communications
    Serial.begin(9600);
    bluetooth.begin(9600);

    // Initialize analog pins
    pinMode(LIGHT_SENSOR_1_PIN, INPUT);
    pinMode(LIGHT_SENSOR_2_PIN, INPUT);
    pinMode(TEMP_SENSOR_PIN, INPUT);
    pinMode(HEART_RATE_PIN, INPUT);

    // Initializing the accelerometer
    accelInitialized = initializeMC3419();
    if (!accelInitialized) {
        Serial.println("Could not initialize MC3419!");
    }
}

void loop() {
    unsigned long currentTime = millis();

    // see if it's time to read light and temperature sensors
    if (currentTime - lastLightTempReading >= LIGHT_TEMP_INTERVAL) {
        currentState = LIGHT_TEMP;
        lastLightTempReading = currentTime;
    }

    // see if it's time to read heart rate sensor
    if (currentTime - lastHeartRateReading >= HEART_RATE_INTERVAL) {
        currentState = HEART_RATE;
        lastHeartRateReading = currentTime;
    }

    // see if it's time to read accelerometer
    if (currentTime - lastAccelReading >= ACCEL_INTERVAL) {
        currentState = ACCELEROMETER;
        lastAccelReading = currentTime;
    }

    // see if it's time to check the battery
    if (currentTime - lastBatteryReading >= BATTERY_INTERVAL) {
        currentState = BATTERY_CHECK;
        lastBatteryReading = currentTime;
    }

    // Process the current state
    processCurrentState();

    // Small delay to prevent overwhelming the system
    delay(10);
}

void processCurrentState() {

    int tempReading;
    float voltage;

    switch (currentState) {
        case LIGHT_TEMP:
            sensorData.light1 = analogRead(LIGHT_SENSOR_1_PIN);
            sensorData.light2 = analogRead(LIGHT_SENSOR_2_PIN);

            // convert analog reading to temperature in Celsius
            tempReading = analogRead(TEMP_SENSOR_PIN);
            voltage = tempReading * (5.0 / 1024.0);
            sensorData.temperature = (voltage - 0.5) * 100;

            sendEnvironmentalData();
            currentState = IDLE;
            break;

        case HEART_RATE:
            sensorData.heartRate = readHeartRateSensor(HEART_RATE_PIN);
            sendHeartRateData();
            currentState = IDLE;
            break;

        case ACCELEROMETER:
            readAccelerometer();
            sendAccelData();
            currentState = IDLE;
            break;

        case BATTERY_CHECK:
            checkBattery();
            sendBatteryData();
            currentState = IDLE;
            break;

        case IDLE:
            // Do nothing in idle state
            break;
    }
}

int readHeartRateSensor(int pin) {
    // Read PPG signal and apply a moving average filter
    ppgBuffer[bufferIndex] = analogRead(pin);
    bufferIndex = (bufferIndex + 1) % SAMPLES;

    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        sum += ppgBuffer[i];
    }
    int filteredValue = sum / SAMPLES;

    // Peak detection
    if (filteredValue > threshold) {
        if (millis() - lastPeakTime > 300) {  // Ignore peaks that are too close
            peakCount++;
            lastPeakTime = millis();
        }
    }

    // Calculate heart rate (BPM) from peak count
    float bpm = (peakCount / (millis() / 60000.0));

    return bpm;
}

void sendEnvironmentalData() {
    // Format: E,light1,light2,temperature
    String data = "E,";
    data += String(sensorData.light1) + ",";
    data += String(sensorData.light2) + ",";
    data += String(sensorData.temperature, 2);

    bluetooth.println(data);
    Serial.println(data);
}

void sendHeartRateData() {
    // Format: H,heartRate
    String data = "H,";
    data += String(sensorData.heartRate);

    bluetooth.println(data);
    Serial.println(data);
}

void sendAccelData() {
    // Format: A,accelX,accelY,accelZ,motionIntensity
    String data = "A,";
    data += String(sensorData.accelRawX) + ",";
    data += String(sensorData.accelRawY) + ",";
    data += String(sensorData.accelRawZ) + ",";
    data += String(sensorData.motionIntensity, 2);

    bluetooth.println(data);
    Serial.println(data);
}

void sendBatteryData() {
    // Format: B,batteryVoltage,batteryPercent,isCharging
    String data = "B,";
    data += String(sensorData.batteryVoltage, 2) + ",";
    data += String(sensorData.batteryPercent) + ",";
    data += sensorData.isCharging ? "1" : "0";

    bluetooth.println(data);
    Serial.println(data);
}

void checkBattery() {
    // Read the battery voltage
    float voltage = analogRead(BATTERY_PIN) * (3.3 / 1024.0) * VBAT_DIVIDER;
    sensorData.batteryVoltage = voltage;

    // Calculate the battery percentage
    int percent = (int)((voltage - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100);
    percent = constrain(percent, 0, 100);
    sensorData.batteryPercent = percent;

    // Check if the battery is charging
    sensorData.isCharging = (voltage > VBAT_MAX);
}
