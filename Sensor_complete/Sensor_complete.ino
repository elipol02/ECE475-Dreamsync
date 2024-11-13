#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define LIGHT_SENSOR_1 A0
#define LIGHT_SENSOR_2 A1
#define TEMP_SENSOR_PIN A2    // DS18B20 connected to this pin
#define HEART_RATE_PIN A3
#define LED_PIN 3  // pin for LED

// MPU6050 Constants
#define MPU6050_ADDR         0x68   // default I2C address 
#define MPU6050_PWR_MGMT_1   0x6B   // Power Management 1 register
#define MPU6050_GYRO_CONFIG  0x1B   // Gyroscope Configuration register
#define MPU6050_ACCEL_CONFIG 0x1C   // Accelerometer Configuration register
#define MPU6050_ACCEL_XOUT_H 0x3B   // first data register 

// bluetooth setup using hardware UART for nRF52840
#define BLE_SERIAL Serial1

// DS18B20 OneWire bus
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// Timing intervals (in milliseconds)
const unsigned long LIGHT_TEMP_INTERVAL = 60000;  // 1 minute for light and temperature reading
const unsigned long HEART_RATE_INTERVAL = 100;   // 100 milliseconds for heart rate checking
const unsigned long HEART_RATE_MEASUREMENT_TIME = 15000;  // 15 seconds for BPM measurement
const unsigned long ENVIRONMENTAL_INTERVAL = 60000;  // 1 minute for environmental data output
const unsigned long MPU6050_INTERVAL = 100;  // 100ms (10Hz)
const unsigned long BATTERY_INTERVAL = 1000;  // 1 second

unsigned long lastLightTempReading = 0;
unsigned long lastHeartRateReading = 0;
unsigned long lastEnvironmentalReading = 0;  // track the last environmental data reading
unsigned long measurementStartTime = 0;  // track start time for heart rate measurement

// i just followed suit with MPU and batter
unsigned long lastMPUReading = 0;       
unsigned long lastBatteryReading = 0;

// Buffer for heart rate data
const int SAMPLES = 100;
int ppgBuffer[SAMPLES];
int bufferIndex = 0;
int peakCount = 0;
unsigned long lastPeakTime = 0;

// sensor data struct that contains our MPU6050 data
struct MPU6050Data {
    // accel raw values (16-bit)
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    
    // gyro raw values (16-bit)
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    
    // temperature
    // this was in teh example code from docs but not sure if we need
    float temperature;
    
    // calculated values
    float pitch;
    float roll;
    float motionIntensity;
} mpuData;

struct SensorData {
    int light1;
    int light2;
    float temperature;
    int heartRate;
    MPU6050Data mpu;
    float batteryVoltage;
    int batteryPercent;
    bool isCharging;
} sensorData;

bool mpuInitialized = false;

void setup() {
    // intialize serial communication
    Serial.begin(115200);
    BLE_SERIAL.begin(9600);  // Initialize Bluetooth

    // initiailize I2C
    Wire.begin();


    // initialize MPU6050
    mpuInitialized = initializeMPU6050();
    if (!mpuInitialized) {
        Serial.println("Failed to initialize MPU6050!");
    } else {
        Serial.println("MPU6050 initialized successfully");
    }

    // initialize temperature sensor
    tempSensor.begin();

    // Set pin modes
    pinMode(LIGHT_SENSOR_1, INPUT);
    pinMode(LIGHT_SENSOR_2, INPUT);
    pinMode(HEART_RATE_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);  // set LED pin as output

    Serial.println("Setup complete");
}

// Most of the initializeMPU6050() is just pulled from tutorial and examples i saw online
bool initializeMPU6050() {
    // check if MPU6050 is responding
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("MPU6050 not found!");
        return false;
    }
    
    // wake up the MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00);  // Clear sleep mode bit
    Wire.endTransmission(true);
    
    delay(100); // Wait for wake-up
    
    // configure accelerometer (+/- 2g)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_CONFIG);
    Wire.write(0x00);  // 0x00 = +/- 2g
    Wire.endTransmission(true);
    
    // configuring gyroscope (+/- 250 deg/s)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_CONFIG);
    Wire.write(0x00);  // 0x00 = +/- 250 degrees/second
    Wire.endTransmission(true);

    return true;
}

void loop() {
    unsigned long currentTime = millis();

    // check light and temperature
    if (currentTime - lastLightTempReading >= LIGHT_TEMP_INTERVAL) {
        readLightAndTemp();
        lastLightTempReading = currentTime;
    }

    // heart rate measurement
    if (currentTime - measurementStartTime >= HEART_RATE_MEASUREMENT_TIME) {
        sensorData.heartRate = calculateBPM();
        sendHeartRateData();
        peakCount = 0;
        measurementStartTime = currentTime;
    }

    // continous heart rate monitoring
    if (currentTime - measurementStartTime < HEART_RATE_MEASUREMENT_TIME) {
        readHeartRateSensor();
    }

    // MPU6050 reading
    if (currentTime - lastMPUReading >= MPU6050_INTERVAL && mpuInitialized) {
        readMPU6050();
        sendMotionData();
        lastMPUReading = currentTime;
    }

    // battery check
    if (currentTime - lastBatteryReading >= BATTERY_INTERVAL) {
        checkBattery();
        sendBatteryData();
        lastBatteryReading = currentTime;
    }

    // environmental data
    if (currentTime - lastEnvironmentalReading >= ENVIRONMENTAL_INTERVAL) {
        sendEnvironmentalData();
        lastEnvironmentalReading = currentTime;
    }

    delay(10);  // small delay to prevent overwhelming the system
}

void readLightAndTemp() {
    sensorData.light1 = analogRead(LIGHT_SENSOR_1);
    sensorData.light2 = analogRead(LIGHT_SENSOR_2);
    tempSensor.requestTemperatures();
    sensorData.temperature = tempSensor.getTempCByIndex(0);
}

void readHeartRateSensor() {
    int rawValue = analogRead(HEART_RATE_PIN);
    ppgBuffer[bufferIndex] = rawValue;
    bufferIndex = (bufferIndex + 1) % SAMPLES;

    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        sum += ppgBuffer[i];
    }
    int filteredValue = sum / SAMPLES;

    int threshold = 730;
    if (rawValue > threshold && (millis() - lastPeakTime) > 300) {
        peakCount++;
        lastPeakTime = millis();
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
    }
}

void readMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    if (Wire.available() >= 14) {
        sensorData.mpu.accelX = Wire.read() << 8 | Wire.read();
        sensorData.mpu.accelY = Wire.read() << 8 | Wire.read();
        sensorData.mpu.accelZ = Wire.read() << 8 | Wire.read();
        
        int16_t tempRaw = Wire.read() << 8 | Wire.read();
        sensorData.mpu.temperature = tempRaw / 340.0 + 36.53;
        
        sensorData.mpu.gyroX = Wire.read() << 8 | Wire.read();
        sensorData.mpu.gyroY = Wire.read() << 8 | Wire.read();
        sensorData.mpu.gyroZ = Wire.read() << 8 | Wire.read();
        
        calculateAngles();
        
        sensorData.mpu.motionIntensity = sqrt(
            pow(sensorData.mpu.accelX / 16384.0, 2) +
            pow(sensorData.mpu.accelY / 16384.0, 2) +
            pow(sensorData.mpu.accelZ / 16384.0, 2)
        );
    }
}

void calculateAngles() {
    float x = sensorData.mpu.accelX / 16384.0;
    float y = sensorData.mpu.accelY / 16384.0;
    float z = sensorData.mpu.accelZ / 16384.0;
    
    sensorData.mpu.pitch = atan2(x, sqrt(y * y + z * z));
    sensorData.mpu.roll = atan2(y, sqrt(x * x + z * z));
    
    sensorData.mpu.pitch *= 180.0 / M_PI;
    sensorData.mpu.roll *= 180.0 / M_PI;
}

int calculateBPM() {
    int bpm = peakCount * 4;
    Serial.print("Heartbeats in 15s: ");
    Serial.println(peakCount);
    Serial.print("Calculated BPM: ");
    Serial.println(bpm);
    return bpm;
}

// TODO: Still need to verify this implementation of the battery
void checkBattery() {
    float measuredvbat = analogRead(A7);
    measuredvbat *= 2;    // voltage divider
    measuredvbat *= 3.3;  // referenc voltage
    measuredvbat /= 1024; // 10-bit ADC resolution
    
    sensorData.batteryVoltage = measuredvbat;
    
    float batteryRange = 4.25 - 2.75;
    float voltageOffset = measuredvbat - 2.75;
    sensorData.batteryPercent = (voltageOffset / batteryRange) * 100.0;
    sensorData.batteryPercent = constrain(sensorData.batteryPercent, 0, 100);
    
    sensorData.isCharging = (measuredvbat > 4.25);
}

void sendEnvironmentalData() {
    String data = "E," + String(sensorData.light1) + "," + 
                 String(sensorData.light2) + "," + 
                 String(sensorData.temperature, 2);
    BLE_SERIAL.println(data);
    Serial.println(data);
}

void sendHeartRateData() {
    String data = "H," + String(sensorData.heartRate);
    BLE_SERIAL.println(data);
    Serial.println(data);
}

void sendMotionData() {
    String data = "M," + 
                 String(sensorData.mpu.accelX) + "," +
                 String(sensorData.mpu.accelY) + "," +
                 String(sensorData.mpu.accelZ) + "," +
                 String(sensorData.mpu.gyroX) + "," +
                 String(sensorData.mpu.gyroY) + "," +
                 String(sensorData.mpu.gyroZ) + "," +
                 String(sensorData.mpu.pitch, 2) + "," +
                 String(sensorData.mpu.roll, 2) + "," +
                 String(sensorData.mpu.motionIntensity, 2);
    
    BLE_SERIAL.println(data);
    Serial.println(data);
}

// TODO: Still need to verify this implementation of the battery
void sendBatteryData() {
    String data = "B," + 
                 String(sensorData.batteryVoltage, 2) + "," +
                 String(sensorData.batteryPercent) + "," +
                 String(sensorData.isCharging ? 1 : 0);
    
    BLE_SERIAL.println(data);
    Serial.println(data);
}