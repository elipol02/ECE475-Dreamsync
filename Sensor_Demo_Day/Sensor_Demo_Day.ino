#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

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

// BLE services 
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

// DS18B20 OneWire bus
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// Timing intervals (in milliseconds)
const unsigned long SENSOR_UPDATE_INTERVAL = 1000;  // 1 second for all sensor readings
const unsigned long HEART_RATE_MEASUREMENT_TIME = 15000;  // 15 seconds for BPM measurement

unsigned long lastSensorUpdateTime = 0;
unsigned long measurementStartTime = 0;  // track start time for heart rate measurement

// Buffer for heart rate data
const int SAMPLES = 100;
int ppgBuffer[SAMPLES];
int bufferIndex = 0;
int peakCount = 0;
unsigned long lastPeakTime = 0;

// Sensor data array
float sensorData[7] = {0};

bool mpuInitialized = false;

// Bluetooth helper
void startAdv(void)
{
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleuart);
    Bluefruit.ScanResponse.addName();
    
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising
}

// Bluetooth helper
void connect_callback(uint16_t conn_handle)
{
    BLEConnection* connection = Bluefruit.Connection(conn_handle);
    char central_name[32] = { 0 };
    connection->getPeerName(central_name, sizeof(central_name));
    Serial.print("Connected to ");
    Serial.println(central_name);
}

// Bluetooth helper
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    Serial.print("Disconnected, reason = 0x");
    Serial.println(reason, HEX);
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    // while (!Serial) delay(10);

    Serial.println("Sleep Mask BLE Setup");

    // Initialize Bluefruit
    Bluefruit.begin();
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
    Bluefruit.setName("SleepMask");
    
    // Callbacks for Peripheral
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // Configure and Start Services
    bledfu.begin();             // OTA DFU service
    
    bledis.setManufacturer("Your Name");
    bledis.setModel("Sleep Mask v1.0");
    bledis.begin();
    
    bleuart.begin();           // Configure and Start BLE Uart service
    blebas.begin();            // Start BLE Battery service
    
    // Start advertising
    startAdv();

    // Initialize I2C
    Wire.begin();

    // Initialize MPU6050
    mpuInitialized = initializeMPU6050();
    if (!mpuInitialized) {
        Serial.println("Failed to initialize MPU6050!");
    } else {
        Serial.println("MPU6050 initialized successfully");
    }

    // Initialize temperature sensor
    tempSensor.begin();

    // Set pin modes
    pinMode(LIGHT_SENSOR_1, INPUT);
    pinMode(LIGHT_SENSOR_2, INPUT);
    pinMode(HEART_RATE_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);  // set LED pin as output

    Serial.println("Setup complete");
}

// Most of the initializeMPU6050() is just pulled from tutorial and examples I saw online
bool initializeMPU6050() {

    // Reset MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x80);  // Reset device
    Wire.endTransmission(true);
    delay(100);  // Wait for reset to complete
    
    // Check if MPU6050 is responding
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("MPU6050 not found!");
        return false;
    }
    
    // Wake up the MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00);  // Clear sleep mode bit
    Wire.endTransmission(true);
    
    delay(100); // Wait for wake-up
    
    // Configure accelerometer (+/- 2g)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_CONFIG);
    Wire.write(0x00);  // 0x00 = +/- 2g
    Wire.endTransmission(true);
    
    // Configure gyroscope (+/- 250 deg/s)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_CONFIG);
    Wire.write(0x00);  // 0x00 = +/- 250 degrees/second
    Wire.endTransmission(true);

    return true;
}

void loop() {
    unsigned long currentTime = millis();
    static bool lastConnected = false;
    bool currentConnected = Bluefruit.connected();
    
    // Using this to test the connection status
    if (currentConnected != lastConnected) {
        if (currentConnected) {
            Serial.println("BLE Connected!");
        } else {
            Serial.println("BLE Disconnected!");
        }
        lastConnected = currentConnected;
    }

    // Heart rate measurement
    if (currentTime - measurementStartTime >= HEART_RATE_MEASUREMENT_TIME) {
        sensorData[3] = calculateBPM();  // Store heart rate in the array
        peakCount = 0;
        measurementStartTime = currentTime;
    }

    // Continuous heart rate monitoring
    if (currentTime - measurementStartTime < HEART_RATE_MEASUREMENT_TIME) {
        readHeartRateSensor();
    }

    // Update sensor data and send every second
    if (currentTime - lastSensorUpdateTime >= SENSOR_UPDATE_INTERVAL) {
        updateSensorData();
        sendSensorData();
        lastSensorUpdateTime = currentTime;
    }

    delay(10);  // Small delay to prevent overwhelming the system
}

void updateSensorData() {
    // Read light and temperature sensors
    sensorData[0] = analogRead(LIGHT_SENSOR_1);
    sensorData[1] = analogRead(LIGHT_SENSOR_2);
    tempSensor.requestTemperatures();
    sensorData[2] = tempSensor.getTempCByIndex(0);

    // Read MPU6050 data
    if (mpuInitialized) {
        readMPU6050();
    }
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
        int16_t accelX = Wire.read() << 8 | Wire.read();
        int16_t accelY = Wire.read() << 8 | Wire.read();
        int16_t accelZ = Wire.read() << 8 | Wire.read();
        
        int16_t tempRaw = Wire.read() << 8 | Wire.read();
        float temperature = tempRaw / 340.0 + 36.53;
        
        int16_t gyroX = Wire.read() << 8 | Wire.read();
        int16_t gyroY = Wire.read() << 8 | Wire.read();
        int16_t gyroZ = Wire.read() << 8 | Wire.read();
        
        sensorData[4] = gyroX;
        sensorData[5] = gyroY;
        sensorData[6] = gyroZ;

        // Debug output
        
        Serial.print("AccelX: "); Serial.print(accelX);
        Serial.print(" AccelY: "); Serial.print(accelY);
        Serial.print(" AccelZ: "); Serial.print(accelZ);
        Serial.print(" Temp: "); Serial.print(temperature);
        Serial.print(" GyroX: "); Serial.print(gyroX);
        Serial.print(" GyroY: "); Serial.print(gyroY);
        Serial.print(" GyroZ: "); Serial.println(gyroZ);
        
    }
}

float calculateBPM() {
    float bpm = (peakCount * 60.0) / (HEART_RATE_MEASUREMENT_TIME / 1000.0);
    // Serial.print("BPM: ");
    // Serial.println(bpm);
    return bpm;
}

void sendSensorData() {
    // Convert sensor data to a string and send over BLE UART
    char dataStr[256];
    snprintf(dataStr, sizeof(dataStr), "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", // The order of this is  "Light1, Light2, Temp, BPM, GyroX, GyroY, GyroZ"
        sensorData[0], sensorData[1], sensorData[2], sensorData[3], sensorData[4], sensorData[5], sensorData[6]);
    
    if (Bluefruit.connected()) {
        bleuart.print(dataStr);
        bleuart.print("\r\n");
    }

    Serial.println(dataStr);
}