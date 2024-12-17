#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <ArduinoJson.h>
#include <SPI.h>

// pins
#define LIGHT_SENSOR_1 A0
#define LIGHT_SENSOR_2 A1
#define TEMP_SENSOR_PIN A2    // DS18B20 connected to this pin
#define HEART_RATE_PIN A3
#define LED_PIN 3  // pin for LED
#define ADC_CS_PIN 4 // pin for spi chip select

// MPU6050 Constants
#define MPU6050_ADDR         0x69   // default I2C address 
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
const unsigned long SENSOR_UPDATE_INTERVAL = 500;  // 2 HZ for all sensor readings
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
float sensorData[14] = {0};

bool mpuInitialized = false;
bool CS5523initialized = false;
bool powerCycleDetected = false;

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
    while (!Serial) delay(10);

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

    CS5523initialized = initializeCS5523();

    if (!CS5523initialized) {
        Serial.println("Failed to initialize CS5523!");
    } else {
        Serial.println("CS5523 initialized successfully");
    }

    Serial.println("Setup complete");
}

bool initializeCS5523() {
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);  // Deselect ADC initially.
    SPI.begin();  // Initialize SPI
    Serial.println("SPI initialized!");

    // Step 1: Wait for oscillator to stabilize
    delay(500);

    // Step 2: Send port initialization sequence
    digitalWrite(ADC_CS_PIN, LOW);  // Select the ADC
    delayMicroseconds(10);          // Small setup delay
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    for (int i = 0; i < 15; i++) {
        SPI.transfer(0xFF); // Send 15 bytes of all 1s
    }
    SPI.transfer(0xF6); // Send the final initialization byte

    SPI.endTransaction();
    digitalWrite(ADC_CS_PIN, HIGH); // Deselect ADC
    Serial.println("Port initialization sequence sent!");

    // Step 3: Perform a system reset
    performSystemReset();

    // Step 4: Verify register defaults
    bool isReset = checkConfigRegister();

    if (!isReset) {
      return false;
    }

    configureChannels();

    return true;
}

void performSystemReset() {
    // Step 3a: Write RS=1 to reset
    Serial.println("System reset requested (RS=1).");
    sendCommand(0x03, 0x00, 0x00, 0x80); // RS = 1

    uint32_t config = 0;
    while (config != 0x0000c0) {
        Serial.print("config is: ");
        Serial.println(config, HEX);
        config = readRegister(0x0B);
        delay(10);
    }

    sendCommand(0x03, 0x00, 0x00, 0x00); // RS = 0

    Serial.println("Exited reset mode (RS=0).");
}

// resets register command with 3 bytes
void sendCommand(uint8_t command, uint8_t byte1, uint8_t byte2, uint8_t byte3) {
    digitalWrite(ADC_CS_PIN, LOW); // Select ADC
    delayMicroseconds(10); 
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

    SPI.transfer(command); // Send the command
    SPI.transfer(byte1);
    SPI.transfer(byte2);
    SPI.transfer(byte3);

    SPI.endTransaction();
    digitalWrite(ADC_CS_PIN, HIGH); // Deselect ADC
}

// reads a 24 bit register
uint32_t readRegister(uint8_t command) {
    uint32_t registerValue = 0;

    digitalWrite(ADC_CS_PIN, LOW); // Select ADC
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    SPI.transfer(command); // Send the read command
    delayMicroseconds(10); // Allow ADC to respond

    registerValue |= ((uint32_t)SPI.transfer(0x00) << 16); // MSB
    registerValue |= ((uint32_t)SPI.transfer(0x00) << 8);  // Middle byte
    registerValue |= (uint32_t)SPI.transfer(0x00);         // LSB

    SPI.endTransaction();
    digitalWrite(ADC_CS_PIN, HIGH); // Deselect ADC

    return registerValue;
}

// checks that the ADC is configured properly
bool checkConfigRegister() {

    // Check Configuration Register
    uint32_t configValue = readRegister(0x0B);
    if (configValue != 0x000040) {
        Serial.print("Configuration Register incorrect! Expected: 0x000040, Got: 0x");
        Serial.println(configValue, HEX);
        return false;
    }
    
    Serial.print("ADC config register is correctly set to the default value of ");
    Serial.println(configValue, HEX);

    return true;
}

void configureChannels() {
    // all setups: 15.0 Sps (default), Gain = 1V, Unipolar mode

    digitalWrite(ADC_CS_PIN, LOW); // Select ADC
    delayMicroseconds(10); 
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

    // set depth pointer bits to 0100 (all four CSR channels)
    SPI.transfer(0x03); // write to config command
    SPI.transfer(0x00);
    SPI.transfer(0x70);
    SPI.transfer(0x00);

    uint32_t configValue = readRegister(0x0B);
    Serial.print("config register: ");
    Serial.println(configValue, HEX);

    readSetups();

    // send setup channel write command
    SPI.transfer(0x05);

    // channel 1 setup 1
    SPI.transfer(0x00);
    SPI.transfer(0x70);
    SPI.transfer(0x00);
    // channel 2 setup 3
    SPI.transfer(0x08);
    SPI.transfer(0x70);
    SPI.transfer(0x00);
    // channel 3 setup 5
    SPI.transfer(0x10);
    SPI.transfer(0x70);
    SPI.transfer(0x00);
    // channel 4 setup 7
    SPI.transfer(0x18);
    SPI.transfer(0x70);
    SPI.transfer(0x00);

    readSetups();

    SPI.endTransaction();
    digitalWrite(ADC_CS_PIN, HIGH); // Deselect ADC

    Serial.println("Channels configured for Setups 1,3,5,7.");
}

void readSetups() {
  // Begin SPI transfer
    SPI.transfer(0x0D); // Send command to read the register

    // Variables to hold the 24-bit segments
    uint32_t segment1 = 0; // First 24 bits
    uint32_t segment2 = 0; // Second 24 bits
    uint32_t segment3 = 0; // Third 24 bits
    uint32_t segment4 = 0; // Fourth 24 bits

    // Read and process each 24-bit segment
    segment1 = ((uint32_t)SPI.transfer(0x00) << 16) | 
              ((uint32_t)SPI.transfer(0x00) << 8) | 
              (uint32_t)SPI.transfer(0x00);

    segment2 = ((uint32_t)SPI.transfer(0x00) << 16) | 
              ((uint32_t)SPI.transfer(0x00) << 8) | 
              (uint32_t)SPI.transfer(0x00);

    segment3 = ((uint32_t)SPI.transfer(0x00) << 16) | 
              ((uint32_t)SPI.transfer(0x00) << 8) | 
              (uint32_t)SPI.transfer(0x00);

    segment4 = ((uint32_t)SPI.transfer(0x00) << 16) | 
              ((uint32_t)SPI.transfer(0x00) << 8) | 
              (uint32_t)SPI.transfer(0x00);

    // Debug print
    Serial.print("Segment 1 (24-bit): 0x");
    Serial.println(segment1, HEX);

    Serial.print("Segment 2 (24-bit): 0x");
    Serial.println(segment2, HEX);

    Serial.print("Segment 3 (24-bit): 0x");
    Serial.println(segment3, HEX);

    Serial.print("Segment 4 (24-bit): 0x");
    Serial.println(segment4, HEX);
}

// Most of the initializeMPU6050() is just pulled from tutorial and examples I saw online
bool initializeMPU6050() {

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
    if ((currentTime - lastSensorUpdateTime >= SENSOR_UPDATE_INTERVAL)) {
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

    // Read CS5523 data
    if (CS5523initialized) {
      readADC();
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
        sensorData[7] = accelX;
        sensorData[8] = accelY;
        sensorData[9] = accelZ;

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

void readADC() {
    for (uint8_t channel = 0; channel < 4; channel++) {
        // Select the channel and initiate conversion
        uint8_t setup = channel * 2;
        uint8_t command = 0x80 | (setup << 4); // 0x80 initiates conversion for the specified channel
        digitalWrite(ADC_CS_PIN, LOW);          // Select ADC
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        SPI.transfer(command);                  // Send conversion command
        SPI.endTransaction();
        digitalWrite(ADC_CS_PIN, HIGH);         // Deselect ADC

        // Wait for conversion to complete
        delay(67);

        // Read the 16-bit conversion result
        uint16_t adcResult = readData();

        // Print the result
        Serial.print("Channel ");
        Serial.print(channel + 1);
        Serial.print(": 0x");
        Serial.println(adcResult, HEX);
    }
}

uint16_t readData() {
    uint16_t data = 0;

    digitalWrite(ADC_CS_PIN, LOW); // Select ADC
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

    SPI.transfer(0x00); // clear 8 bits of SDO flag

    // Read the 16-bit data (MSB first)
    data |= ((uint16_t)SPI.transfer(0x00) << 8); // Read MSB
    data |= (uint16_t)SPI.transfer(0x00);        // Read LSB

    SPI.transfer(0x00); // clear the remaining 8 bits

    SPI.endTransaction();
    digitalWrite(ADC_CS_PIN, HIGH); // Deselect ADC

    return data;
}

void sendSensorData() {
    //if (!Bluefruit.connected()) {
    //    Serial.println("BLE UART not connected...");
    //    return;
    //}


    // Create JSON document for each sensor and send it individually
    for (int i = 0; i < 10; i++) {
        StaticJsonDocument<64> jsonDoc; // Small size since we're sending one sensor at a time
        String key;

        // Assign a key based on the sensor index
        switch (i) {
            case 0: key = "Light1"; break;
            case 1: key = "Light2"; break;
            case 2: key = "Temp"; break;
            case 3: key = "BPM"; break;
            case 4: key = "GyroX"; break;
            case 5: key = "GyroY"; break;
            case 6: key = "GyroZ"; break;
            case 7: key = "AccelX"; break;
            case 8: key = "AccelY"; break;
            case 9: key = "AccelZ"; break;
            default: key = "Unknown"; break;
        }

        // Populate JSON document
        jsonDoc[key] = sensorData[i];

        // Serialize JSON
        char jsonString[64]; // Buffer for a single sensor JSON
        serializeJson(jsonDoc, jsonString, sizeof(jsonString));

        // Send JSON string over BLE
        bleuart.println(jsonString);
        Serial.println(jsonString); // Debug output
        delay(50); // Small delay to ensure data is transmitted properly
    }
} 
