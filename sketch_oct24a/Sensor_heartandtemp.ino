#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin definitions for Adafruit Feather nRF52840
#define LIGHT_SENSOR_1 A0  // Use A0 (P0.26)
#define LIGHT_SENSOR_2 A1  // Use A1 (P0.27)
#define HEART_RATE_PIN A3  // Use A3 (P0.29)

// DS18B20 temperature sensor pin
#define TEMP_SENSOR_PIN A2 // Use A2 (P0.28)

// Bluetooth module pins (if using SoftSerial)
#define BT_RX 2  // Use GPIO2 (P0.02)
#define BT_TX 3  // Use GPIO3 (P0.03)

// Create bluetooth serial connection
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Create OneWire instance
OneWire oneWire(TEMP_SENSOR_PIN);

// Create DallasTemperature instance
DallasTemperature sensors(&oneWire);

// Timing variables (all in milliseconds)
const unsigned long LIGHT_TEMP_INTERVAL = 300000;  // 5 minutes
const unsigned long HEART_RATE_INTERVAL = 120000;  // 2 minutes

unsigned long lastLightTempReading = 0;
unsigned long lastHeartRateReading = 0;

// Sensor state management
enum SensorState {
    LIGHT_1,
    LIGHT_2,
    TEMPERATURE,
    HEART_RATE,
    IDLE
};

SensorState currentState = IDLE;

// Buffer for sensor readings
struct SensorData {
    int light1;
    int light2;
    float temperature;
    int heartRate;
} sensorData;

const int SAMPLES = 100;
int ppgBuffer[SAMPLES];
int bufferIndex = 0;

// Heart rate detection variables
float threshold = 0.5;  // Threshold for peak detection
int peakCount = 0;
unsigned long lastPeakTime = 0;

void setup() {
    // Initialize serial communications
    Serial.begin(9600);
    bluetooth.begin(9600);

    // Initialize analog pins
    pinMode(LIGHT_SENSOR_1, INPUT);
    pinMode(LIGHT_SENSOR_2, INPUT);
    pinMode(HEART_RATE_PIN, INPUT);

    // Initialize the temperature sensor
    sensors.begin();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check if it's time to read light and temperature sensors
    if (currentTime - lastLightTempReading >= LIGHT_TEMP_INTERVAL) {
        readLightAndTemp();
        lastLightTempReading = currentTime;
    }
    
    // Check if it's time to read heart rate sensor
    if (currentTime - lastHeartRateReading >= HEART_RATE_INTERVAL) {
        readHeartRate();
        lastHeartRateReading = currentTime;
    }
    
    // Process the current state
    processCurrentState();
    
    // Small delay to prevent overwhelming the system
    delay(10);
}

void processCurrentState() {
    switch (currentState) {
        case LIGHT_1:
            sensorData.light1 = analogRead(LIGHT_SENSOR_1);
            currentState = LIGHT_2;
            break;
            
        case LIGHT_2:
            sensorData.light2 = analogRead(LIGHT_SENSOR_2);
            currentState = TEMPERATURE;
            break;
            
        case TEMPERATURE:
            // Get temperature from DS18B20 sensor
            sensors.requestTemperatures();  // Request temperature reading
            sensorData.temperature = sensors.getTempCByIndex(0); // Get temperature in Celsius
            
            // Debugging output
            Serial.print("Temperature (C): ");
            Serial.println(sensorData.temperature);
            
            currentState = IDLE;
            sendEnvironmentalData();
            break;
            
        case HEART_RATE:
            sensorData.heartRate = readHeartRateSensor(HEART_RATE_PIN, ppgBuffer, bufferIndex, threshold, peakCount, lastPeakTime);
            currentState = IDLE;
            sendHeartRateData();
            break;
            
        case IDLE:
            // Do nothing in idle state
            break;
    }
}

void readLightAndTemp() {
    currentState = LIGHT_1;  // Start the environmental reading sequence
}

void readHeartRate() {
    currentState = HEART_RATE;  // Start the heart rate reading sequence
}

int readHeartRateSensor(int pin, int* ppgBuffer, int &bufferIndex, float &threshold, int &peakCount, unsigned long &lastPeakTime) {
    // Read PPG signal and apply a moving average filter
    ppgBuffer[bufferIndex] = analogRead(pin);
    bufferIndex = (bufferIndex + 1) % SAMPLES;
    
    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        sum += ppgBuffer[i];
    }
    int filteredValue = sum / SAMPLES;
    
    // Debugging output
    Serial.print("Raw Heart Rate Reading: ");
    Serial.println(filteredValue);
    
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
    // Format: T,temperature
    String data = "T,";
    data += String(sensorData.temperature, 2);
    
    bluetooth.println(data);
    Serial.println(data);  // For debugging
}

void sendHeartRateData() {
    // Format: H,heartRate
    String data = "H,";
    data += String(sensorData.heartRate);
    
    bluetooth.println(data);
    Serial.println(data);  // For debugging
}

