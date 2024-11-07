#include <SoftwareSerial.h>

// Pin definitions for Adafruit Feather nRF52840
#define TEMP_SENSOR A2     // Use A2 (P0.28)
#define HEART_RATE_PIN A3  // Use A3 (P0.29)

// Bluetooth module pins (if using SoftSerial)
#define BT_RX 2  // Use GPIO2 (P0.02)
#define BT_TX 3  // Use GPIO3 (P0.03)

// Create bluetooth serial connection
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Timing variables (all in milliseconds)
const unsigned long TEMP_INTERVAL = 300000;  // 5 minutes
const unsigned long HEART_RATE_INTERVAL = 120000;  // 2 minutes

unsigned long lastTempReading = 0;
unsigned long lastHeartRateReading = 0;

// Sensor state management
enum SensorState {
    TEMPERATURE,
    HEART_RATE,
    IDLE
};

SensorState currentState = IDLE;

// Buffer for sensor readings
struct SensorData {
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
    pinMode(TEMP_SENSOR, INPUT);
    pinMode(HEART_RATE_PIN, INPUT);
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check if it's time to read temperature sensor
    if (currentTime - lastTempReading >= TEMP_INTERVAL) {
        readTemperature();
        lastTempReading = currentTime;
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
        case TEMPERATURE:
            // Convert analog reading to temperature in Celsius
            {
                int tempReading = analogRead(TEMP_SENSOR);
                float voltage = tempReading * (3.3 / 1024.0);
                sensorData.temperature = (voltage - 0.5) * 100;
            }
            currentState = IDLE;
            sendTemperatureData();
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

void readTemperature() {
    currentState = TEMPERATURE;  // Start the temperature reading sequence
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

void sendTemperatureData() {
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
