#include <SoftwareSerial.h>
// Pin definitions
#define LIGHT_SENSOR_1 A0
#define LIGHT_SENSOR_2 A1
#define TEMP_SENSOR A2
#define HEART_RATE_1 A3
#define HEART_RATE_2 A4
// Bluetooth module pins
#define BT_RX 2
#define BT_TX 3
// Create bluetooth serial connection
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Timing variables
const unsigned long LIGHT_TEMP_INTERVAL = 300000;  // 5 minutes
const unsigned long HEART_RATE_INTERVAL = 120000;  // 2 minutes
unsigned long lastLightTempReading = 0;
unsigned long lastHeartRateReading = 0;

enum SensorState {
    LIGHT_1,
    LIGHT_2,
    TEMPERATURE,
    HEART_RATE_1,
    HEART_RATE_2,
    IDLE
};

SensorState currentState = IDLE;

struct SensorData {
    int light1;
    int light2;
    float temperature;
    int heartRate1;
    int heartRate2;
} sensorData;

void setup() {
    Serial.begin(9600);
    bluetooth.begin(9600);
    pinMode(LIGHT_SENSOR_1, INPUT);
    pinMode(LIGHT_SENSOR_2, INPUT);
    pinMode(TEMP_SENSOR, INPUT);
    pinMode(HEART_RATE_1, INPUT);
    pinMode(HEART_RATE_2, INPUT);
}

void loop() {
    unsigned long currentTime = millis();

    // Check for light and temperature readings
    if (currentTime - lastLightTempReading >= LIGHT_TEMP_INTERVAL) {
        readLightAndTemp();
        lastLightTempReading = currentTime;
    }

    // Check for heart rate readings
    if (currentTime - lastHeartRateReading >= HEART_RATE_INTERVAL) {
        readHeartRate();
        lastHeartRateReading = currentTime;
    }

    processCurrentState();
    delay(100);
}

void processCurrentState() {
    switch (currentState) {
        case LIGHT_1:
            sensorData.light1 = analogRead(LIGHT_SENSOR_1);
            currentState = LIGHT_2;
            break;
            
        case LIGHT_2:
            sensorData.light2 = analogRead(LIGHT_SENSOR_2);
            currentState = TEMPERATURE; // Proceed to temperature
            break;
            
        case TEMPERATURE:
            // Temperature logic is incomplete
            currentState = IDLE;
            break;
            
        case HEART_RATE_1:
            // Heart rate reading logic not fully implemented
            currentState = HEART_RATE_2;
            break;
            
        case HEART_RATE_2:
            // Missing code to read second heart rate
            currentState = IDLE;
            break;
            
        case IDLE:
            // Do nothing
            break;
    }
}

void readLightAndTemp() {
    currentState = LIGHT_1;  // Start the reading process
}

void readHeartRate() {
    currentState = HEART_RATE_1;  // Start heart rate reading
}

// Function to read heart rate sensor
int readHeartRateSensor(int pin) {
    // Placeholder implementation
    return analogRead(pin); // Just read the value without averaging
}

void sendEnvironmentalData() {
    // Incomplete: Format and send environmental data
}

void sendHeartRateData() {
    // Incomplete: Send heart rate data
}
