#include <SoftwareSerial.h>

// Pin definitions for Adafruit Feather nRF52840
#define LIGHT_SENSOR_PIN A0  // Use A0 (P0.26)
#define TEMP_SENSOR A2       // Use A2 (P0.28)
#define HEART_RATE_PIN A3    // Use A3 (P0.29)

// Control pins for the analog switches (assuming using CD4051)
#define LIGHT_SENSOR_SEL 4  // Control pin for light sensor selection
#define HEART_RATE_SEL 5    // Control pin for heart rate sensor selection

// Bluetooth module pins (if using SoftSerial)
#define BT_RX 2  // Use GPIO2 (P0.02)
#define BT_TX 3  // Use GPIO3 (P0.03)

// Create bluetooth serial connection
SoftwareSerial bluetooth(BT_RX, BT_TX);

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
    HEART_RATE_1,
    HEART_RATE_2,
    IDLE
};

SensorState currentState = IDLE;

// Buffer for sensor readings
struct SensorData {
    int light1;
    int light2;
    float temperature;
    int heartRate1;
    int heartRate2;
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
    pinMode(LIGHT_SENSOR_PIN, INPUT);
    pinMode(TEMP_SENSOR, INPUT);
    pinMode(HEART_RATE_PIN, INPUT);
    
    // Initialize control pins
    pinMode(LIGHT_SENSOR_SEL, OUTPUT);
    pinMode(HEART_RATE_SEL, OUTPUT);
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check if it's time to read light and temperature sensors
    if (currentTime - lastLightTempReading >= LIGHT_TEMP_INTERVAL) {
        readLightAndTemp();
        lastLightTempReading = currentTime;
    }
    
    // Check if it's time to read heart rate sensors
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
            digitalWrite(LIGHT_SENSOR_SEL, LOW);  // Select first light sensor
            sensorData.light1 = analogRead(LIGHT_SENSOR_PIN);
            currentState = LIGHT_2;
            break;
            
        case LIGHT_2:
            digitalWrite(LIGHT_SENSOR_SEL, HIGH);  // Select second light sensor
            sensorData.light2 = analogRead(LIGHT_SENSOR_PIN);
            currentState = TEMPERATURE;
            break;
            
        case TEMPERATURE:
            // Convert analog reading to temperature in Celsius
            int tempReading = analogRead(TEMP_SENSOR);
            float voltage = tempReading * (5.0 / 1024.0);
            sensorData.temperature = (voltage - 0.5) * 100;
            currentState = IDLE;
            sendEnvironmentalData();
            break;
            
        case HEART_RATE_1:
            digitalWrite(HEART_RATE_SEL, LOW);  // Select first heart rate sensor
            sensorData.heartRate1 = readHeartRateSensor(HEART_RATE_PIN);
            currentState = HEART_RATE_2;
            break;
            
        case HEART_RATE_2:
            digitalWrite(HEART_RATE_SEL, HIGH);  // Select second heart rate sensor
            sensorData.heartRate2 = readHeartRateSensor(HEART_RATE_PIN);
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
    currentState = HEART_RATE_1;  // Start the heart rate reading sequence
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
    Serial.println(data);  // For debugging
}

void sendHeartRateData() {
    // Format: H,heartRate1,heartRate2
    String data = "H,";
    data += String(sensorData.heartRate1) + ",";
    data += String(sensorData.heartRate2);
    
    bluetooth.println(data);
    Serial.println(data);  // For debugging
}
