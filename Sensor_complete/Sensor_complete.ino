#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define LIGHT_SENSOR_1 A0
#define LIGHT_SENSOR_2 A1
#define TEMP_SENSOR_PIN A2    // DS18B20 connected to this pin
#define HEART_RATE_PIN A3
#define LED13 13              // Onboard LED for heartbeat indication

// Bluetooth setup (using hardware UART for Adafruit Feather nRF52840)
#define BLE_SERIAL Serial1

// DS18B20 OneWire bus
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// Timing intervals (in milliseconds)
const unsigned long LIGHT_TEMP_INTERVAL = 3000;  // 3 seconds
const unsigned long HEART_RATE_INTERVAL = 1200;  // 1.2 seconds
unsigned long lastLightTempReading = 0;
unsigned long lastHeartRateReading = 0;

// Buffer for heart rate data
const int SAMPLES = 100;
int ppgBuffer[SAMPLES];
int bufferIndex = 0;
int peakCount = 0;
unsigned long lastPeakTime = 0;
unsigned long lastBPMCalculationTime = 0;
unsigned long peakIntervals[10];  // Array to store intervals between peaks
int peakIntervalIndex = 0;

struct SensorData {
    int light1;
    int light2;
    float temperature;
    int heartRate;
    int rawHeartRate;
} sensorData;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    BLE_SERIAL.begin(9600);  // Initialize Bluetooth

    // Initialize temperature sensor
    tempSensor.begin();

    // Set pin modes
    pinMode(LIGHT_SENSOR_1, INPUT);
    pinMode(LIGHT_SENSOR_2, INPUT);
    pinMode(HEART_RATE_PIN, INPUT);
    pinMode(LED13, OUTPUT);  // Set the onboard LED pin as output

    Serial.println("Setup complete");
}

void loop() {
    unsigned long currentTime = millis();

    // Check if it's time to read light and temperature sensors
    if (currentTime - lastLightTempReading >= LIGHT_TEMP_INTERVAL) {
        readLightAndTemp();
        lastLightTempReading = currentTime;
    }

    // Check if it's time to read heart rate
    if (currentTime - lastHeartRateReading >= HEART_RATE_INTERVAL) {
        sensorData.heartRate = readHeartRateSensor();
        sendHeartRateData();
        lastHeartRateReading = currentTime;
    }

    delay(10);  // Small delay to avoid overwhelming the system
}

void readLightAndTemp() {
    // Read light sensors
    sensorData.light1 = analogRead(LIGHT_SENSOR_1);
    sensorData.light2 = analogRead(LIGHT_SENSOR_2);

    // Read temperature from DS18B20
    tempSensor.requestTemperatures();  // Request temperature data
    sensorData.temperature = tempSensor.getTempCByIndex(0);  // Get temperature in Celsius

    // Debugging output in desired format
    Serial.print("Light 1: ");
    Serial.print(sensorData.light1);
    Serial.print(", Light 2: ");
    Serial.print(sensorData.light2);
    Serial.print(", Temperature: ");
    Serial.println(sensorData.temperature, 2);  // Limit to 2 decimal places

    sendEnvironmentalData();
}

int readHeartRateSensor() {
    // Read pulse sensor data into buffer and calculate heart rate
    int rawValue = analogRead(HEART_RATE_PIN);
    ppgBuffer[bufferIndex] = rawValue;
    bufferIndex = (bufferIndex + 1) % SAMPLES;

    // Calculate moving average
    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        sum += ppgBuffer[i];
    }
    int filteredValue = sum / SAMPLES;

    // Store the raw value
    sensorData.rawHeartRate = rawValue;

    // Debugging output for raw and filtered values
    Serial.print("Raw heart rate value: ");
    Serial.println(rawValue);
    Serial.print("Filtered heart rate value: ");
    Serial.println(filteredValue);

    // Peak detection logic
    int threshold = 80;  // Adjust threshold as needed based on signal amplitude
    int debounceTime = 300;  // Minimum time between peaks to avoid counting noise as peaks
    unsigned long currentTime = millis();
    if (filteredValue > threshold && (currentTime - lastPeakTime) > debounceTime) {
        peakCount++;
        unsigned long peakInterval = currentTime - lastPeakTime;
        peakIntervals[peakIntervalIndex] = peakInterval;
        peakIntervalIndex = (peakIntervalIndex + 1) % 10;  // Keep a rolling buffer of the last 10 intervals
        lastPeakTime = currentTime;
        digitalWrite(LED13, HIGH);  // Blink onboard LED when heartbeat is detected
    } else {
        digitalWrite(LED13, LOW);   // Turn off LED if no heartbeat detected
    }

    // Calculate BPM from peak intervals
    int intervalSum = 0;
    for (int i = 0; i < 10; i++) {
        intervalSum += peakIntervals[i];
    }
    int bpm = 0;
    if (intervalSum > 0) {
        float averageInterval = intervalSum / 10.0;
        bpm = 60000 / averageInterval;
    }

    // Debugging output for BPM
    Serial.print("Calculated BPM: ");
    Serial.println(bpm);

    return bpm;
}

void sendEnvironmentalData() {
    // Format environmental data: E,light1,light2,temperature
    String data = "E," + String(sensorData.light1) + "," + String(sensorData.light2) + "," + String(sensorData.temperature, 2);
    BLE_SERIAL.println(data);
    Serial.println(data);  // For debugging
}

void sendHeartRateData() {
    // Format heart rate data: H,rawHeartRate,BPM
    String data = "H," + String(sensorData.rawHeartRate) + "," + String(sensorData.heartRate);
    BLE_SERIAL.println(data);
    Serial.println(data);  // For debugging
}

