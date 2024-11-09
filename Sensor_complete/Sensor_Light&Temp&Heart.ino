#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define LIGHT_SENSOR_1 A0
#define LIGHT_SENSOR_2 A1
#define TEMP_SENSOR_PIN A2    // DS18B20 connected to this pin
#define HEART_RATE_PIN A3
#define LED_PIN 3  // Pin for LED

// Bluetooth setup (using hardware UART for Adafruit Feather nRF52840)
#define BLE_SERIAL Serial1

// DS18B20 OneWire bus
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// Timing intervals (in milliseconds)
const unsigned long LIGHT_TEMP_INTERVAL = 60000;  // 1 minute for light and temperature reading
const unsigned long HEART_RATE_INTERVAL = 100;   // 100 milliseconds for heart rate checking
const unsigned long HEART_RATE_MEASUREMENT_TIME = 15000;  // 15 seconds for BPM measurement
const unsigned long ENVIRONMENTAL_INTERVAL = 60000;  // 1 minute for environmental data output

unsigned long lastLightTempReading = 0;
unsigned long lastHeartRateReading = 0;
unsigned long lastEnvironmentalReading = 0;  // Track the last environmental data reading
unsigned long measurementStartTime = 0;  // Track start time for heart rate measurement

// Buffer for heart rate data
const int SAMPLES = 100;
int ppgBuffer[SAMPLES];
int bufferIndex = 0;
int peakCount = 0;
unsigned long lastPeakTime = 0;

struct SensorData {
    int light1;
    int light2;
    float temperature;
    int heartRate;
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
    pinMode(LED_PIN, OUTPUT);  // Set LED pin as output

    Serial.println("Setup complete");
}

void loop() {
    unsigned long currentTime = millis();

    // Check if it's time to read light and temperature sensors (every 1 minute)
    if (currentTime - lastLightTempReading >= LIGHT_TEMP_INTERVAL) {
        readLightAndTemp();
        lastLightTempReading = currentTime;
    }

    // Start the heart rate measurement every 15 seconds
    if (currentTime - measurementStartTime >= HEART_RATE_MEASUREMENT_TIME) {
        // Calculate BPM after the 15 seconds
        sensorData.heartRate = calculateBPM();
        sendHeartRateData();

        // Reset the measurement for the next 15 seconds
        peakCount = 0;  // Reset the peak count for the next measurement window
        measurementStartTime = currentTime;  // Set the new start time for the next measurement
    }

    // Continuously check for heart rate data during the 15-second measurement window
    if (currentTime - measurementStartTime < HEART_RATE_MEASUREMENT_TIME) {
        readHeartRateSensor();  // Keep reading heart rate sensor during the 15 seconds
    }

    // Check if it's time to output environmental data every minute
    if (currentTime - lastEnvironmentalReading >= ENVIRONMENTAL_INTERVAL) {
        sendEnvironmentalData();
        lastEnvironmentalReading = currentTime;  // Update the time of last environmental data output
    }

    delay(HEART_RATE_INTERVAL);  // Small delay to allow time for heart rate readings
}

void readLightAndTemp() {
    // Read light sensors
    sensorData.light1 = analogRead(LIGHT_SENSOR_1);
    sensorData.light2 = analogRead(LIGHT_SENSOR_2);

    // Read temperature from DS18B20
    tempSensor.requestTemperatures();  // Request temperature data
    sensorData.temperature = tempSensor.getTempCByIndex(0);  // Get temperature in Celsius
}

void readHeartRateSensor() {
    // Read pulse sensor data into buffer and calculate heart rate
    int rawValue = analogRead(HEART_RATE_PIN);
    ppgBuffer[bufferIndex] = rawValue;
    bufferIndex = (bufferIndex + 1) % SAMPLES;

    // Calculate moving average (filtering)
    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        sum += ppgBuffer[i];
    }
    int filteredValue = sum / SAMPLES;

    // Debugging output for raw and filtered values (commented out)
    /*
    Serial.print("Raw heart rate value: ");
    Serial.println(rawValue);
    Serial.print("Filtered heart rate value: ");
    Serial.println(filteredValue);
    */

    // Peak detection logic based on raw value exceeding threshold
    int threshold = 730;  // Only count heartbeat when raw value exceeds 730
    if (rawValue > threshold && (millis() - lastPeakTime) > 300) {  // Count heartbeat if rawValue exceeds 730
        peakCount++;
        lastPeakTime = millis();
        digitalWrite(LED_PIN, HIGH);  // Turn LED on when heartbeat detected
        delay(100);  // Keep LED on for 100ms
        digitalWrite(LED_PIN, LOW);   // Turn LED off
    }
}

int calculateBPM() {
    // Calculate BPM based on peak count multiplied by 4 (for a 15-second measurement period)
    int bpm = peakCount * 4;

    // Display only the number of heartbeats and the calculated BPM
    Serial.print("Heartbeats in 15s: ");
    Serial.println(peakCount);
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
    // Format heart rate data: H,heartRate
    String data = "H," + String(sensorData.heartRate);
    BLE_SERIAL.println(data);
    Serial.println(data);  // For debugging
}
