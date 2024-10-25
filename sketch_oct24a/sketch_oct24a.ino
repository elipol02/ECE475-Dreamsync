// Define the number of tasks
#define NUM_TASKS 6

// Include necessary libraries
// #include <YourSensorLibraries.h> // Uncomment and include your specific sensor libraries

// Define task function prototypes
void measureLight();
void measureTemperature();
void measureHeartRate();
void measureAccelerometer();
void sendData();
void checkBattery();

// Define a Task structure
struct Task {
  void (*taskFunction)();      // Pointer to the task function
  unsigned long interval;      // Interval between executions (in milliseconds)
  unsigned long lastRun;       // Timestamp of the last execution
};

// Initialize the tasks with their respective intervals
Task tasks[NUM_TASKS] = {
  {measureLight,        300000, 0},   // Task 1: Measure Light every 5 minutes
  {measureTemperature,  300000, 0},   // Task 2: Measure Temperature every 5 minutes
  {measureHeartRate,    120000, 0},   // Task 3: Measure Heart Rate every 2 minutes
  {measureAccelerometer,   100, 0},   // Task 4: Measure Accelerometer 10x per second
  {sendData,            600000, 0},   // Task 5: Send Data every 10 minutes
  {checkBattery,        1000, 0}      // Task 6: Check Battery every second
};

void setup() {
  // Initialize Serial Communication for debugging (optional)
  Serial.begin(9600);
  
  // Initialize sensors and peripherals here
  // initializeLightSensor();
  // initializeTemperatureSensor();
  // initializeHeartRateSensor();
  // initializeAccelerometer();
  // initializeCommunicationModule();
  // initializeBatteryMonitor();
  
  Serial.println("Scheduler Initialized");
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time
  
  for (int i = 0; i < NUM_TASKS; i++) {
    // Check if it's time to run the task
    if (currentMillis - tasks[i].lastRun >= tasks[i].interval) {
      tasks[i].taskFunction();          // Execute the task
      tasks[i].lastRun = currentMillis; // Update the last run time
    }
  }
  
  // Optionally, perform other non-blocking operations here
}

// -------------------- Task Definitions -------------------- //

// Task 1: Measure Light
void measureLight() {
  // Example: Read from a light sensor
  // int lightValue = analogRead(LIGHT_SENSOR_PIN);
  // Serial.println("Light Level: " + String(lightValue));
  
  Serial.println("Executing Task 1: Measure Light");
  
  // TODO: Add your light measurement code here
}

// Task 2: Measure Temperature
void measureTemperature() {
  // Example: Read from a temperature sensor
  // float temperature = readTemperatureSensor();
  // Serial.println("Temperature: " + String(temperature) + " °C");
  
  Serial.println("Executing Task 2: Measure Temperature");
  
  // TODO: Add your temperature measurement code here
}

// Task 3: Measure Heart Rate
void measureHeartRate() {
  // Example: Read from a heart rate sensor
  // int heartRate = readHeartRateSensor();
  // Serial.println("Heart Rate: " + String(heartRate) + " BPM");
  
  Serial.println("Executing Task 3: Measure Heart Rate");
  
  // TODO: Add your heart rate measurement code here
}

// Task 4: Measure Accelerometer
void measureAccelerometer() {
  // Example: Read from an accelerometer
  // float ax, ay, az;
  // readAccelerometer(ax, ay, az);
  // Serial.println("Accelerometer: " + String(ax) + ", " + String(ay) + ", " + String(az));
  
  Serial.println("Executing Task 4: Measure Accelerometer");
  
  // TODO: Add your accelerometer measurement code here
}

// Task 5: Send Data
void sendData() {
  // Example: Send collected data over UART, SPI, or another communication protocol
  // sendCollectedData();
  // Serial.println("Data Sent Successfully");
  
  Serial.println("Executing Task 5: Send Data");
  
  // TODO: Add your data transmission code here
}

// Task 6: Check Battery
void checkBattery() {
  // Example: Check battery voltage
  // float batteryLevel = readBatteryVoltage();
  // Serial.println("Battery Level: " + String(batteryLevel) + " V");
  
  Serial.println("Executing Task 6: Check Battery");
  
  // TODO: Add your battery monitoring code here
}
