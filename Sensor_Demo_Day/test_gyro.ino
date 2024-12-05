#include <Wire.h>

#define MPU6050_ADDR 0x68  // Default I2C address for MPU6050
#define MPU6050_ACCEL_XOUT_H 0x3B  // Accelerometer data register
#define LED_PIN 13  // LED pin to show activity (optional)

// Debugging flags
bool sensorInitialized = false;

void setup() {
  Serial.begin(9600);   // Initialize serial communication
  Wire.begin();         // Initialize I2C bus
  
  // Initialize the MPU6050 sensor
  if (initializeMPU6050()) {
    sensorInitialized = true;
    Serial.println("MPU6050 initialized successfully.");
  } else {
    sensorInitialized = false;
    Serial.println("MPU6050 initialization failed.");
  }
  
  pinMode(LED_PIN, OUTPUT);  // Initialize LED pin
}

void loop() {
  if (sensorInitialized) {
    readMPU6050();  // Read data from the MPU6050 sensor
  } else {
    Serial.println("Sensor not initialized. Please check connections.");
  }
  
  delay(500);  // Delay between reads
}

bool initializeMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake up the sensor by writing 0 to the power management register
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("Error initializing MPU6050: ");
    Serial.println(error);
    return false;
  }
  
  // Verify if the sensor is responding by reading the WHO_AM_I register (should return 0x68)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  
  if (Wire.available()) {
    byte whoAmI = Wire.read();
    if (whoAmI == 0x68) {
      Serial.println("MPU6050 is responding correctly.");
      return true;
    } else {
      Serial.print("MPU6050 WHO_AM_I register returned: 0x");
      Serial.println(whoAmI, HEX);
      return false;
    }
  } else {
    Serial.println("Failed to read WHO_AM_I register.");
    return false;
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
    
    // Debugging output
    Serial.print("AccelX: "); Serial.print(accelX);
    Serial.print(" AccelY: "); Serial.print(accelY);
    Serial.print(" AccelZ: "); Serial.print(accelZ);
    Serial.print(" Temp: "); Serial.print(temperature);
    Serial.print(" GyroX: "); Serial.print(gyroX);
    Serial.print(" GyroY: "); Serial.print(gyroY);
    Serial.print(" GyroZ: "); Serial.println(gyroZ);
    
    // Check if gyroscope values are zero
    if (gyroX == 0 && gyroY == 0 && gyroZ == 0) {
      Serial.println("Warning: Gyroscope values are zero.");
    } else {
      Serial.println("Gyroscope data looks good.");
    }
  } else {
    Serial.println("Error reading data from MPU6050.");
  }
}
