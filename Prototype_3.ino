// Prototype I code 

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPIFFS.h>

#define motorPin 19

#define buzzerPin 25
#define ledPin 13

#define accThreshold 4
#define gyroThreshold 1

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);

  pinMode(buzzerPin, OUTPUT); // Set the buzzer pin as an output
  pinMode(ledPin, OUTPUT);    // Set the LED pin as an output
  pinMode(motorPin, OUTPUT);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  long accAverage = abs(sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)-9.8);
  long gyroAverage = sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z);

  // Serial.print("Acceleration average: ");
  // Serial.print(accAverage);
  // Serial.println(" m/s^2");

  // Serial.print("Gyro average: ");
  // Serial.print(gyroAverage);
  // Serial.println(" rad/s");

  int startTime = millis();
  digitalWrite(motorPin, HIGH);
  int endTime = millis();
  Serial.print(endTime-startTime);

  if (accAverage > accThreshold && gyroAverage > gyroThreshold) {
    // Serial.println("Fall detected!");

    // Serial.print("Fall Acceleration: ");
    // Serial.print(accAverage);
    // Serial.println(" m/s^2");
    // Serial.println(a.acceleration.x);
    // Serial.println(a.acceleration.y);
    // Serial.println(a.acceleration.z);
    // Serial.println();

    // Serial.print("Fall Gyro: ");
    // Serial.print(gyroAverage);
    // Serial.println(" rad/s");
    // Serial.println(g.gyro.x);
    // Serial.println(g.gyro.y);
    // Serial.println(g.gyro.z);
    // Serial.println();
    activate(); // trigger LED, nuzzer and actuator
    delay(2000);
  }
  delay(2000);
}

void activate(){
  int startTime = millis();
  digitalWrite(motorPin, HIGH);
  int endTime = millis();
  Serial.print(endTime-startTime);

  for (int i = 0; i < 5; i++){
    digitalWrite(buzzerPin, HIGH); // Turn on the buzzer
    digitalWrite(ledPin, HIGH);    // Turn on the LED

    delay(500); // Keep the buzzer and LED on for 0.5 second

    digitalWrite(buzzerPin, LOW);  // Turn off the buzzer
    digitalWrite(ledPin, LOW);     // Turn off the LED
    delay(500);
  }
  digitalWrite(motorPin, LOW);
}

void printData(){

  // Open the file for reading
  File file = SPIFFS.open("/sensor_data.txt", FILE_READ);
   if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  // Read data from the file
  while (file.available()) {
    Serial.write(file.read());
  }
  // Close the file
  file.close();
}

void storeData(long ax, long ay, long az, long gx, long gy, long gz){
  // Open file for writing
  File file = SPIFFS.open("/sensor_data.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Write sensor data to file
  file.print("Accelerometer: ");
  file.print(ax); file.print(", ");
  file.print(ay); file.print(", ");
  file.println(az);

  file.print("Gyroscope: ");
  file.print(gx); file.print(", ");
  file.print(gy); file.print(", ");
  file.println(gz);

  // Close the file
  file.close();
}
