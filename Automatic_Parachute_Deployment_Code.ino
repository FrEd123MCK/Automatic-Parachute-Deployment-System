#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Servo.h>

#define BMP_CS 10

Adafruit_BMP3XX bmp;
Servo myServo;

float initialPressure_HPA = 1013.25; // Placeholder for initial pressure
float altitudeOffset = 0.0; // Offset to zero the altitude
const int numReadings = 20; // Number of readings for averaging
const int servoPin = 3; // Change to D3 for servo control
const float deployThreshold = 0.5; // Altitude threshold for deployment in meters

enum State {
  WAITING_FOR_THRESHOLD_CROSS,
  DEPLOY_READY,
  DEPLOYED
};

State currentState = WAITING_FOR_THRESHOLD_CROSS;
unsigned long firstThresholdCrossTime = 0; // Time when altitude first goes above threshold

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize the sensor using SPI
  if (!bmp.begin_SPI(BMP_CS)) {
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING); // Disable temperature oversampling
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X); // Adjust as necessary for accuracy vs. speed
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ); // Set to the highest output data rate

  delay(2000); // Initial delay to ensure the sensor has stabilized

  // Take multiple initial pressure readings to set as the reference pressure
  float totalPressure = 0;
  for (int i = 0; i < numReadings; i++) {
    if (bmp.performReading()) {
      totalPressure += bmp.pressure / 100.0; // Convert Pa to hPa
    } else {
      Serial.println("Failed to perform initial reading.");
    }
    delay(100); // Small delay between readings
  }
  initialPressure_HPA = totalPressure / numReadings;
  Serial.print("Averaged initial pressure set to: ");
  Serial.print(initialPressure_HPA);
  Serial.println(" hPa");

  // Calculate the initial altitude and set it as the offset
  altitudeOffset = bmp.readAltitude(initialPressure_HPA);
  Serial.print("Initial altitude set to: ");
  Serial.print(altitudeOffset);
  Serial.println(" m");

  // Initialize the servo motor on D3
  myServo.attach(servoPin);
  myServo.write(50); // Start at 50 degrees
  Serial.println("Servo initialized at 50 degrees");
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Calculate the current altitude and subtract the initial offset
  float currentAltitude = bmp.readAltitude(initialPressure_HPA) - altitudeOffset;
  Serial.print("Approx. Altitude = ");
  Serial.print(currentAltitude);
  Serial.println(" m");

  // State machine to manage the deployment process
  switch (currentState) {
    case WAITING_FOR_THRESHOLD_CROSS:
      if (currentAltitude > deployThreshold) {
        firstThresholdCrossTime = millis(); // Record the time of the crossing
        Serial.println("Threshold crossed. Waiting for 3 seconds...");
        currentState = DEPLOY_READY;
      }
      break;

    case DEPLOY_READY:
      if (currentAltitude <= deployThreshold) {
        // Reset the state if altitude drops below threshold
        Serial.println("Altitude dropped below threshold. Resetting...");
        currentState = WAITING_FOR_THRESHOLD_CROSS;
      } else if (millis() - firstThresholdCrossTime >= 3000) { // Check if three seconds have passed
        Serial.println("Ready to deploy on threshold drop.");
        currentState = DEPLOYED;
      }
      break;

    case DEPLOYED:
      if (currentAltitude <= deployThreshold) {
        myServo.write(0); // Move to 0 degrees
        Serial.println("Servo moved to 0 degrees");
        while (true) {
          // Device is halted after servo deployment
        }
      }
      break;
  }

  delay(15); // Match the sensor's reading rate of 200 Hz (5 milliseconds per reading)
}
