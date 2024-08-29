#include <Arduino.h>
#include <Servo.h>

#define TRIG_PIN D7  // GPIO 5 for Trigger
#define ECHO_PIN D5  // GPIO 4 for Echo
#define SERVO_PIN D6 // GPIO 14 for Servo

#define MOTOR_A_IN1 D1 // GPIO 12 for Motor A Input 1
#define MOTOR_A_IN2 D3 // GPIO 13 for Motor A Input 2
#define MOTOR_B_IN1 D2 // GPIO 15 for Motor B Input 1
#define MOTOR_B_IN2 D4 // GPIO 3 for Motor B Input 2

#define NUM_READINGS 19  // Number of readings across the 180-degree arc
#define SAFE_DISTANCE 50 // Safe distance in centimeters

const bool DEMO_MODE = false;

#define TESTING 1
#ifdef TESTING
#include "Simulator.h"
Simulator simulator(20, 20); // Initialize simulator with room size 20x20
#endif

Servo ultraServo;
long distances[NUM_READINGS]; // Array to hold distance measurements

// Base speed for motors
const int baseSpeed = 200; // Base speed, can be set from 0 (stopped) to 255 (full speed)

// Speed multiplier (percentage, range from 0.0 to 1.0)
const float speedMultiplier = 0.15; // Default to 15% speed
// Adjust speeds based on multiplier
int adjustedSpeed = baseSpeed * speedMultiplier;

long measureDistance()
{
  const float SOUND_SPEED_DIVISOR = 29.1;
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / SOUND_SPEED_DIVISOR; // Calculate distance in cm
  return distance;
}

void scanSurroundingsSimulator()
{
  int angleStep = 180 / (NUM_READINGS - 1);
  for (int i = 0; i < NUM_READINGS; i++)
  {
    int angle = -90 + i * angleStep;
    ultraServo.write(90 + angle); // Servo position 0 to 180 corresponds to -90 to +90 degrees
    delay(100);                   // Wait for the servo to reach the position
    distances[i] = simulator.getDistance(angle);
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" - Distance: ");
    Serial.print(distances[i]);
    Serial.println(" cm");
  }
}

void scanSurroundings()
{
  int angleStep = 180 / (NUM_READINGS - 1);
  for (int i = 0; i < NUM_READINGS; i++)
  {
    int angle = -90 + i * angleStep;
    ultraServo.write(90 + angle); // Servo position 0 to 180 corresponds to -90 to +90 degrees
    delay(100);                   // Wait for the servo to reach the position
    distances[i] = measureDistance();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" - Distance: ");
    Serial.print(distances[i]);
    Serial.println(" cm");
  }
}

void findBestDirection()
{
  int bestDirection = 0;
  long maxAverageDistance = 0;

  // Iterate over the distances array in sectors
  for (int i = 1; i < NUM_READINGS - 1; i++)
  {
    long averageDistance = (distances[i - 1] + distances[i] + distances[i + 1]) / 3;
    if (averageDistance > maxAverageDistance)
    {
      maxAverageDistance = averageDistance;
      bestDirection = i;
    }
  }

  int angleStep = 180 / (NUM_READINGS - 1);
  int turnAngle = -90 + bestDirection * angleStep;

  Serial.print("Best direction: ");
  Serial.print(turnAngle);
  Serial.print(" degrees with average distance: ");
  Serial.println(maxAverageDistance);

  // Simulate turning towards the best direction
  if (maxAverageDistance < SAFE_DISTANCE)
  {
    simulator.turn(turnAngle);
  }
}

void simulateDriveForward()
{
  // Simulate moving forward
  simulator.move(adjustedSpeed, 500); // Move for 500ms with adjusted speed
}

void simulateDriveBackward()
{
  // Simulate moving backward
  simulator.move(-adjustedSpeed, 500); // Move backward for 500ms with adjusted speed
}

void driveForward()
{
  // Set motor A to move forward
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  // Set motor B to move forward
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  // Set motor speed
  analogWrite(MOTOR_A_IN1, adjustedSpeed);
  analogWrite(MOTOR_B_IN1, adjustedSpeed);
}

void driveBackward()
{
  // Set motor A to move backward
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  // Set motor B to move backward
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  // Set motor speed
  analogWrite(MOTOR_A_IN1, adjustedSpeed);
  analogWrite(MOTOR_B_IN1, adjustedSpeed);
}

void stopMotors()
{
  // In simulation, this would correspond to not moving
  // Do nothing or just a small delay to simulate pause
  delay(500);
}

void demoDriveAbitAround()
{
  // Simulate moving forward
  simulateDriveForward();
  // Stop for a moment
  stopMotors();
  // Simulate moving backward
  simulateDriveBackward();
  // Stop for a moment
  stopMotors();
}

void setup()
{
  Serial.begin(115200);

  // Setup pins for ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Setup pins for motors
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Attach the servo motor to the specified pin
  ultraServo.attach(SERVO_PIN);
  // servo specific microsecond pulse width range
  // I'm running it with 3,3V, so... 720-2000
  ultraServo.attach(D6, 720, 2000); // Start with standard pulse widths
  Serial.println("Setup complete");
}

void loop()
{
  if (DEMO_MODE)
  {
    demoDriveAbitAround();
    return;
  }

  if (TESTING)
  {
    scanSurroundingsSimulator();
    findBestDirection();    // Find the best direction based on simulated data
    simulateDriveForward(); // Simulate driving forward in the best direction
  }
  else
  {
    Serial.println("distance in meters is: ");
    Serial.println(measureDistance());

    // Sweep the servo and measure distances
    scanSurroundings();

    // Determine the best direction and act on it
    findBestDirection();
    driveForward();
  }
}
