#include <Arduino.h>
#include <Servo.h>

#define TRIG_PIN D1  // GPIO 5 for Trigger
#define ECHO_PIN D2  // GPIO 4 for Echo
#define SERVO_PIN D5 // GPIO 14 for Servo

#define MOTOR_A_IN1 D6 // GPIO 12 for Motor A Input 1
#define MOTOR_A_IN2 D7 // GPIO 13 for Motor A Input 2
#define MOTOR_B_IN1 D8 // GPIO 15 for Motor B Input 1
#define MOTOR_B_IN2 D9 // GPIO 3 for Motor B Input 2

#define NUM_READINGS 19  // Number of readings across the 180-degree arc
#define SAFE_DISTANCE 50 // Safe distance in centimeters

Servo ultraServo;
long distances[NUM_READINGS]; // Array to hold distance measurements

// Base speed for motors
int baseSpeed = 200; // Base speed, can be set from 0 (stopped) to 255 (full speed)

// Speed multiplier (percentage, range from 0.0 to 1.0)
float speedMultiplier = 0.5; // Default to 50% speed

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
}

long measureDistance()
{
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1; // Calculate distance in cm
  return distance;
}

void scanSurroundings()
{
  int angleStep = 180 / (NUM_READINGS - 1);
  for (int i = 0; i < NUM_READINGS; i++)
  {
    int angle = -90 + i * angleStep;
    ultraServo.write(90 + angle); // Servo position 0 to 180 corresponds to -90 to +90 degrees
    delay(300);                   // Wait for the servo to reach the position
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

  // Slow down and turn towards the best direction
  if (maxAverageDistance < SAFE_DISTANCE)
  {
    // Stop motors
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    delay(1000); // Stop for a moment

    // Turn towards the best direction
    if (turnAngle > 0)
    {
      // Turn right
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
    }
    else
    {
      // Turn left
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
    }
    delay(1000); // Turn for a set period
  }
}

void loop()
{
  // Adjust speeds based on multiplier
  int adjustedSpeed = baseSpeed * speedMultiplier;

  // Sweep the servo and measure distances
  scanSurroundings();

  // Determine the best direction and act on it
  findBestDirection();

  // Move forward
  analogWrite(MOTOR_A_IN1, adjustedSpeed);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_B_IN1, adjustedSpeed);
  digitalWrite(MOTOR_B_IN2, LOW);
  delay(500);
}
