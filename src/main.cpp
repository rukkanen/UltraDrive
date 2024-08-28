#include <Arduino.h>
#include <Servo.h>

#define TRIG_PIN D0  // GPIO 5 for Trigger
#define ECHO_PIN D5  // GPIO 4 for Echo
#define SERVO_PIN D6 // GPIO 14 for Servo

#define MOTOR_A_IN1 D1 // GPIO 12 for Motor A Input 1
#define MOTOR_A_IN2 D3 // GPIO 13 for Motor A Input 2
#define MOTOR_B_IN1 D2 // GPIO 15 for Motor B Input 1
#define MOTOR_B_IN2 D4 // GPIO 3 for Motor B Input 2

/*
Board uses 4 pins to control motors. PWMA, PWMB- speed of motors.
DA, DB-direction of motors(0-straight;1-reverse).
Here pinout for NodeMcu Amica.
    PWMA-GPIO5(D20)
    PWMB-GPIO4(D19)
    DA-GPIO0(18)
    DB-GPIO2(17)
int PWMA=5;//Right side
int PWMB=4;//Left side
int DA=0;//Right reverse
int DB=2;//Left reverse
*/

#define NUM_READINGS 19  // Number of rejadings across the 180-degree arc
#define SAFE_DISTANCE 50 // Safe distance in centimeters

const bool DEMO_MODE = false;

Servo ultraServo;
long distances[NUM_READINGS]; // Array to hold distance measurements

// Base speed for motors
const int baseSpeed = 200; // Base speed, can be set from 0 (stopped) to 255 (full speed)

// Speed multiplier (percentage, range from 0.0 to 1.0)
const float speedMultiplier = 0.15; // Default to 50% speed
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

void driveForward()
{
  // Move forward
  analogWrite(MOTOR_A_IN1, baseSpeed);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_B_IN1, baseSpeed);
  digitalWrite(MOTOR_B_IN2, LOW);
  delay(500);
}

void driveBackward()
{
  // Move backward
  analogWrite(MOTOR_A_IN1, baseSpeed);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_B_IN1, baseSpeed);
  digitalWrite(MOTOR_B_IN2, HIGH);
  delay(500);
}

void stopMotors()
{
  // Stop motors
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}

void dance()
{
  // Dance routine
  for (int i = 0; i < 4; i++)
  {
    // Move forward
    driveForward();
    // Turn right
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    delay(1000);
  }
}

void demoDriveAbitAround()
{

  // Move forward
  driveForward();
  // Stop for a moment
  stopMotors();
  // Move backward
  driveBackward();
  // Stop for a moment
  stopMotors();
  // Dance
  dance();
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

  Serial.println("distance in meters is: ");
  Serial.println(measureDistance());

  // Sweep the servo and measure distances
  scanSurroundings();

  // Determine the best direction and act on it
  findBestDirection();
  driveForward();
}
