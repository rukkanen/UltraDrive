#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>

class Simulator
{
private:
  int roomWidth;
  int roomHeight;
  int robotX, robotY;                 // Robot's position
  int robotAngle;                     // Robot's orientation in degrees
  std::vector<std::vector<int>> room; // 2D grid representing the room

  // Convert angle to radians
  float toRadians(int angle);

  // Get the next position based on distance and angle
  void moveForward(float distance);

public:
  // Constructor
  Simulator(int width, int height);

  // Set the robot's position and orientation
  void setRobotPosition(int x, int y, int angle);

  // Simulate moving the robot
  void move(float speed, int duration);

  // Simulate turning the robot
  void turn(int degrees);

  // Get the simulated distance based on the servo angle
  long getDistance(int servoAngle);
};

#endif // SIMULATOR_H
