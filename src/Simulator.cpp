#include "Simulator.h"
#include <cmath>
#include <algorithm>

Simulator::Simulator(int width, int height)
    : roomWidth(width), roomHeight(height), robotX(width / 2), robotY(height / 2), robotAngle(0)
{
  room = std::vector<std::vector<int>>(roomHeight, std::vector<int>(roomWidth, 0));
  // Add walls
  for (int i = 0; i < roomWidth; i++)
  {
    room[0][i] = 1;              // Top wall
    room[roomHeight - 1][i] = 1; // Bottom wall
  }
  for (int i = 0; i < roomHeight; i++)
  {
    room[i][0] = 1;             // Left wall
    room[i][roomWidth - 1] = 1; // Right wall
  }
  // Add obstacles
  room[roomHeight / 2][roomWidth / 2] = 1;
}

float Simulator::toRadians(int angle)
{
  return angle * (M_PI / 180.0);
}

void Simulator::moveForward(float distance)
{
  float radians = toRadians(robotAngle);
  int dx = distance * cos(radians);
  int dy = distance * sin(radians);
  robotX = std::clamp(robotX + dx, 0, roomWidth - 1);
  robotY = std::clamp(robotY + dy, 0, roomHeight - 1);
}

void Simulator::setRobotPosition(int x, int y, int angle)
{
  robotX = x;
  robotY = y;
  robotAngle = angle;
}

void Simulator::move(float speed, int duration)
{
  float distance = speed * duration / 1000.0; // Convert speed and duration to distance
  moveForward(distance);
}

void Simulator::turn(int degrees)
{
  robotAngle = (robotAngle + degrees) % 360;
}

long Simulator::getDistance(int servoAngle)
{
  int angle = (robotAngle + servoAngle) % 360;
  float radians = toRadians(angle);
  int dx = cos(radians);
  int dy = sin(radians);
  int distance = 0;
  int x = robotX, y = robotY;

  while (x >= 0 && x < roomWidth && y >= 0 && y < roomHeight && room[y][x] == 0)
  {
    x += dx;
    y += dy;
    distance++;
  }

  return distance * 5; // Scale to approximate real-world distances
}
