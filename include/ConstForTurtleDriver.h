#pragma once

struct TurtlePose
{
    float x = 0.0f;
    float y = 0.0f;
    float headingAngle = 0.0f;
};


struct TurtleOrder
{
    float goalx = 0.0f;
    float goaly = 0.0f;
    float goaltheta = 0.0f;
    float speed = 1.0f;
};


struct VelocityCommand
{
    float vx = 0.0f;
    float vy = 0.0f;
    float az = 0.0f;
};


enum class TurtleOrderType {None, Linear, Rotation, Circular};

const float GOAL_REACHED_THRESHOLD = 0.1f;
const float ROTATION_THRESHOLD = 0.0174f; // 1 degree