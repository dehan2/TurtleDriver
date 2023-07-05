#include "TurtleDriver.h"

#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>

TurtleDriver& TurtleDriver::operator=(const TurtleDriver& rhs)
{
    if (this != &rhs)
        copy(rhs);
    
    return *this;
}



void TurtleDriver::copy(const TurtleDriver& rhs)
{
    m_pose = rhs.m_pose;
    m_orders = rhs.m_orders;

    m_currentOrder = m_orders.begin();
    auto it = rhs.m_orders.begin();

    while(it != rhs.m_currentOrder)
    {
        m_currentOrder++;
        it++;
    }
}



bool TurtleDriver::move_to_next_order(const bool& isFirst)
{
    if (m_currentOrder == m_orders.end())
        return false;

    if(isFirst == false)
        m_currentOrder++;
    
    if(m_currentOrder != m_orders.end())
    {
        TurtleOrderType orderType = evaluate_order_type(*m_currentOrder);
        if (orderType == TurtleOrderType::Linear)
        {
            bool bRotationAdded = is_rotation_required_for_linear_move(*m_currentOrder);
            if(bRotationAdded == true)
            {
                cout<<"Rotation order added: " << m_currentOrder->goaltheta << endl;
            }
        }
    }

    return true;
}



bool TurtleDriver::read_order_file(const string& orderFile)
{
    m_orders.clear();

    ifstream file(orderFile);

    if (!file.is_open())
        return false;

    string line;
    while (getline(file, line))
    {
        list<float> values;

        istringstream iss(line);
        string token;
        while (getline(iss, token, ' '))
        {
            if (token.empty())
                continue;

            float value = stof(token);
            values.push_back(value);
        }

        if (values.size() != 4)
            continue;

        TurtleOrder order;
        order.goalx = values.front();
        values.pop_front();
        order.goaly = values.front();
        values.pop_front();
        order.goaltheta = values.front();
        values.pop_front();
        order.speed = values.front();

        m_orders.push_back(order);
    }

    initialize_current_order();
    return true;
}



bool TurtleDriver::is_goal_reached() const
{
    TurtleOrderType orderType = evaluate_order_type(*m_currentOrder);
    bool isGoalReached = false;

    switch(orderType)
    {
    case TurtleOrderType::Linear:
    case TurtleOrderType::Circular:
        isGoalReached = is_move_finished(*m_currentOrder);
        break;
    case TurtleOrderType::Rotation:
        isGoalReached = is_rotation_finished(*m_currentOrder);
        break;
    default:
    case TurtleOrderType::None:
        isGoalReached = false;
        break;
    }

    return isGoalReached;
}



VelocityCommand TurtleDriver::generate_velocity_command_to_achieve_goal(const TurtleOrder& order) const
{
    VelocityCommand command;

    TurtleOrderType orderType = evaluate_order_type(order);
    switch(orderType)
    {
    case TurtleOrderType::Linear:
        command = generate_linear_move_velocity_command(order);
        break;
    case TurtleOrderType::Rotation:
        command = generate_rotation_velocity_command(order);
        break;
    case TurtleOrderType::Circular:
        command = generate_circular_move_velocity_command(order);
        break;
    case TurtleOrderType::None:
    default:
        break;
    }

    return command;
}



TurtleOrderType TurtleDriver::evaluate_order_type(const TurtleOrder& order) const
{
    TurtleOrderType orderType = TurtleOrderType::None;
    if(order.goalx == 0.0f && order.goaly == 0.0f && order.goaltheta == 0.0f)
        return TurtleOrderType::None;

    // Case 1: Move --> goalx >= 0 , goaly >= 0, goaltheta = 0
    // Case 2: Rotate --> goalx = 0 , goaly = 0, goaltheta > 0
    // Case 3: Circular --> goalx >= 0 , goaly >= 0, goaltheta > 0
    
    if(order.goaltheta == 0.0f)
    {
        orderType = TurtleOrderType::Linear;
    }
    else
    {
        if(order.goalx == 0.0f && order.goaly == 0.0f) 
            orderType = TurtleOrderType::Rotation;
        else 
            orderType = TurtleOrderType::Circular;
    }

    return orderType;
}



void TurtleDriver::pose_callback(const turtlesim::Pose::ConstPtr& msg)
{
    m_pose.x = msg->x;
    m_pose.y = msg->y;
    m_pose.headingAngle = msg->theta;
}



bool TurtleDriver::is_move_finished(const TurtleOrder& moveOrder) const
{
    float dx = moveOrder.goalx - m_pose.x;
    float dy = moveOrder.goaly - m_pose.y;
    float distance = sqrt(dx*dx + dy*dy);

    cout<<"x: "<<m_pose.x<<" y: "<<m_pose.y<<", gx: "<<moveOrder.goalx<<", gy: "<<moveOrder.goaly<<", d: "<<distance<<endl;

    if(distance < GOAL_REACHED_THRESHOLD)
        return true;
    else
        return false;
}



bool TurtleDriver::is_rotation_finished(const TurtleOrder& rotationOrder) const
{
    float angleDiff = abs(rotationOrder.goaltheta - m_pose.headingAngle);
    if(angleDiff > M_PI)
        angleDiff = 2.0f*M_PI - angleDiff;

    cout<<"theta: "<<m_pose.headingAngle<<", gtheta: "<<rotationOrder.goaltheta<<", diff: "<<angleDiff<<endl;

    if(angleDiff < GOAL_REACHED_THRESHOLD)
        return true;
    else
        return false;
}



VelocityCommand TurtleDriver::generate_linear_move_velocity_command(const TurtleOrder& linearOrder) const
{
    // Just move forward - direction is assumed as correct.
    VelocityCommand command;
    command.vx = linearOrder.speed;
    command.vy = 0.0f;
    command.az = 0.0f;
    return command;
}



VelocityCommand TurtleDriver::generate_rotation_velocity_command(const TurtleOrder& rotationOrder) const
{
    VelocityCommand command;
    command.vx = 0.0f;
    command.vy = 0.0f;

    // If the angle is positive, rotate CCW
    float angleDiff = rotationOrder.goaltheta - m_pose.headingAngle;
    if(angleDiff > M_PI)
        angleDiff = - (2.0f*M_PI - angleDiff); // Minus means CW

    if(angleDiff > 0.0f)
        command.az = rotationOrder.speed;
    else
        command.az = -rotationOrder.speed;

    return command;
}



VelocityCommand TurtleDriver::generate_circular_move_velocity_command(const TurtleOrder& circularOrder) const
{
    VelocityCommand command;

    float dx = circularOrder.goalx - m_pose.x;
    float dy = circularOrder.goaly - m_pose.y;

    float distance = sqrt(dx*dx + dy*dy);

    float r = distance / (2.0f * sin(abs(circularOrder.goaltheta) / 2.0f));
        command.vx = 2.0f*r; // Radius of the circle
        command.vy = 0.0f;
        
        if(circularOrder.goaltheta > 0)
            command.az = circularOrder.speed;
        else
            command.az = -circularOrder.speed;

    return command;
}




bool TurtleDriver::is_rotation_required_for_linear_move(const TurtleOrder& linearOrder)
{
    float dx = linearOrder.goalx - m_pose.x;
    float dy = linearOrder.goaly - m_pose.y;

    float angle = atan2(dy, dx);
    if(angle < 0.0f)
        angle += 2.0f*M_PI;

    // tranlate pose theat to -pi to pi
    float headingAngle = m_pose.headingAngle;
    if(headingAngle < 0.0f)
        headingAngle += 2.0f*M_PI;

    float angleDiff = angle - headingAngle;
    if (abs(angleDiff) > GOAL_REACHED_THRESHOLD)
    {
        TurtleOrder rotateOrder;
        rotateOrder.goaltheta = angle;
        rotateOrder.speed = 0.01;
        add_order(rotateOrder);
        return true;
    }
    else
        return false;
}

