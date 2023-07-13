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
        cout<<"Next order: "<< m_currentOrder->goalx << " " << m_currentOrder->goaly << " " << m_currentOrder->goaltheta << " " << m_currentOrder->speed << endl;

        TurtleOrderType orderType = evaluate_order_type(*m_currentOrder);
        if (orderType == TurtleOrderType::Linear)
        {
            bool bRotationAdded = is_rotation_required_for_linear_move(*m_currentOrder);
            if(bRotationAdded == true)
            {
                cout<<"Rotation order added: " << m_currentOrder->goaltheta << endl;
            }
        }

        return true;
    }
    else
        return false;
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
        order.goaltheta = M_PI * values.front() / 180;
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
        isGoalReached = is_move_finished(*m_currentOrder);
        break;
    case TurtleOrderType::Circular:
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

    m_bIsPoseInitialized = true;
}



bool TurtleDriver::is_move_finished(const TurtleOrder& moveOrder) const
{
    float distance = calculate_distance_to_goal(m_pose.x, m_pose.y, moveOrder.goalx, moveOrder.goaly);

    bool isPassedby = is_target_passed_by(m_pose.headingAngle, moveOrder.goalx, moveOrder.goaly, m_pose.x, m_pose.y);

    if(distance < GOAL_REACHED_THRESHOLD || isPassedby == true)
        return true;
    else
        return false;
}



bool TurtleDriver::is_target_passed_by(const float& headingAngle, const float& goalX, const float& goalY, const float& currX, const float& currY) const
{
    float headingX = cos(headingAngle);
    float headingY = sin(headingAngle);

    float dx = goalX - currX;
    float dy = goalY - currY;

    float dotProduct = headingX * dx + headingY * dy;
    float angle = acos(dotProduct / (sqrt(dx*dx + dy*dy)));

    if(abs(angle) >  M_PI_2)
        return true;
    else
        return false;
}



bool TurtleDriver::is_rotation_finished(const TurtleOrder& rotationOrder) const
{
    float angleDiff = calculate_angle_to_goal(m_pose.headingAngle, rotationOrder.goaltheta);

    if(abs(angleDiff) < ROTATION_THRESHOLD)
    {
        calculate_angle_to_goal(m_pose.headingAngle, rotationOrder.goaltheta, true);
        cout<<"Rotation finished - theta: "<<m_pose.headingAngle<<", gtheta: "<<rotationOrder.goaltheta<<", diff: "<<angleDiff<<endl;
        return true;
    }
    else
        return false;
}




float TurtleDriver::calculate_distance_to_goal(const float& currX, const float& currY, const float& goalX, const float& goalY) const
{
    float dx = goalX - currX;
    float dy = goalY - currY;
    float distance = sqrt(dx*dx + dy*dy);
    return distance;
}



float TurtleDriver::calculate_angle_to_goal(const float& currAngle, const float& goalAngle, const bool& isVerbose) const
{
    /*float currX = cos(currAngle);
    float currY = sin(currAngle);
    float goalX = cos(goalAngle);
    float goalY = sin(goalAngle);

    float crossProduct = currX * goalY - currY * goalX;
    float angle = asin(crossProduct);

    if(isVerbose)
    {
        cout<<"curr angle: "<<currAngle<<"-> ["<<currX<<", "<<currY<<"], goal angle: "<<goalAngle<<"-> ["<<goalX<<", "<<goalY<<"], cross product: "<<crossProduct<<", asin: "<<angle<<endl;
    }*/

    float angle = goalAngle - currAngle;

    if(isVerbose)
        cout<<"curr angle: "<<currAngle<<", goal angle: "<<goalAngle<<", diff: "<<angle;

    if(angle > M_PI)
        angle -= 2 * M_PI;
    else if(angle < -M_PI)
        angle += 2 * M_PI;

    if(isVerbose)
        cout<<"modified: "<<angle<<endl;

    return angle;
}



VelocityCommand TurtleDriver::generate_linear_move_velocity_command(const TurtleOrder& linearOrder) const
{
    // Just move forward - direction is assumed as correct.
    VelocityCommand command;

    float distance = calculate_distance_to_goal(m_pose.x, m_pose.y, linearOrder.goalx, linearOrder.goaly);

    command.vx = linearOrder.speed * distance;
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
    float angleDiff = calculate_angle_to_goal(m_pose.headingAngle, rotationOrder.goaltheta);
    command.az = rotationOrder.speed*angleDiff;

    return command;
}



VelocityCommand TurtleDriver::generate_circular_move_velocity_command(const TurtleOrder& circularOrder) const
{
    VelocityCommand command;
    
    command.vx = circularOrder.goalx; // Radius of the circle
    command.vy = 0.0f;
    
    if(circularOrder.goaly > 0)
        command.az = 1;
    else
        command.az = -1;

    return command;
}




bool TurtleDriver::is_rotation_required_for_linear_move(const TurtleOrder& linearOrder)
{
    float dx = linearOrder.goalx - m_pose.x;
    float dy = linearOrder.goaly - m_pose.y;

    float angle = atan2(dy, dx);

    float angleDiff = calculate_angle_to_goal(m_pose.headingAngle, angle);

    cout<<"curr pos: ["<<m_pose.x<<", "<<m_pose.y<<"], angle: "<<angle<<", angleDiff: "<<angleDiff<<endl;

    if (abs(angleDiff) > ROTATION_THRESHOLD)
    {
        TurtleOrder rotateOrder;
        rotateOrder.goaltheta = angle;
        rotateOrder.speed = 1;
        add_order(rotateOrder);
        return true;
    }
    else
        return false;
}

