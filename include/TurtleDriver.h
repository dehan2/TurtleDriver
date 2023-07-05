#pragma once

#include <list>
#include <string>
#include "ConstForTurtleDriver.h"
#include "ros/ros.h"
#include "turtlesim/Pose.h"

using namespace std;

class TurtleDriver
{
private:
    TurtlePose m_pose;

    list<TurtleOrder> m_orders;
    list<TurtleOrder>::iterator m_currentOrder = m_orders.end();

public:
    TurtleDriver() = default;
    TurtleDriver(const TurtleDriver& rhs) {copy(rhs);}
    ~TurtleDriver() = default;

    TurtleDriver& operator=(const TurtleDriver& rhs);

    void copy(const TurtleDriver& rhs);

    void set_pose(const TurtlePose& pose) {m_pose = pose;}
    void set_orders(const list<TurtleOrder>& orders) {m_orders = orders;}
    
    const TurtlePose& get_pose() const {return m_pose;}
    const list<TurtleOrder>& get_orders() const {return m_orders;}
    list<TurtleOrder>::iterator get_current_order() {return m_currentOrder;}

    void add_order(const TurtleOrder& order) {m_currentOrder = m_orders.insert(m_currentOrder, order);}
    void clear_orders() {m_orders.clear();}
    
    void initialize_current_order() {m_currentOrder = m_orders.begin();}
    bool move_to_next_order(const bool& isFirst = false);

    bool read_order_file(const string& orderFile);

    bool is_goal_reached() const;

    VelocityCommand generate_velocity_command_to_achieve_goal(const TurtleOrder& order) const;

    TurtleOrderType evaluate_order_type(const TurtleOrder& order) const;
    
    // ROS callback functions
    void pose_callback(const turtlesim::Pose::ConstPtr& msg);

private:
    bool is_move_finished(const TurtleOrder& moveOrder) const; // Both for linear and circular move
    bool is_rotation_finished(const TurtleOrder& rotationOrder) const;
    
    VelocityCommand generate_linear_move_velocity_command(const TurtleOrder& linearOrder) const;
    VelocityCommand generate_rotation_velocity_command(const TurtleOrder& rotationOrder) const;
    VelocityCommand generate_circular_move_velocity_command(const TurtleOrder& circularOrder) const;

    bool is_rotation_required_for_linear_move(const TurtleOrder& linearOrder);
};