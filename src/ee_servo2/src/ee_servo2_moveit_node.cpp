#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <controller_manager/controller_manager.h>
#include "control_state.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "load_param.hpp"
#include <thread>

using namespace std;

moveit::planning_interface::MoveGroupInterface *Arm = nullptr;
std::string move_to_topic;

ControlState controlState;
ros::Subscriber move_to_sub;
ros::Publisher ee_servo_pub;

std_msgs::Empty empty_msg;

void MoveToCallback(const std_msgs::String &msg)
{
    if (Arm != nullptr)
    {
        controlState.SwitchTo(ControlState::State::MoveIt);
        if (Arm->setNamedTarget(msg.data) == true)
        {
            Arm->move();
        }

        ee_servo_pub.publish(empty_msg);
        controlState.SwitchTo(ControlState::State::Servo);
    };
}

void InitMoveIt(ros::NodeHandle &node)
{

    LoadParam(node, "move_to_topic", move_to_topic, std::string("move_to"));
    // 初始化需要使用move group控制的机械臂中的arm group
    Arm = new moveit::planning_interface::MoveGroupInterface("manipulator");

    // 设置目标位置所使用的参考坐标系
    Arm->setPoseReferenceFrame("world");

    // 当运动规划失败后，允许重新规划
    Arm->allowReplanning(true);

    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    Arm->setGoalPositionTolerance(0.001);
    Arm->setGoalOrientationTolerance(0.005);

    move_to_sub = node.subscribe(move_to_topic, 1, MoveToCallback);

    controlState.SwitchTo(ControlState::State::Servo);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_servo_moveit");
    ros::NodeHandle node("~");

    // 多线程
    ros::AsyncSpinner spinner(4);
    // 开启新的线程
    spinner.start();
    std::string ee_servo2_node_name;
    LoadParam(node, "ee_servo2_node_name", ee_servo2_node_name, std::string("end_effector_servo"));
    std::string update_cached_pos_topic_name = "/" + ee_servo2_node_name + "/update_cached_pos";
    ROS_INFO_STREAM("update_cached_pos_topic_name:" << update_cached_pos_topic_name);
    ee_servo_pub = node.advertise<std_msgs::Empty>(update_cached_pos_topic_name, 1);

    InitMoveIt(node);
    ros::waitForShutdown();
}
