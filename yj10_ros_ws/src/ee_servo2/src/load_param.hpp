#pragma once
#include <ros/ros.h>

template <typename T>
bool LoadParam(ros::NodeHandle &node, const std::string &param_name, T &param_val, const T &default_val)
{
    auto result = node.param(param_name, param_val, default_val);
    if (result == true)
    {
        ROS_INFO_STREAM("LoadParam: " << param_name << " = " << param_val);
    }
    else
    {
        ROS_INFO_STREAM("LoadDefaultParam: " << param_name << " = " << param_val);
    }
    return result;
}
