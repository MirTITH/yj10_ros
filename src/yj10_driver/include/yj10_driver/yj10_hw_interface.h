
#pragma once

#include <ros_control_boilerplate/generic_hw_interface.h>
#include "yj10_driver/yj10.hpp"

/// \brief Hardware interface for a robot
class Yj10HWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
    /**
     * @brief Construct a new Yj10 hardware interface
     *
     * @param nh Node handle for topics
     * @param urdf_model when this is Null it looks for urdf at robot_description parameter server
     */
    Yj10HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

    /**
     * @brief Read the state from the robot hardware.
     * @note REQUIRED or wont compile.
     */
    virtual void read(ros::Duration &elapsed_time) override;

    /**
     * @brief Read the state from the robot hardware.
     * @note REQUIRED or wont compile.
     */
    virtual void write(ros::Duration &elapsed_time) override;

    /**
     * @brief Enforce limits for all values before writing.
     * @note REQUIRED or wont compile.
     */
    virtual void enforceLimits(ros::Duration &period) override;

    Yj10 arm;
}; // class
