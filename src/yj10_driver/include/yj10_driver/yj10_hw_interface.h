
#pragma once

#include <ros_control_boilerplate/generic_hw_interface.h>
#include "yj10_driver/yj10.hpp"

/// \brief Hardware interface for a robot
class Yj10HWInterface : public ros_control_boilerplate::GenericHWInterface
{
private:
    int read_retry_time = 0;
    int write_retry_time = 3;
    volatile bool is_connected = false;
    Yj10 arm;

public:
    volatile bool is_fake_connect = false; // 假装连接上了。如果此值为 true，获取的关节位置为假值

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

    void Connect(const std::string device, int device_id = 0x01, int baud = 9600, char parity = 'N', int data_bit = 8, int stop_bit = 1)
    {
        arm.Connect(device, device_id, baud, parity, data_bit, stop_bit);
        is_connected = true;
    }

}; // class
