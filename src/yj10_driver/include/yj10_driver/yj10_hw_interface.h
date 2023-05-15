
#pragma once

#include <ros_control_boilerplate/generic_hw_interface.h>
#include "yj10_driver/yj10.hpp"
#include <mutex>
#include "yj10_driver/Clamp.h"

/// \brief Hardware interface for a robot
class Yj10HWInterface : public ros_control_boilerplate::GenericHWInterface
{
private:
    // 注意，这个机械臂的固件很垃圾，交替读写一定会失败。但是可以读两次写两次。因此重试次数必须不为 0
    int read_retry_time = 0;        // 读取重试次数，设为 0 即不重试，设为 -1 即不读取
    int write_retry_time = 0;       // 写入重试次数，设为 0 即不重试，设为 -1 即不写入
    int clamp_write_retry_time = 3; // 夹爪写入重试次数
    int clamp_read_retry_time = 3;  // 夹爪读取重试次数
    volatile bool is_connected = false;
    bool is_first_read = true;
    decltype(joint_position_) last_joint_position_cmd_;
    decltype(joint_position_) initial_joint_position_;
    Yj10 arm;
    std::mutex mux_;
    decltype(nh_.advertise<yj10_driver::Clamp>("clamp/fdb", 1)) clamp_pub;

public:
    volatile bool is_fake_connect = false; // 假装连接上了。如果此值为 true，获取的关节位置为假值

    void init() override
    {
        clamp_pub = nh_.advertise<yj10_driver::Clamp>("clamp/fdb", 1);
        ros_control_boilerplate::GenericHWInterface::init();
    }

    void ReadClamper();

    /**
     * @brief 夹爪最大电流
     * @note 这个函数返回的单位是安培，而 Yj10::ClamperMaxCurrent() 返回的是毫安
     * @return double 安培
     */
    double ClamperMaxCurrent()
    {
        return arm.ClamperMaxCurrentMA() / 1000.0;
    }

    /**
     * @brief 设置夹爪闭合电流
     *
     * @param current 安培（最大 0.8）
     */
    void WriteClamperClosingCurrent(double current)
    {
        std::lock_guard<std::mutex> guard(mux_);
        arm.WriteClamperClosingCurrent(current * 1000);
    }

    void ReadConfig();

    void SetInitialJointPos(decltype(joint_position_) joint_pos)
    {
        initial_joint_position_ = joint_pos;
    }

    Yj10::ClamperState GetClamperState() const
    {
        return arm.Clamper();
    }

    void WriteClamperInstruction(Yj10::ClamperState state);

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

    void Connect(const std::string device, int device_id = 0x01, int baud = 9600, char parity = 'N', int data_bit = 8, int stop_bit = 1);
}; // class
