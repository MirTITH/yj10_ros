#include "yj10_driver/yj10_hw_interface.h"
#include <thread>
#include <chrono>
#include "yj10_hw_interface.h"
#include <exception>

using namespace std;

void Yj10HWInterface::ReadClamper()
{
    std::lock_guard<std::mutex> guard(mux_);

    if (is_fake_connect)
    {
        throw runtime_error("Unsupported in fake connect mode");
    }

    if (is_connected == false)
    {
        return;
    }

    for (size_t i = 0; i < clamp_read_retry_time; i++)
    {
        try
        {
            arm.ReadClamper();
            return;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM("ReadClamper: " << e.what() << " Retrying...");
            this_thread::sleep_for(chrono::milliseconds(50));
        }
    }
    // 多试一次（主要是为了把 exception 传出去）
    arm.ReadClamper();
}
void Yj10HWInterface::ReadConfig()
{
    std::lock_guard<std::mutex> guard(mux_);
    arm.ReadAllInputRegs();
}

void Yj10HWInterface::WriteClamperInstruction(Yj10::ClamperState state)
{
    std::lock_guard<std::mutex> guard(mux_);

    if (is_fake_connect)
    {
        return;
    }

    if (is_connected == false)
    {
        return;
    }

    for (size_t i = 0; i < clamp_write_retry_time; i++)
    {
        try
        {
            arm.WriteClamperInstruction(state);
            return;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM("WriteClamper: " << e.what() << " Retrying...");
            this_thread::sleep_for(chrono::milliseconds(50));
        }
    }
    // 多试一次（主要是为了把 exception 传出去）
    arm.WriteClamperInstruction(state);
}

Yj10HWInterface::Yj10HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
    // last_joint_position_cmd_.resize(5);
    ROS_INFO_NAMED("yj10_hw_interface", "Yj10HWInterface Ready.");
}

void Yj10HWInterface::read(ros::Duration &elapsed_time)
{
    if (is_first_read)
    {
        is_first_read = false;
        auto origin_size = joint_position_.size();
        joint_position_ = initial_joint_position_;
        joint_position_.resize(origin_size);

        while (ros::ok())
        {
            // 尝试获取机械臂配置
            try
            {
                ReadConfig();
                break; // 成功后退出循环
            }
            catch (const std::exception &e)
            {
                ROS_ERROR_STREAM("Failed to ReadConfig. Retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }
    else
    {
        // 直接返回假数据
        joint_position_ = joint_position_command_;
    }

    if (is_fake_connect)
    {
        return;
    }

    // 读取夹爪
    yj10_driver::Clamp clamp_msg;
    if (is_connected)
    {
        for (int i = 0; i <= read_retry_time; i++)
        {
            try
            {
                // this_thread::sleep_for(chrono::milliseconds(50));
                std::lock_guard<std::mutex> guard(mux_);
                arm.ReadClamper();
                clamp_msg.current = arm.ClamperCurrent() / 1000.0;
                clamp_msg.state = (int)arm.Clamper();
                clamp_msg.closing_current = arm.ClamperClosingCurrent() / 1000.0;
                clamp_msg.max_current = arm.ClamperMaxCurrentMA() / 1000.0;
                clamp_pub.publish(clamp_msg);
                // ROS_INFO_STREAM("ReadClamper success!");
                break;
            }
            catch (const std::exception &e)
            {
                this_thread::sleep_for(chrono::milliseconds(50));
                // ROS_INFO_STREAM("ReadClamper failed. what(): " << e.what());
            }
        }
    }

    // 读取关节位置意义不大，读取到的是机械臂内部 PID 的期望值，而不是机械臂的实际姿态
    // if (is_connected)
    // {
    //     // 如果失败，就多尝试几次
    //     for (int retried_time = 0; retried_time <= read_retry_time; retried_time++)
    //     {
    //         try
    //         {
    //             this_thread::sleep_for(chrono::milliseconds(500));
    //             arm.ReadAllJointsPwm();
    //             ROS_INFO_STREAM("Read success!");
    //             for (size_t i = 0; i < num_joints_; i++)
    //             {
    //                 joint_position_.at(i) = arm.JointRad(i);
    //                 ROS_INFO_STREAM(i << ": " << joint_position_.at(i));
    //             }
    //         }
    //         catch (const std::exception &e)
    //         {
    //             ROS_INFO_STREAM("Read failed. what(): " << e.what());
    //         }
    //     }
    // }
}

void Yj10HWInterface::write(ros::Duration &elapsed_time)
{
    // Safety
    enforceLimits(elapsed_time);

    if (is_fake_connect)
    {
        return;
    }

    // 当位置更新的时候才写入
    if (is_connected && (last_joint_position_cmd_ != joint_position_command_))
    {
        for (int i = 0; i <= write_retry_time; i++)
        {
            try
            {
                // this_thread::sleep_for(chrono::milliseconds(50));
                std::lock_guard<std::mutex> guard(mux_);
                arm.WriteAllJointsRad(joint_position_command_);

                // 更新 last_joint_position_cmd_
                if (last_joint_position_cmd_.size() != joint_position_command_.size())
                {
                    last_joint_position_cmd_.resize(joint_position_command_.size());
                }

                last_joint_position_cmd_ = joint_position_command_;
            }
            catch (const std::exception &e)
            {
                // 机械臂的固件垃圾，写入失败经常发生，注释掉以不污染输出
                // ROS_ERROR_STREAM("Write failed. what(): " << e.what());
                // this_thread::sleep_for(chrono::milliseconds(50));
            }
        }
    }
}

void Yj10HWInterface::enforceLimits(ros::Duration &period)
{
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    // vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
}

void Yj10HWInterface::Connect(const std::string device, int device_id, int baud, char parity, int data_bit, int stop_bit)
{
    std::lock_guard<std::mutex> guard(mux_);
    arm.Connect(device, device_id, baud, parity, data_bit, stop_bit);
    is_connected = true;
}
