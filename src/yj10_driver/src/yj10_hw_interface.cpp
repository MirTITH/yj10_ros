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

    for (size_t i = 0; i < read_retry_time; i++)
    {
        try
        {
            arm.ReadClamper();
            return;
        }
        catch (const std::exception &e)
        {
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
    // 多试一次
    arm.ReadClamper();
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

    for (size_t i = 0; i < write_retry_time; i++)
    {
        try
        {
            arm.WriteClamperInstruction(state);
            return;
        }
        catch (const std::exception &e)
        {
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
    // 多试一次
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
    std::lock_guard<std::mutex> guard(mux_);
    // 这个机械臂的固件非常垃圾，交替读写会失败，于是不读取机械臂姿态了，返回假数据
    // 即使读取也意义不大，读取到的是机械臂内部 PID 的期望值，而不是机械臂的实际姿态
    joint_position_ = joint_position_command_;

    if (is_fake_connect)
    {
        return;
    }

    // if (is_connected)
    // {
    //     for (int i = 0; i <= read_retry_time; i++)
    //     {
    //         try
    //         {
    //             arm.ReadClamper();
    //             ROS_INFO_STREAM("ReadClamper success!");
    //             break;
    //         }
    //         catch (const std::exception &e)
    //         {
    //             this_thread::sleep_for(chrono::milliseconds(80));
    //             ROS_INFO_STREAM("ReadClamper failed. what(): " << e.what());
    //         }
    //     }
    // }

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
    std::lock_guard<std::mutex> guard(mux_);
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
                this_thread::sleep_for(chrono::milliseconds(80));
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
