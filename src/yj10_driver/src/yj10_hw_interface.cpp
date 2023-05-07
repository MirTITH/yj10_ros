#include "yj10_driver/yj10_hw_interface.h"

Yj10HWInterface::Yj10HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
    ROS_INFO_NAMED("yj10_hw_interface", "Yj10HWInterface Ready.");
}

void Yj10HWInterface::read(ros::Duration &elapsed_time)
{
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------

    // ROS_INFO("Called read. ");
}

void Yj10HWInterface::write(ros::Duration &elapsed_time)
{
    // Safety
    enforceLimits(elapsed_time);

    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
    //
    // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
    // VELOCITY FROM POSITION WITH SMOOTHING, SEE
    // sim_hw_interface.cpp IN THIS PACKAGE
    //
    // DUMMY PASS-THROUGH CODE
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
        joint_position_[joint_id] = joint_position_command_[joint_id];
    // END DUMMY CODE
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
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
