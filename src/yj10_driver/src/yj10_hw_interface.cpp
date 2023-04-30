#include "yj10_driver/yj10_hw_interface.h"

namespace yj10_control_ns
{

    Yj10HWInterface::Yj10HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        ROS_INFO("Yj10HWInterface declared.");
    }

    void Yj10HWInterface::init()
    {
        // Call parent class version of this function
        /*
        this looks at controller yaml "hardware" namespace to get "joints". from this list the number of joints is known so hardware interfaces are initialized.
        it starts a joint_state, position, velocity and effort iterface. joint limits are also grabbed from parameter server urdf if urdf=NULL.
        */
        ros_control_boilerplate::GenericHWInterface::init();

        ROS_INFO("Yj10HWInterface initiated.");
    }

    void Yj10HWInterface::read(ros::Duration &elapsed_time)
    {
        // ROS_INFO("Called read. ");
        for (size_t i = 0; i < num_joints_; i++)
        {
            joint_position_.at(i) = joint_position_command_.at(i);
        }

        // ros::spinOnce(); //is not required here because of asyncspinner
    }

    void Yj10HWInterface::write(ros::Duration &elapsed_time)
    {
        // ROS_INFO("Called write. pos=%lf", joint_position_command_.at(0));
    }

    void Yj10HWInterface::enforceLimits(ros::Duration &period)
    {
        // Enforces position and velocity
        // pos_jnt_sat_interface_.enforceLimits(period);
    }

} // namespace yj10_control_ns
