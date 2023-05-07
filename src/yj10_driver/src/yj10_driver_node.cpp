#include "yj10_driver/yj10_hw_interface.h"
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <thread>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yj10_hw_interface");
    ros::NodeHandle nh;

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Create the hardware interface specific to your robot
    std::shared_ptr<Yj10HWInterface> yj10_hw_interface_instance(new Yj10HWInterface(nh));

    std::string device;

    bool is_fake_connect = false;

    nh.param("yj10_connection/fake_connect", is_fake_connect, false);

    if (is_fake_connect)
    {
        yj10_hw_interface_instance->is_fake_connect = true;
        ROS_WARN_STREAM("yj10_connection/fake_connect is set to true. Use fake connect");
    }
    else
    {
        if (nh.getParam("yj10_connection/serial_device", device) == false)
        {
            ROS_ERROR_STREAM("Failed to get 'yj10_connection/serial_device' in param server.");
            return 1;
        }
        else
        {
            ROS_INFO_STREAM("Read yj10_connection/serial_device from param server:" << device);
        }

        while (ros::ok())
        {
            try
            {
                yj10_hw_interface_instance->arm.Connect(device);
            }
            catch (const std::exception &e)
            {
                ROS_ERROR_STREAM("Failed to connect to YJ10 arm. Device:[" << device << "] Retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    }

    yj10_hw_interface_instance->init(); // size and register required interfaces inside generic_hw_interface.cpp

    // Start the control loop
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh, yj10_hw_interface_instance);
    control_loop.run(); // Blocks until shutdown signal recieved -> read -> update -> write -> repeat inside generic_hw_control_loop.cpp

    return 0;
}
