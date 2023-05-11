#include "yj10_driver/yj10_hw_interface.h"
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <thread>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

using namespace std;

std::shared_ptr<Yj10HWInterface> yj10_hw_interface_instance;

static bool handle_close_clamp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Closing clamp");
    yj10_hw_interface_instance->WriteClamperInstruction(Yj10::ClamperState::Close);

    return true;
}

static bool handle_stop_clamp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Stoping clamp");
    yj10_hw_interface_instance->WriteClamperInstruction(Yj10::ClamperState::Stop);

    return true;
}

static bool handle_open_clamp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Opening clamp");
    yj10_hw_interface_instance->WriteClamperInstruction(Yj10::ClamperState::Open);
    return true;
}

static bool handle_get_clamp_state(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    yj10_hw_interface_instance->ReadClamper();
    auto state = yj10_hw_interface_instance->GetClamperState();
    switch (state)
    {
    case Yj10::ClamperState::Close:
        res.message = "Close";
        res.success = true;
        break;
    case Yj10::ClamperState::Error:
        res.message = "Error";
        res.success = true;
        break;
    case Yj10::ClamperState::Middle:
        res.message = "Middle";
        res.success = true;
        break;
    case Yj10::ClamperState::Open:
        res.message = "Open";
        res.success = true;
        break;
    case Yj10::ClamperState::Stop:
        res.message = "Stop";
        res.success = true;
        break;

    default:
        res.message = "Unknown";
        res.success = false;
        break;
    }
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yj10_hw_interface");
    ros::NodeHandle nh;

    // 夹爪服务
    auto s1 = nh.advertiseService("clamp/close", handle_close_clamp);
    auto s2 = nh.advertiseService("clamp/open", handle_open_clamp);
    auto s3 = nh.advertiseService("clamp/stop", handle_stop_clamp);
    auto s4 = nh.advertiseService("clamp/get_state", handle_get_clamp_state);

    // NOTE: We run the ROS loop in a separate thread as external calls s   uch
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Create the hardware interface specific to your robot
    yj10_hw_interface_instance.reset(new Yj10HWInterface(nh));

    yj10_hw_interface_instance->init(); // size and register required interfaces inside generic_hw_interface.cpp

    // Start the control loop
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh, yj10_hw_interface_instance);

    std::string device;
    bool is_fake_connect = false;
    nh.param("yj10_connection/fake_connect", is_fake_connect, false);

    if (is_fake_connect)
    {
        // 假装连接上了，用于 debug
        yj10_hw_interface_instance->is_fake_connect = true;
        ROS_WARN_STREAM("yj10_connection/fake_connect is set to true. Use fake connect");
    }
    else
    {
        // 反复尝试连接
        while (ros::ok())
        {
            // 获取串口设备
            if (nh.getParam("yj10_connection/serial_device", device) == false)
            {
                ROS_ERROR_STREAM("Failed to get 'yj10_connection/serial_device' in param server.");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            else
            {
                ROS_INFO_STREAM("Read 'yj10_connection/serial_device' from param server. Device: " << device);
            }

            // 尝试连接
            try
            {
                yj10_hw_interface_instance->Connect(device);

                // 连接成功后退出循环
                break;
            }
            catch (const std::exception &e)
            {
                ROS_ERROR_STREAM("Failed to connect to YJ10 arm. Retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    }

    control_loop.run(); // Blocks until shutdown signal recieved -> read -> update -> write -> repeat inside generic_hw_control_loop.cpp

    return 0;
}
