#include "yj10_driver/yj10_hw_interface.h"
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <thread>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>

using namespace std;
#if ROS_VERSION_MINIMUM(1, 15, 7) // noetic
std::shared_ptr<Yj10HWInterface> yj10_hw_interface_instance;
#else
boost::shared_ptr<Yj10HWInterface> yj10_hw_interface_instance;
#endif

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

void SetClosingCurrentCallback(const std_msgs::Float32::ConstPtr &msg)
{
    if (msg.get()->data >= 0 && msg.get()->data <= yj10_hw_interface_instance->ClamperMaxCurrent())
    {
        while (ros::ok())
        {
            try
            {
                yj10_hw_interface_instance->WriteClamperClosingCurrent(msg.get()->data);
                ROS_INFO_STREAM("SetClosingCurrent to " << msg.get()->data);
                break;
            }
            catch (const std::exception &e)
            {
                ROS_WARN_STREAM("SetClosingCurrent failed: " << e.what() << " Retrying");
                this_thread::sleep_for(chrono::milliseconds(100));
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("SetClosingCurrent: invalid value: " << msg.get()->data << ". The max current is " << yj10_hw_interface_instance->ClamperMaxCurrent() << " A");
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yj10_hw_interface");
    ros::NodeHandle nh;

    // 夹爪服务
    auto s1 = nh.advertiseService("clamp/close", handle_close_clamp);
    auto s2 = nh.advertiseService("clamp/open", handle_open_clamp);
    auto s3 = nh.advertiseService("clamp/stop", handle_stop_clamp);
    auto clamp_current_sub = nh.subscribe("clamp/set_closing_current", 1, SetClosingCurrentCallback);
    // auto sub = nh.subscribe;
    // to do

    // NOTE: We run the ROS loop in a separate thread as external calls s   uch
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Create the hardware interface specific to your robot
    yj10_hw_interface_instance.reset(new Yj10HWInterface(nh));
    yj10_hw_interface_instance->init(); // size and register required interfaces inside generic_hw_interface.cpp

    // 设置初始位姿
    std::vector<double> joint_pos(5);
    joint_pos.at(0) = nh.param("/yj10_driver/initial_joint_position/joint1", 0.0);
    joint_pos.at(1) = nh.param("/yj10_driver/initial_joint_position/joint2", 0.0);
    joint_pos.at(2) = nh.param("/yj10_driver/initial_joint_position/joint3", 0.0);
    joint_pos.at(3) = nh.param("/yj10_driver/initial_joint_position/joint4", 0.0);
    joint_pos.at(4) = nh.param("/yj10_driver/initial_joint_position/joint5", 0.0);
    yj10_hw_interface_instance->SetInitialJointPos(joint_pos);

    // Start the control loop
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh, yj10_hw_interface_instance);

    std::string device;
    bool is_fake_connect = false;
    nh.param("yj10_driver/fake_connect", is_fake_connect, false);

    if (is_fake_connect)
    {
        // 假装连接上了，用于 debug
        yj10_hw_interface_instance->is_fake_connect = true;
        ROS_WARN_STREAM("yj10_driver/fake_connect is set to true. Use fake connect");
    }
    else
    {
        // 反复尝试连接
        while (ros::ok())
        {
            // 获取串口设备
            if (nh.getParam("yj10_driver/serial_device", device) == false)
            {
                ROS_ERROR_STREAM("Failed to get 'yj10_driver/serial_device' in param server.");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            else
            {
                ROS_INFO_STREAM("Read 'yj10_driver/serial_device' from param server. Device: " << device);
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
