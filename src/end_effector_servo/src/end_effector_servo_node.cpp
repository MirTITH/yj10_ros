#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <thread>
#include <controller_manager/controller_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <mutex>

std::mutex mux;
KDL::Frame NowPos;

KDL::Frame ConvertToKdlFrame(geometry_msgs::Transform transform)
{
    KDL::Frame result;

    result.p(0) = transform.translation.x;
    result.p(1) = transform.translation.y;
    result.p(2) = transform.translation.z;
    result.M = KDL::Rotation::Quaternion(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w);
    return result;
}

class KeyboardAsync
{
private:
    struct termios oldt, newt;
    int flags;

public:
    KeyboardAsync()
    {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    /**
     * @brief 异步读取键盘
     *
     * @return int 如果没读到，返回 EOF
     */
    int Read()
    {
        return getchar();
    }

    ~KeyboardAsync()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
};

void TargetSetter(ros::NodeHandle *node, KDL::Frame *now_pos, KDL::Frame *target_pos)
{
    KeyboardAsync keyboard;
    KDL::Frame temp_pos;

    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ros::Rate rate(100);

    *target_pos = *now_pos;

    while (node->ok())
    {
        auto ch = keyboard.Read();
        mux.lock();

        switch (ch)
        {
        case 'w':
            *target_pos = *now_pos;
            target_pos->p(1) += 0.01;
            break;
        case 's':
            *target_pos = *now_pos;
            target_pos->p(1) -= 0.01;
            break;
        case 'a':
            *target_pos = *now_pos;
            target_pos->p(0) -= 0.01;
            break;
        case 'd':
            *target_pos = *now_pos;
            target_pos->p(0) += 0.01;
            break;
        case 'f':
            *target_pos = *now_pos;
            target_pos->p(2) += 0.01;
            break;
        case 'v':
            *target_pos = *now_pos;
            target_pos->p(2) -= 0.01;
            break;

        default:
            break;
        }
        mux.unlock();

        // if (ch != EOF)
        // {
        //     printf("Got %c\n", ch);
        // }

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_servo_node");
    ros::NodeHandle node;

    // tf2
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // 发布关节指令
    ros::Publisher pos_pub = node.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
    std_msgs::Float64MultiArray pos_msg;

    //
    std::string chain_start, chain_end, urdf_param;
    urdf_param = "/robot_description";
    chain_start = "world";
    chain_end = "fake_link6";

    double timeout = 0.005;
    const double error = 1e-5;

    if (chain_start == "" || chain_end == "")
    {
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }

    // node.param("timeout", timeout, 0.005);
    // node.param("urdf_param", urdf_param, std::string("/robot_description"));

    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed);

    KDL::Chain chain;
    bool valid = ik_solver.getKDLChain(chain);

    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
        return -1;
    }

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();

    // Create the frame that will contain the results
    KDL::Frame target_pos;

    KDL::JntArray joint_search_start_pos(nj); // 迭代数值解法的初值
    KDL::SetToZero(joint_search_start_pos);
    KDL::JntArray result(joint_search_start_pos);

    pos_msg.data.resize(nj, 0);

    ros::Rate rate(10.0);

    geometry_msgs::TransformStamped transformStamped;

    transformStamped = tfBuffer.lookupTransform("world", "fake_link6",
                                                ros::Time(0), ros::Duration(1));

    mux.lock();
    NowPos = ConvertToKdlFrame(transformStamped.transform);
    mux.unlock();

    std::thread key_thread(TargetSetter, &node, &NowPos, &target_pos);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (node.ok())
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform("world", "fake_link6",
                                                        ros::Time(0), ros::Duration(1));

            // ROS_INFO("end effector position: (%f, %f, %f)", transformStamped.transform.translation.x,
            //          transformStamped.transform.translation.y, transformStamped.transform.translation.z);

            mux.lock();
            NowPos = ConvertToKdlFrame(transformStamped.transform);
            KDL::Frame temp_target_pos = target_pos;
            mux.unlock();

            for (size_t i = 0; i < 3; i++)
            {
                std::cout << temp_target_pos.p(i) << " ";
            }
            std::cout << std::endl;

            int rc = ik_solver.CartToJnt(joint_search_start_pos, temp_target_pos, result);
            joint_search_start_pos = result;

            if (rc < 0)
            {
                printf("TRAC IK failed\n");
            }
            else
            {
                for (unsigned int i = 0; i < nj; i++)
                {
                    pos_msg.data.at(i) = result(i);
                }
                pos_pub.publish(pos_msg);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }

    key_thread.join();

    return 0;
};