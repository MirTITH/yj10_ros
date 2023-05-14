#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <controller_manager/controller_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <mutex>
#include <atomic>
#include <array>

template <typename T>
class SafeGlobal
{
public:
    SafeGlobal() {}
    SafeGlobal(const T &value) : data_(value) {}

    T get()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_;
    }

    void set(const T &value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = value;
    }

private:
    T data_;
    std::mutex mutex_;
};

SafeGlobal<std::vector<double>> JointPositions;
std::atomic<bool> HasGotJointPos(false);
SafeGlobal<KDL::Frame> NowEEPos; // current end effector position
SafeGlobal<KDL::Frame> ExpEEPos; // expected end effector position
unsigned int NumberOfJoint = 0;
std::atomic<double> JointStateRcvPeriod(0);

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

KDL::JntArray ConvertToKdlJntArray(std::vector<double> vec)
{
    KDL::JntArray result(vec.size());
    for (size_t i = 0; i < vec.size(); i++)
    {
        result(i) = vec.at(i);
    }
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

/**
 * @brief 发布关节命令
 * @note controller 只接受长度等于 controller_joint_num 的数组，如果长度不够，这个函数会自动填充 0，如果长度超出，会自动截断
 *
 * @param pub
 * @param joint_pos
 * @param controller_joint_num
 */
void PublishJoints(ros::Publisher &pub, KDL::JntArray joint_pos, int controller_joint_num)
{
    std_msgs::Float64MultiArray pos_msg;
    pos_msg.data.resize(controller_joint_num, 0);
    size_t num_of_effective_joint = joint_pos.data.size() < controller_joint_num ? joint_pos.data.size() : controller_joint_num;
    for (size_t i = 0; i < num_of_effective_joint; i++)
    {
        pos_msg.data.at(i) = joint_pos(i);
    }
    pub.publish(pos_msg);
}

void TargetSetter(ros::NodeHandle *node, std::string start_chain, std::string end_chain, std::string rot_link_name)
{
    KeyboardAsync keyboard;
    // KDL::Frame temp_pos;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped rot_link;
    geometry_msgs::TransformStamped rot_link_to_ee_link;

    std::array<double, 3> velocity_pos = {};
    std::array<double, 3> velocity_rpy = {};

    auto UpdateExpEEPos = [&]()
    {
        KDL::Frame temp_pos = ConvertToKdlFrame(rot_link.transform);
        auto period = JointStateRcvPeriod.load();
        for (size_t i = 0; i < 3; i++)
        {
            temp_pos.p(i) += velocity_pos.at(i) * period;
        }
        temp_pos.M = temp_pos.M * KDL::Rotation::RPY(velocity_rpy.at(0) * period, velocity_rpy.at(1) * period, velocity_rpy.at(2) * period);
        temp_pos = temp_pos * ConvertToKdlFrame(rot_link_to_ee_link.transform);
        ExpEEPos.set(temp_pos);
    };

    ros::Rate rate(100);

    while (node->ok())
    {
        auto ch = keyboard.Read();
        rot_link = tfBuffer.lookupTransform(start_chain, rot_link_name, ros::Time(0), ros::Duration(1));
        rot_link_to_ee_link = tfBuffer.lookupTransform(rot_link_name, end_chain, ros::Time(0), ros::Duration(1));

        velocity_pos.fill(0);
        velocity_rpy.fill(0);

        switch (ch)
        {
        case 'w':
            velocity_pos.at(1) = 0.1;
            UpdateExpEEPos();
            break;
        case 's':
            velocity_pos.at(1) = -0.1;
            UpdateExpEEPos();
            break;
        case 'a':
            velocity_pos.at(0) = -0.1;
            UpdateExpEEPos();
            break;
        case 'd':
            velocity_pos.at(0) = 0.1;
            UpdateExpEEPos();
            break;
        case 'r':
            velocity_pos.at(2) = 0.1;
            UpdateExpEEPos();
            break;
        case 'f':
            velocity_pos.at(2) = -0.1;
            UpdateExpEEPos();
            break;
        case 'j':
            velocity_rpy.at(1) = -1;
            UpdateExpEEPos();
            break;
        case 'l':
            velocity_rpy.at(1) = 1;
            UpdateExpEEPos();
            break;
        case 'i':
            velocity_rpy.at(0) = 1;
            UpdateExpEEPos();
            break;
        case 'k':
            velocity_rpy.at(0) = -1;
            UpdateExpEEPos();
            break;

        default:
            break;
        }

        if (ch != EOF)
        {
            printf("%c", ch);
        }

        rate.sleep();
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    JointStateRcvPeriod.store((current_time - last_time).toSec());
    // ROS_INFO("JointState received period: %fs", JointStateRcvPeriod.load());
    last_time = current_time;
    JointPositions.set(msg->position);
    HasGotJointPos = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_servo_node");
    ros::NodeHandle node("~");

    std::string chain_start, chain_end, rot_link_name, urdf_param, ik_solve_type;
    double ik_timeout;
    double ik_error;
    double ik_loop_rate;

    std::string joint_states_topic, controller_command_topic;
    int controller_joint_num;

    node.param("chain_start", chain_start, std::string("world"));
    node.param("chain_end", chain_end, std::string("fake_link6"));
    node.param("rot_link_name", rot_link_name, std::string("link5"));
    node.param("urdf_param", urdf_param, std::string("/robot_description"));
    node.param("ik_timeout", ik_timeout, 0.005);
    node.param("ik_error", ik_error, 1e-5);
    node.param("ik_solve_type", ik_solve_type, std::string("Speed"));
    node.param("ik_loop_rate", ik_loop_rate, 10.0);
    node.param("joint_states_topic", joint_states_topic, std::string("/joint_states"));
    node.param("controller_command_topic", controller_command_topic, std::string("/joint_group_position_controller/command"));
    node.param("controller_joint_num", controller_joint_num, 6);

    ROS_INFO_STREAM("node.getNamespace(): " << node.getNamespace());
    ROS_INFO_STREAM("chain_start: " << chain_start);
    ROS_INFO_STREAM("chain_end: " << chain_end);
    ROS_INFO_STREAM("rot_link_name: " << rot_link_name);
    ROS_INFO_STREAM("urdf_param: " << urdf_param);
    ROS_INFO_STREAM("ik_solve_type: " << ik_solve_type);
    ROS_INFO_STREAM("ik_timeout: " << ik_timeout);
    ROS_INFO_STREAM("ik_error: " << ik_error);
    ROS_INFO_STREAM("ik_loop_rate: " << ik_loop_rate);
    ROS_INFO_STREAM("joint_states_topic: " << joint_states_topic);
    ROS_INFO_STREAM("controller_command_topic: " << controller_command_topic);
    ROS_INFO_STREAM("controller_joint_num: " << controller_joint_num);

    // 获取joint_state
    ros::Subscriber sub = node.subscribe(joint_states_topic, 1, jointStateCallback);

    // 关节控制器
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::Publisher pos_pub = node.advertise<std_msgs::Float64MultiArray>(controller_command_topic, 1);

    if (chain_start == "" || chain_end == "")
    {
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }

    TRAC_IK::SolveType solve_type = TRAC_IK::Speed;

    if (ik_solve_type == "Speed")
    {
        solve_type = TRAC_IK::Speed;
    }
    else if (ik_solve_type == "Distance")
    {
        solve_type = TRAC_IK::Distance;
    }
    else if (ik_solve_type == "Manip1")
    {
        solve_type = TRAC_IK::Manip1;
    }
    else if (ik_solve_type == "Manip2")
    {
        solve_type = TRAC_IK::Manip2;
    }
    else
    {
        ROS_WARN_STREAM("Invalid ik_solve_type '" << ik_solve_type << "'");
    }

    // % NOTE: The last arguments to the constructors are optional.
    // % The type can be one of the following:
    // % Speed: returns very quickly the first solution found
    // % Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
    // % Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
    // % Manip2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, ik_timeout, ik_error, TRAC_IK::Distance);

    KDL::Chain chain;
    if (!ik_solver.getKDLChain(chain))
    {
        ROS_ERROR("There was no valid KDL chain found");
        return -1;
    }

    // Create joint array
    NumberOfJoint = chain.getNrOfJoints();

    // Create the frame that will contain the results
    KDL::JntArray joint_search_start_pos(NumberOfJoint); // 迭代数值解法的初值
    KDL::SetToZero(joint_search_start_pos);
    KDL::JntArray joint_pos_result(NumberOfJoint);
    KDL::SetToZero(joint_pos_result);

    // tf2
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    // 获取第一个末端位置消息
    while (node.ok())
    {
        if (tfBuffer.canTransform(chain_start, chain_end, ros::Time(0), ros::Duration(5)))
        {
            transformStamped = tfBuffer.lookupTransform(chain_start, chain_end, ros::Time(0), ros::Duration(1));
            NowEEPos.set(ConvertToKdlFrame(transformStamped.transform));
            ExpEEPos.set(NowEEPos.get()); // 相当于 ExpEEPos = NowEEPos
            ROS_INFO_STREAM("Got tf message");
            break;
        }
        else
        {
            ROS_WARN_STREAM("Unable to get tf message");
        }
    }

    // 获取第一个关节位置消息
    while (node.ok())
    {
        // ros::spinOnce();
        if (HasGotJointPos)
        {
            break;
        }
        else
        {
            ROS_INFO_STREAM("Unable to get joint state msg");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::thread key_thread(TargetSetter, &node, chain_start, chain_end, rot_link_name);

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ros::Rate rate(ik_loop_rate);

    // 主循环
    while (node.ok())
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform(chain_start, chain_end, ros::Time(0), ros::Duration(1));

            // ROS_INFO("end effector position: (%f, %f, %f)", transformStamped.transform.translation.x,
            //          transformStamped.transform.translation.y, transformStamped.transform.translation.z);

            NowEEPos.set(ConvertToKdlFrame(transformStamped.transform));
            joint_search_start_pos = ConvertToKdlJntArray(JointPositions.get());

            // 反运动学
            int rc = ik_solver.CartToJnt(joint_search_start_pos, ExpEEPos.get(), joint_pos_result);
            if (rc < 0)
            {
                printf("TRAC IK failed\n");
            }
            else
            {
                PublishJoints(pos_pub, joint_pos_result, controller_joint_num);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        // ros::spinOnce();
        rate.sleep();
    }

    key_thread.join();
    ros::waitForShutdown();

    return 0;
};