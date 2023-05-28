#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <controller_manager/controller_manager.h>
#include <mutex>
#include <atomic>
#include <array>
#include <std_srvs/Empty.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "safe_global.hpp"
#include <std_msgs/String.h>
#include <controller_manager_msgs/SwitchController.h>
#include <kdl_parser/kdl_parser.hpp>

// Params
std::string chain_start, chain_end;
std::string urdf_param;
std::string joint_states_topic;
std::string joint_group_position_controller;
std::string end_effector_velocity_topic;
std::string ik_solve_type;
double ik_timeout;
double ik_error;
int controller_joint_num;

// Global values
// std::atomic<double> JointStateRcvPeriod(0);
TRAC_IK::TRAC_IK *ik_solver;                // 逆运动学解算
KDL::ChainFkSolverPos_recursive *fk_solver; // 正运动学解算
KDL::Chain fake_chain;                      // 包括假关节的关节链
std::array<double, 3> velocity_pos;
std::array<double, 3> velocity_rpy; // 0: r,绕 x 旋转；1: p,绕 y 旋转；2: y,绕 z 旋转；
double jnt_state_period = 0;

ros::Publisher pos_pub;

template <typename T>
bool LoadParam(ros::NodeHandle &node, const std::string &param_name, T &param_val, const T &default_val)
{
    auto result = node.param(param_name, param_val, default_val);
    if (result == true)
    {
        ROS_INFO_STREAM("LoadParam: " << param_name << " = " << param_val);
    }
    else
    {
        ROS_INFO_STREAM("LoadDefaultParam: " << param_name << " = " << param_val);
    }
    return result;
}

void LoadAllParam(ros::NodeHandle &node)
{
    LoadParam(node, "chain_start", chain_start, std::string("world"));
    LoadParam(node, "chain_end", chain_end, std::string("link4"));
    LoadParam(node, "urdf_param", urdf_param, std::string("/robot_description"));
    LoadParam(node, "joint_states_topic", joint_states_topic, std::string("/joint_states"));
    LoadParam(node, "joint_group_position_controller", joint_group_position_controller, std::string("/joint_group_position_controller"));
    LoadParam(node, "end_effector_velocity_topic", end_effector_velocity_topic, std::string("end_effector_velocity"));
    LoadParam(node, "ik_solve_type", ik_solve_type, std::string("Distance"));
    LoadParam(node, "ik_timeout", ik_timeout, 0.005);
    LoadParam(node, "ik_error", ik_error, 1e-5);
    LoadParam(node, "controller_joint_num", controller_joint_num, 6);
}

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

KDL::JntArray ConvertToKdlJntArray(std::vector<double> vec)
{
    KDL::JntArray result(vec.size());
    for (size_t i = 0; i < vec.size(); i++)
    {
        result(i) = vec.at(i);
    }
    return result;
}

/**
 * @brief 将原始关节角度转换为增加了假关节后的关节角度
 *
 * @param real_joints
 * @param joint_num_of_fake_chain fake_chain 的总关节数
 * @return KDL::JntArray
 */
KDL::JntArray ConvertToFakeJntArray(const KDL::JntArray &real_joints, unsigned int joint_num_of_fake_chain)
{
    KDL::JntArray fake_joints = real_joints;
    fake_joints.data.resize(joint_num_of_fake_chain);

    // 最后3个是假关节
    for (size_t i = joint_num_of_fake_chain - 3; i < joint_num_of_fake_chain; i++)
    {
        fake_joints.data(i) = 0;
    }
    return fake_joints;
}

KDL::JntArray ConvertToRealJntArray(const KDL::JntArray &fake_joints, const KDL::JntArray &real_joints)
{
    KDL::JntArray result = real_joints;
    auto num_of_effective_joints = real_joints.rows() - 3;
    for (size_t i = 0; i < num_of_effective_joints; i++)
    {
        result.data(i) = fake_joints.data(i);
    }
    return result;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    jnt_state_period = (current_time - last_time).toSec();
    last_time = current_time;

    // 避免长时间没收到 joint_states 导致解算出的位置变化异常大
    if (jnt_state_period > 1)
    {
        ROS_WARN_STREAM("last joint_states topic got more than 1s ago");
        return;
    }

    auto now_joint_pos = ConvertToKdlJntArray(msg->position);
    ROS_INFO_STREAM("now_joint_pos: " << now_joint_pos.data);
    auto fake_joint_pos = ConvertToFakeJntArray(now_joint_pos, fake_chain.getNrOfJoints());
    // ROS_INFO_STREAM("fake_joint_pos: " << fake_joint_pos.data);

    KDL::Frame now_ee_pos; // 当前末端的位置（解算的末端，不一定是整个机械臂的末端）
    fk_solver->JntToCart(fake_joint_pos, now_ee_pos);
    // ROS_INFO_STREAM("now_ee_pos: " << now_ee_pos.p.data[0] << " " << now_ee_pos.p.data[1] << " " << now_ee_pos.p.data[2]);

    // 计算新的末端位置
    for (size_t i = 0; i < 3; i++)
    {
        now_ee_pos.p.data[i] += velocity_pos.at(i) * jnt_state_period;
    }

    // IK
    KDL::JntArray ik_result;
    ik_solver->CartToJnt(fake_joint_pos, now_ee_pos, ik_result);

    auto result_joint_pos = ConvertToRealJntArray(ik_result, now_joint_pos);

    // 旋转
    result_joint_pos.data(3) += velocity_rpy.at(0) * jnt_state_period; // 腕关节
    result_joint_pos.data(4) += velocity_rpy.at(1) * jnt_state_period; // 夹爪关节
    result_joint_pos.data(0) += velocity_rpy.at(2) * jnt_state_period; // 基座关节

    ROS_INFO_STREAM("result_joint_pos: " << result_joint_pos.data);

    PublishJoints(pos_pub, result_joint_pos, controller_joint_num);
}

TRAC_IK::SolveType GetSolveType(std::string &solver_type, TRAC_IK::SolveType default_type = TRAC_IK::Speed)
{
    TRAC_IK::SolveType result;
    if (ik_solve_type == "Speed")
    {
        result = TRAC_IK::Speed;
    }
    else if (ik_solve_type == "Distance")
    {
        result = TRAC_IK::Distance;
    }
    else if (ik_solve_type == "Manip1")
    {
        result = TRAC_IK::Manip1;
    }
    else if (ik_solve_type == "Manip2")
    {
        result = TRAC_IK::Manip2;
    }
    else
    {
        ROS_WARN_STREAM("Invalid ik_solve_type '" << ik_solve_type << "'");
    }
    return result;
}

KDL::Chain GenerateFakeKdlChain(std::string &robot_desc_string, bool add_fake_x, bool add_fake_y, bool add_fake_z)
{
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }

    KDL::Chain result_chain;
    my_tree.getChain(chain_start, chain_end, result_chain);

    KDL::Segment my_segment_x("fake_x", KDL::Joint("fake_joint_x", KDL::Joint::RotX),
                              KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                         KDL::Vector(0.0, 0.0, 0.0)));
    KDL::Segment my_segment_y("fake_y", KDL::Joint("fake_joint_y", KDL::Joint::RotY),
                              KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                         KDL::Vector(0.0, 0.0, 0.0)));
    KDL::Segment my_segment_z("fake_z", KDL::Joint("fake_joint_z", KDL::Joint::RotZ),
                              KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                         KDL::Vector(0.0, 0.0, 0.0)));

    if (add_fake_x)
    {
        result_chain.addSegment(my_segment_x);
    }

    if (add_fake_y)
    {
        result_chain.addSegment(my_segment_y);
    }

    if (add_fake_z)
    {
        result_chain.addSegment(my_segment_z);
    }
    return result_chain;
}

void PrintChain(KDL::Chain &chain, std::string title = "Chain:")
{
    ROS_INFO_STREAM(title);
    ROS_INFO("  Number Of Joints: %d", chain.getNrOfJoints());
    ROS_INFO("  Number Of Segments: %d", chain.getNrOfSegments());
    ROS_INFO("  Segments:");
    for (auto &segment : chain.segments)
    {
        ROS_INFO_STREAM("    " << segment.getName());
        ROS_INFO_STREAM("      Joint name:" << segment.getJoint().getName() << " Type: " << segment.getJoint().getTypeName());
        ROS_INFO_STREAM("      Axes x: " << segment.getJoint().JointAxis().data[0] << "  y: " << segment.getJoint().JointAxis().data[1] << "  z: " << segment.getJoint().JointAxis().data[2]);
    }
}

void InitTracIk()
{

    if (chain_start == "" || chain_end == "")
    {
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }

    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param(urdf_param, robot_desc_string, std::string());
    fake_chain = GenerateFakeKdlChain(robot_desc_string, false, true, true);

    // 关节限位
    KDL::JntArray q_min(fake_chain.getNrOfJoints());
    KDL::JntArray q_max(fake_chain.getNrOfJoints());

    // 给假关节很大的角度限位
    for (size_t i = q_min.rows() - 3; i < q_min.rows(); i++)
    {
        q_min.data(i) = -1000;
        q_max.data(i) = 1000;
    }

    // 真关节的角度限位
    for (size_t i = 0; i < q_min.rows() - 3; i++)
    {
        q_min.data(i) = -M_PI_2;
        q_max.data(i) = M_PI_2;
    }

    // KDL FK
    fk_solver = new KDL::ChainFkSolverPos_recursive(fake_chain); // Forward kin. solver

    // Trac IK
    auto solve_type = GetSolveType(ik_solve_type, TRAC_IK::Distance);
    // % NOTE: The last arguments to the constructors are optional.
    // % The type can be one of the following:
    // % Speed: returns very quickly the first solution found
    // % Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
    // % Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
    // % Manip2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
    // ik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, ik_timeout, ik_error, solve_type);
    ik_solver = new TRAC_IK::TRAC_IK(fake_chain, q_min, q_max, ik_timeout, ik_error, solve_type);

    // 检测是否合理
    KDL::Chain chain;
    if (!ik_solver->getKDLChain(chain))
    {
        ROS_ERROR("There was no valid KDL chain found");
        exit(-1);
    }

    PrintChain(chain, "trac_ik chain:");

    KDL::JntArray ll, ul; // lower joint limits, upper joint limits
    if (!ik_solver->getKDLLimits(ll, ul))
    {
        ROS_ERROR("There were no valid KDL joint limits found");
        return;
    }

    ROS_INFO_STREAM("lower joint limits: " << ll.data);
    ROS_INFO_STREAM("upper joint limits: " << ul.data);
}

void EndEffectorVelocityCallback(const geometry_msgs::Twist &msg)
{
    velocity_pos.at(0) = msg.linear.x;
    velocity_pos.at(1) = msg.linear.y;
    velocity_pos.at(2) = msg.linear.z;
    velocity_rpy.at(0) = msg.angular.x;
    velocity_rpy.at(1) = msg.angular.y;
    velocity_rpy.at(2) = msg.angular.z;
    ROS_INFO_STREAM("Got ee velocity");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_servo");
    ros::NodeHandle node("~");
    LoadAllParam(node);

    // 获取 joint_state
    ros::Subscriber joint_state_sub = node.subscribe(joint_states_topic, 1, JointStateCallback);

    // 用于控制关节位置
    pos_pub = node.advertise<std_msgs::Float64MultiArray>(joint_group_position_controller.append("/command"), 1);

    // 由用户发布，期望的末端执行器速度
    ros::Subscriber ee_velocity_sub = node.subscribe(end_effector_velocity_topic, 1, EndEffectorVelocityCallback);

    InitTracIk();

    ros::spin();
    return 0;
}
