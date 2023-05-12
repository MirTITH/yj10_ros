#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <termio.h>
#include <stdio.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <std_srvs/Empty.h>

int scanKeyboard()
{
    //  struct termios
    //    {
    //      tcflag_t c_iflag;		/* input mode flags */
    //      tcflag_t c_oflag;		/* output mode flags */
    //      tcflag_t c_cflag;		/* control mode flags */
    //      tcflag_t c_lflag;		/* local mode flags */
    //      cc_t c_line;			/* line discipline */
    //      cc_t c_cc[NCCS];		/* control characters */
    //      speed_t c_ispeed;		/* input speed */
    //      speed_t c_ospeed;		/* output speed */
    //  #define _HAVE_STRUCT_TERMIOS_C_ISPEED 1
    //  #define _HAVE_STRUCT_TERMIOS_C_OSPEED 1
    //    };
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(STDIN_FILENO, &stored_settings); // 获得stdin 输入
    new_settings = stored_settings;            //
    new_settings.c_lflag &= (~ICANON);         //
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO, &stored_settings); // 获得stdin 输入
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings); //

    in = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
    return in;
}

/* void ForwardKinematics(moveit::planning_interface::MoveGroupInterface &arm)
{
    // 设置机械臂运动的允许误差
    arm.setGoalJointTolerance(0.001);
    // 设置允许的最大速度和加速度
    // arm.setMaxAccelerationScalingFactor(0.5);
    // arm.setMaxVelocityScalingFactor(0.4);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("fold");
    arm.move(); // 规划+运动
    sleep(1);

    // 定义一个数组，存放6个关节的信息
    std::vector<double> joint_group_positions = {0.391410, -0.676384, -0.376217, 0.0, 1.052834};

    // 将关节值写入
    arm.setJointValueTarget(joint_group_positions);
    arm.move(); // 规划+移动
    sleep(1);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("origin");
    arm.move();
    sleep(1);
}
 */
/* void InverseKinematics(moveit::planning_interface::MoveGroupInterface &arm)
{
    // 设置目标位置所使用的参考坐标系
    arm.setPoseReferenceFrame("base_link");

    // 当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);

    // 设置允许的最大速度和加速度
    // arm.setMaxVelocityScalingFactor(0.4);

    auto targetPose = arm.getCurrentPose();

    ROS_INFO_STREAM(targetPose.pose);

    while (ros::ok())
    {
        targetPose.pose.position.z -= 0.2;

        // 设置机器臂当前的状态作为运动初始状态
        arm.setStartStateToCurrentState();
        // 将目标位姿写入
        arm.setApproximateJointValueTarget(targetPose.pose);

        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto success = arm.plan(plan);

        // 输出成功与否的信息
        ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");

        // 让机械臂按照规划的轨迹开始运动
        if (success)
            arm.execute(plan);

        targetPose.pose.position.z += 0.2;

        // 设置机器臂当前的状态作为运动初始状态
        arm.setStartStateToCurrentState();
        // 将目标位姿写入
        arm.setApproximateJointValueTarget(targetPose.pose);

        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        success = arm.plan(plan);

        // 输出成功与否的信息
        ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");

        // 让机械臂按照规划的轨迹开始运动
        if (success)
            arm.execute(plan);
    }

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("fold");
    arm.move();
    sleep(1);
} */

void Cartesian(moveit::planning_interface::MoveGroupInterface &arm)
{
    // 获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    // 设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    // 当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);

    // 设置允许的最大速度和加速度
    arm.setMaxVelocityScalingFactor(0.4);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("origin");
    arm.move(); // 规划+移动
    sleep(1);   // 停1s

    // 获取当前位姿数据最为机械臂运动的起始位姿
    geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;

    // 初始化路径点向量
    std::vector<geometry_msgs::Pose> waypoints;

    // 将初始位姿加入路点列表
    waypoints.push_back(start_pose);

    start_pose.position.z -= 0.2;
    waypoints.push_back(start_pose);

    start_pose.position.x += 0.1;
    waypoints.push_back(start_pose);

    start_pose.position.y += 0.1;
    waypoints.push_back(start_pose);

    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;
    int maxtries = 100; // 最大尝试规划次数
    int attempts = 0;   // 已经尝试规划次数

    while (fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;

        if (attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if (fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");

        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        // 执行运动
        arm.execute(plan);
        sleep(1);
    }
    else
    {
        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm.execute(plan);
        sleep(1);
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("origin");
    arm.move();
    sleep(1);
}

std::mutex Mutex_;

void ExecuteThread(moveit::planning_interface::MoveGroupInterface *arm, geometry_msgs::Pose *eef_pose)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::Pose lastPose, nowPose;

    {
        std::lock_guard<std::mutex> lockGuard(Mutex_);
        lastPose = *eef_pose;
    }

    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lockGuard(Mutex_);
            nowPose = *eef_pose;
        }

        if (nowPose != lastPose)
        {
            arm->setStartStateToCurrentState();

            // 将目标位姿写入
            arm->setApproximateJointValueTarget(nowPose);
            // arm->
            // arm->setPositionTarget(nowPose.position.x, nowPose.position.y, nowPose.position.z);

            // 规划，并判断是否规划成功
            if (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                // 执行，并判断是否执行成功
                if (arm->asyncExecute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    // 只有成功后才更新位置
                    lastPose = nowPose;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int main(int argc, char **argv) // 主函数
{
    // ros初始化节点
    ros::init(argc, argv, "yj10_moveit_group");
    ros::NodeHandle nh;
    // 多线程
    ros::AsyncSpinner spinner(1);
    // 开启新的线程
    spinner.start();

    ros::ServiceClient client_close = nh.serviceClient<std_srvs::Empty>("clamp/close");
    ros::ServiceClient client_open = nh.serviceClient<std_srvs::Empty>("clamp/open");
    ros::ServiceClient client_stop = nh.serviceClient<std_srvs::Empty>("clamp/stop");

    std_srvs::Empty srv;

    // 初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface arm("arm_group");

    // 设置目标位置所使用的参考坐标系
    arm.setPoseReferenceFrame("base_link");

    // 当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.005);

    // 设置允许的最大速度和加速度
    // arm.setMaxVelocityScalingFactor(0.4);

    ROS_INFO_STREAM("Return to 'down' position.");
    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("down");
    arm.move();

    auto targetPose = arm.getCurrentPose().pose;

    std::thread execute_thread(ExecuteThread, &arm, &targetPose);
    ROS_INFO_STREAM("You can use w,s,a,d,i,k to control the end now.");
    ROS_INFO_STREAM("You can use o to open the clamper.");
    ROS_INFO_STREAM("You can use c to close the clamper.");
    ROS_INFO_STREAM("You can use p to stop the clamper.");

    while (ros::ok())
    {
        auto key = scanKeyboard();
        std::lock_guard<std::mutex> lockGuard(Mutex_);
        switch (key)
        {
        case 'w': // w
            targetPose.position.y += 0.05;
            break;
        case 's': // s
            targetPose.position.y -= 0.05;
            break;
        case 'a': // a
            targetPose.position.x -= 0.05;
            break;
        case 'd': // d
            targetPose.position.x += 0.05;
            break;
        case 'k':
            targetPose.position.z -= 0.02;
            break;
        case 'i':
            targetPose.position.z += 0.02;
            break;
        case 'o':
            client_open.call(srv);
            break;
        case 'c':
            client_close.call(srv);
            break;
        case 'p':
            client_stop.call(srv);
            break;

        default:
            break;
        }
        ROS_INFO_STREAM(targetPose);
    }

    execute_thread.join();

    return 0;
}