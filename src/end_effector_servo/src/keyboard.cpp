/* void TargetSetter(ros::NodeHandle *node, std::string start_chain, std::string end_chain, std::string rot_link_name)
{
    KeyboardAsync keyboard;

    tf2_ros::TransformListener tfListener();
    geometry_msgs::TransformStamped rot_link;
    geometry_msgs::TransformStamped rot_link_to_ee_link;

    ros::ServiceClient client_close = node->serviceClient<std_srvs::Empty>("/clamp/close");
    ros::ServiceClient client_open = node->serviceClient<std_srvs::Empty>("/clamp/open");
    ros::ServiceClient client_stop = node->serviceClient<std_srvs::Empty>("/clamp/stop");
    std_srvs::Empty srv;

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

        // 变换矩阵运算
        temp_pos.M = temp_pos.M * KDL::Rotation::RPY(velocity_rpy.at(0) * period, velocity_rpy.at(1) * period, velocity_rpy.at(2) * period);
        temp_pos = temp_pos * ConvertToKdlFrame(rot_link_to_ee_link.transform);
        ExpEEPos.set(temp_pos);
    };

    ROS_INFO_STREAM("You can use w,s,a,d to move");
    ROS_INFO_STREAM("You can use i,k,j,l to rotate");
    ROS_INFO_STREAM("You can use o,p,[ to open,close,stop the clamp");

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
            velocity_pos.at(1) = 0.2;
            UpdateExpEEPos();
            break;
        case 's':
            velocity_pos.at(1) = -0.2;
            UpdateExpEEPos();
            break;
        case 'a':
            velocity_pos.at(0) = -0.2;
            UpdateExpEEPos();
            break;
        case 'd':
            velocity_pos.at(0) = 0.2;
            UpdateExpEEPos();
            break;
        case 'r':
            velocity_pos.at(2) = 0.2;
            UpdateExpEEPos();
            break;
        case 'f':
            velocity_pos.at(2) = -0.2;
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
        case 'o':
            client_open.call(srv);
            break;
        case 'p':
            client_close.call(srv);
            break;
        case '[':
            client_stop.call(srv);
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
 */