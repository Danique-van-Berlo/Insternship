#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include"nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "Functions.h"
#include "ropod_ros_msgs/DockingCommand.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_node");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    geometry_msgs::Twist twist_msg;
    ros::Publisher docking_pub = nh.advertise<ropod_ros_msgs::DockingCommand>("docking", 1);
    double F = 10;

    ros::Rate loop_rate(F);

    std::vector<std::vector<double>> PointCloud;
    std::vector<Segment> segments;
    sensor_msgs::LaserScan laser;
    nav_msgs::Odometry odom;
    std::vector<double> speed = {0,0,0};
    std::vector<double> robot_pose = {0,0,0};
    double robot_width = 0.266;
    double robot_front = 0.4686;
    double robot_back = -0.4565;
    double sensor_dist = 0.3975;
    Line area;
    Line facing;

    area.p1 = {4.62110, 1.15434};
    area.p2 = {9.24219, -1.15434};
    area.p1 = {4.62110, 1.15434};
    area.p2 = {9.24219, 1.15434};

    ROS_INFO("%s", "Start code" );

    /*std::cout << " What is the area of the cart w.r.t. the entrance?: ";
    std::cin >> area; */

    /* Initialize: translate the information of the cart w.r.t. the entrance to w.r.t. start pose of the robot. */
    std::vector<double> entrance = FindingEntrance(laser, robot_pose);
    ROS_INFO("%s", "Found entrance" );
    area.p1 = TransformPosition(area.p1, entrance);
    area.p2 = TransformPosition(area.p2, entrance);
    facing.p1 = TransformPosition(facing.p1, entrance);
    facing.p2 = TransformPosition(facing.p2, entrance);
    std::vector<double> destination = FindAreaPose(facing);
    ROS_INFO("%s", "Found destination" );

    /* Driving: destination is set to be the area of the cart, and the robot drives up there */
    while (ros::ok())
    {
        std::vector<double> pose_diff = FindPoseDiff(robot_pose, entrance);
        if (std::abs(pose_diff[0]) < 0.02 && std::abs(pose_diff[1]) < 0.08 && std::abs(pose_diff[2]) < 0.05*M_PI) {
            break;
        }
        speed = CalculateSpeed(pose_diff, speed, F, laser, robot_width);
        twist_msg.linear.x = speed[0];
        twist_msg.linear.y = speed[1];
        twist_msg.angular.y = speed[2];
        cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
        robot_pose = {robot_pose[0]+speed[0]/F, robot_pose[1]+speed[1]/F, robot_pose[2]+speed[2]/F};

        loop_rate.sleep();
    }

    /* Making sure the robot does not move anymore */
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.angular.y = 0;
    cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));

    /* Finding the cart: the LiDAR data of the robot is used to find the important segment of the cart.
     * Aftwerwards, the intermediate position and the desired position are calculated.
     * Finally, the desired distance to each realiable/useful object segment is calculated */
    PointCloud = MakingPointCloud(laser, robot_pose);
    segments = MakingLineSegments(PointCloud, 0, 0);
    Segment cart = FindCart(segments, area, facing);
    std::vector<double> int_pose = FindDesiredPose(cart, robot_width+0.5, robot_width+0.1);
    std::vector<double> desired_pose = FindDesiredPose(cart, 0, robot_width+0.1);
    std::vector<Segment> localization_segments = CertaintyFilter(segments, 5, cart);
    std::vector<Distance> object_distance = FindDistance(desired_pose, localization_segments);

    /* Driving: the destination is set to be the intermediate pose, and the robot drives up there. */
    while (ros::ok())
    {
        std::vector<double> pose_diff = FindPoseDiff(robot_pose, int_pose);
        if (std::abs(pose_diff[0]) < 0.02 && std::abs(pose_diff[1]) < 0.08 && std::abs(pose_diff[2]) < 0.05*M_PI) {
            break;
        }
        speed = CalculateSpeed(pose_diff, speed, F, laser, robot_width);
        twist_msg.linear.x = speed[0];
        twist_msg.linear.y = speed[1];
        twist_msg.angular.y = speed[2];
        cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
        robot_pose = {robot_pose[0]+speed[0]/F, robot_pose[1]+speed[1]/F, robot_pose[2]+speed[2]/F};

        loop_rate.sleep();
    }

    /* Making sure the robot does not move anymore */
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.angular.y = 0;
    cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));


    /* Driving: the destination is set to be the desired pose, and the robot drives up there.
     * However, it does on the base of checking its distance to nearby objects */
    while (ros::ok())
    {
        PointCloud = MakingPointCloud(laser, {0,0,0});
        segments = MakingLineSegments(PointCloud, 0, 0);
        std::vector<Segment> localization2_segments = CompareSegments(localization_segments, segments, robot_pose);
        std::vector<double> error = CalculateError(object_distance, localization2_segments);

        if (std::abs(error[0]) < 0.02 && std::abs(error[1]) < 0.08 && std::abs(error[2]) < 0.05*M_PI) {
            break;
        }
        speed = CalculateSpeed(error, speed, F, laser, robot_width);
        twist_msg.linear.x = speed[0];
        twist_msg.linear.y = speed[1];
        twist_msg.angular.y = speed[2];
        cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
        robot_pose = {robot_pose[0]+speed[0]/F, robot_pose[1]+speed[1]/F, robot_pose[2]+speed[2]/F};

        loop_rate.sleep();
    }

    /* Making sure the robot does not move anymore */
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.angular.y = 0;
    cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));


    // docking
    docking_pub.publish(ropod_ros_msgs::DockingCommand::DOCKING_COMMAND_DOCK);

    return 0;
}