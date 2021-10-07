#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include"nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "Functions.h"
#include "visualization_msgs/Marker.h"

std::vector<Segment> segments;
std::vector<std::vector<double>> PointCloud;
std::vector<int> CloudIndex;
std::vector<double> robot_pose;
std::vector<double> robot_vel;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_vel.clear();
    robot_vel.push_back(msg->twist.twist.linear.x);
    robot_vel.push_back(msg->twist.twist.linear.y);
    robot_vel.push_back(msg->twist.twist.angular.z);
    ROS_INFO("Robot velocity: %f %f %f", robot_vel[0], robot_vel[1], robot_vel[2]);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //--------------------------------------Make the point cloud--------------------------------------------------------
    ROS_INFO("%s", "PointCloud");
    PointCloud.clear();
    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < float(5)) {
            double alpha = msg->angle_min + (msg->angle_increment * float(i));
            std::vector<double> point = {msg->ranges[i] * std::cos(alpha) + 0.38, -msg->ranges[i] * std::sin(alpha)};
            PointCloud.push_back(point);
            CloudIndex.push_back(i);
        }
    }
    //-------------------Turn point cloud into segments-----------------------------------------------------------------
    ROS_INFO("%s", "Segments");
    segments.clear();
    MakingLineSegments(segments, PointCloud, CloudIndex, 0, 0, 0);
    ROS_INFO("Resetting segments");
    //ResetSegmentFrame(segments, robot_pose);
    ROS_INFO("done");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_node");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/load/cmd_vel", 1);
    geometry_msgs::Twist twist_msg;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>( "segments_marker", 0 );
    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/ropod/odom", 1, odomCallback);
    ros::Subscriber sub_laser = nh.subscribe<sensor_msgs::LaserScan>("/ropod/laser/scan", 1, laserCallback);



    std::vector<double> speed = {0,0,0};
    robot_pose = {0,0,0};
    double robot_width = 0.266;
    Line area;
    Line facing;

    /* area.p1 = {4.62110, 1.15434};
    area.p2 = {9.24219, -1.15434};
    facing.p1 = {4.62110, 1.15434};
    facing.p2 = {9.24219, 1.15434}; */

    area.p1 = {5.55, -1.55};
    area.p2 = {8.05, .55};
    facing.p1 = {area.p1[0], area.p1[1]};
    facing.p2 = {area.p1[0], area.p2[1]};


    double F = 10;
    ros::Rate loop_rate(F);

    while(nh.ok()){
        ros::spinOnce();
        if (!segments.empty()) {
            //---------------------Search for the entrance------------------------------------------------------------------
            ROS_INFO("%s", "Entrance");
            std::vector<double> entrance = FindingEntrance(segments, robot_pose);
            ROS_INFO("The entrance is: (%f,%f,%f)", entrance[0], entrance[1], entrance[2]);
            //--------------------------Transform coordinates---------------------------------------------------------------
            area.p1 = TransformPositionB(area.p1, entrance);
            area.p2 = TransformPositionB(area.p2, entrance);
            facing.p1 = TransformPositionB(facing.p1, entrance);
            facing.p2 = TransformPositionB(facing.p2, entrance);
            //-------------------------Set destination----------------------------------------------------------------------
            std::vector<double> destination;
            FindAreaPose(facing, destination);
            ROS_INFO("The destination is: (%f,%f,%f)", destination[0], destination[1], destination[2]);
            //---------------------------Drive past entrance----------------------------------------------------------------
            static const std::vector<double> entrance_goal = TransformPose({7,0,0}, entrance);
            ROS_INFO("Past entrance goal is: %f %f %f", entrance_goal[0], entrance_goal[1], entrance_goal[2]);
            int i = 0;
            while (ros::ok()) {
                std::vector<double> pose_diff = FindPoseDiff3(robot_pose, entrance_goal);
                ROS_INFO("Drive to entrance: Difference is: x=%f, y=%f and theta=%f", pose_diff[0], pose_diff[1], pose_diff[2]);
                if (std::abs(pose_diff[0]) < 0.08 && std::abs(pose_diff[1]) < 0.08 && std::abs(pose_diff[2]) < 0.03*M_PI) {
                    SetTwistMessage(twist_msg, {0,0,0});
                    cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                    break;
                }
                speed = CalculateSpeed2(pose_diff, entrance_goal, robot_vel, F, robot_width, PointCloud, area);
                ROS_INFO("speed %f %f %f", speed[0], speed[1], speed[2]);
                SetTwistMessage(twist_msg, speed);
                cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));

                ros::spinOnce();
                //----------------------Visualize segments------------------------------------------------
                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "ropod/base_link";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = "points_and_lines";
                line_list.pose.orientation.w = 1.0;
                line_list.id = 2;
                line_list.color.a = 1.0;
                line_list.color.b = 1.0;
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.scale.x = 0.1;
                geometry_msgs::Point point1, point2;
                for (auto & segment : segments){
                    point1.x = segment.p1[0];
                    point1.y = -segment.p1[1];
                    point1.z = point2.z = 0;
                    point2.x = segment.p2[0];
                    point2.y = -segment.p2[1];
                    line_list.points.push_back(point1);
                    line_list.points.push_back(point2);
                }
                marker_pub.publish(line_list);

                CalculateNewRobotPose (robot_pose, robot_vel, F);
                ROS_INFO("robot pose %f %f %f", robot_pose[0], robot_pose[1], robot_pose[2]);

                loop_rate.sleep();
            }
            //---------------------------Drive to destination---------------------------------------------------------------
            while (ros::ok()) {
                std::vector<double> pose_diff = FindPoseDiff3(robot_pose, destination);
                ROS_INFO("Robot pose: (%f, %f, %f)", robot_pose[0], robot_pose[1], robot_pose[2]);
                ROS_INFO("Destination: (%f, %f, %f)", destination[0], destination[1], destination[2]);
                ROS_INFO("Drive to cart area- Difference is: x=%f, y=%f and theta=%f", pose_diff[0], pose_diff[1], pose_diff[2]);
                if (std::abs(pose_diff[0]) < 0.08 && std::abs(pose_diff[1]) < 0.08 && std::abs(pose_diff[2]) < 0.03*M_PI) {
                    SetTwistMessage(twist_msg, {0,0,0});
                    cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                    break;
                }
                speed = CalculateSpeed2(pose_diff, destination, speed, F, robot_width, PointCloud, area);
                SetTwistMessage(twist_msg, speed);
                cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));

                ros::spinOnce();
                //----------------------Visualize segments------------------------------------------------
                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "ropod/base_link";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = "points_and_lines";
                line_list.pose.orientation.w = 1.0;
                line_list.id = 2;
                line_list.color.a = 1.0;
                line_list.color.b = 1.0;
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.scale.x = 0.1;
                geometry_msgs::Point point1, point2;
                for (auto & segment : segments){
                    point1.x = segment.p1[0];
                    point1.y = -segment.p1[1];
                    point1.z = point2.z = 0;
                    point2.x = segment.p2[0];
                    point2.y = -segment.p2[1];
                    line_list.points.push_back(point1);
                    line_list.points.push_back(point2);
                }
                marker_pub.publish(line_list);

                CalculateNewRobotPose (robot_pose, robot_vel, F);

                loop_rate.sleep();
            }
            ros::shutdown();
            //-------------------------------------------Segmentation-------------------------------------------------------
            ros::spinOnce();
            //------------------------------------------Find cart-----------------------------------------------------------
            ROS_INFO("Search for cart");
            Segment cart = FindCart(segments, area, facing);
            ROS_INFO("Cart is found at (%f,%f) and (%f,%f)", cart.p1[0], cart.p1[1], cart.p2[0], cart.p2[1]);
            //------------------------------------------Find intermediate pose----------------------------------------------
            ROS_INFO("Start intermediate pose calculations");
            std::vector<double> int_pose = FindDesiredPose(cart, 0.5, robot_width+0.3);
            ROS_INFO("The intermediate pose is: (%f,%f,%f)", int_pose[0], int_pose[1], int_pose[2]);
            //------------------------------------------Find desired pose---------------------------------------------------
            ROS_INFO("Start desired pose calculations");
            std::vector<double> desired_pose = FindDesiredPose(cart, 0, robot_width+0.3);
            ROS_INFO("The desired pose is: (%f,%f,%f)", desired_pose[0], desired_pose[1], desired_pose[2]);
            //------------------------------------------Filter the segments used for localization---------------------------
            std::vector<Segment> localization_segments = FindObjects(segments, desired_pose, cart);
            //------------------------------------------Find the desired distance to the objects----------------------------
            std::vector<Distance> object_distance = FindDistance(desired_pose, robot_pose, localization_segments);

            //---------------------------Drive to intermediate pose---------------------------------------------------------
            while (ros::ok())
            {
                std::vector<double> pose_diff = FindPoseDiff(robot_pose, int_pose);
                ROS_INFO("Drive to intermediate pose- Difference is: x=%f, y=%f and theta=%f", pose_diff[0], pose_diff[1], pose_diff[2]);
                if (std::abs(pose_diff[0]) < 0.08 && std::abs(pose_diff[1]) < 0.08 && std::abs(pose_diff[2]) < 0.03*M_PI) {
                    SetTwistMessage(twist_msg, {0,0,0});
                    cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                    break;
                }
                speed = CalculateSpeed(pose_diff, speed, F, robot_width, PointCloud, area);
                SetTwistMessage(twist_msg, speed);
                cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));

                ros::spinOnce();
                //----------------------Visualize segments------------------------------------------------
                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "ropod/base_link";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = "points_and_lines";
                line_list.pose.orientation.w = 1.0;
                line_list.id = 2;
                line_list.color.a = 1.0;
                line_list.color.b = 1.0;
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.scale.x = 0.1;
                geometry_msgs::Point point1, point2;
                for (auto & segment : segments){
                    point1.x = segment.p1[0];
                    point1.y = -segment.p1[1];
                    point1.z = point2.z = 0;
                    point2.x = segment.p2[0];
                    point2.y = -segment.p2[1];
                    line_list.points.push_back(point1);
                    line_list.points.push_back(point2);
                }
                marker_pub.publish(line_list);

                CalculateNewRobotPose (robot_pose, robot_vel, F);

                loop_rate.sleep();
            }
            //---------------------------Drive to desired pose---------------------------------------------------------
            while (ros::ok()) {
                ros::spinOnce();
                std::vector<Segment> localization2_segments = CompareSegments(localization_segments, segments, robot_pose);
                std::vector<double> error = CalculateError(object_distance, localization2_segments);
                ROS_INFO("Drive to destination- Difference is: x=%f, y=%f and theta=%f", error[0], error[1], error[2]);
                if (std::abs(error[0]) < 0.02 && std::abs(error[1]) < 0.08 && std::abs(error[2]) < 0.05 * M_PI) {
                    SetTwistMessage(twist_msg, {0, 0, 0});
                    cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                    break;
                }
                speed = CalculatePositionSpeed(error, speed, F);
                SetTwistMessage(twist_msg, speed);
                cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));

                ros::spinOnce();
                //----------------------Visualize segments------------------------------------------------
                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "ropod/base_link";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = "points_and_lines";
                line_list.pose.orientation.w = 1.0;
                line_list.id = 2;
                line_list.color.a = 1.0;
                line_list.color.b = 1.0;
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.scale.x = 0.1;
                geometry_msgs::Point point1, point2;
                for (auto & segment : segments){
                    point1.x = segment.p1[0];
                    point1.y = -segment.p1[1];
                    point1.z = point2.z = 0;
                    point2.x = segment.p2[0];
                    point2.y = -segment.p2[1];
                    line_list.points.push_back(point1);
                    line_list.points.push_back(point2);
                }
                marker_pub.publish(line_list);

                CalculateNewRobotPose (robot_pose, robot_vel, F);

                loop_rate.sleep();
            }
        }


/* // docking
docking_pub.publish(ropod_ros_msgs::DockingCommand::DOCKING_COMMAND_DOCK);*/

    }
    return 0;
}