#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include"nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "Functions.h"

std::vector<Segment> segments;
geometry_msgs::Twist twist_msg;
std::vector<std::vector<double>> PointCloud;
sensor_msgs::LaserScan laser;
bool publish = false;
nav_msgs::Odometry odom;
std::vector<double> speed = {0,0,0};
std::vector<double> robot_pose = {0,0,0};
double robot_width = 0.266;
Line area;
Line facing;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //--------------------------------------Make the point cloud--------------------------------------------------------
    ROS_INFO("%s", "PointCloud");
    PointCloud.clear();
    int k = 0;
    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < float(5)) {
            k += 1;
            double alpha = msg->angle_min + (msg->angle_increment * float(i));
            std::vector<double> point = {msg->ranges[i] * std::cos(alpha) + 0.38, -msg->ranges[i] * std::sin(alpha)};
            PointCloud.push_back(point);
            //ROS_INFO("point %i: (%f, %f) with range %f and angle %f", k, point[0], point[1], msg->ranges[i], alpha);
        }
    }
    //-------------------Turn point cloud into segments-----------------------------------------------------------------
    ROS_INFO("%s", "Segments");
    segments.clear();
    MakingLineSegments(segments, PointCloud, 0, 0, 0);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_node");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/load/cmd_vel", 1);
    ros::Subscriber sub_laser = nh.subscribe<sensor_msgs::LaserScan>("/ropod/laser/scan", 1, laserCallback);


    /* area.p1 = {4.62110, 1.15434};
    area.p2 = {9.24219, -1.15434};
    facing.p1 = {4.62110, 1.15434};
    facing.p2 = {9.24219, 1.15434}; */

    area.p1 = {4.55, -1.05};
    area.p2 = {7.05, 1.05};
    facing.p1 = {area.p1[0], area.p1[1]};
    facing.p2 = {area.p1[0], area.p2[1]};


    double F = 10;
    ros::Rate loop_rate(F);


    while(nh.ok()){
        ros::spinOnce();
        //---------------------Search for the entrance----------------------------------------------------------------------
        ROS_INFO("%s", "Entrance");
        std::vector<double> entrance = FindingEntrance(segments, {0, 0, 0});
        ROS_INFO("The entrance is: (%f,%f,%f)", entrance[0], entrance[1], entrance[2]);
        //--------------------------Transform coordinates-------------------------------------------------------------------
        ROS_INFO("Area point one: (%f,%f)", area.p1[0], area.p1[1]);
        area.p1 = TransformPositionB(area.p1, entrance);
        ROS_INFO("Area point one new: (%f,%f)", area.p1[0], area.p1[1]);
        area.p2 = TransformPositionB(area.p2, entrance);
        facing.p1 = TransformPositionB(facing.p1, entrance);
        facing.p2 = TransformPositionB(facing.p2, entrance);
        //-------------------------Set destination--------------------------------------------------------------------------
        std::vector<double> destination;
        FindAreaPose(facing, destination);
        ROS_INFO("The destination is: (%f,%f,%f)", destination[0], destination[1], destination[2]);
        //---------------------------Drive to entrance----------------------------------------------------------------------
        while (ros::ok())
        {
            ROS_INFO("Find the difference between the current pose and destination");
            std::vector<double> pose_diff = FindPoseDiff(robot_pose, entrance);
            ROS_INFO("Difference is: x=%f, y=%f and theta=%f", pose_diff[0], pose_diff[1], pose_diff[2]);
            if (std::abs(pose_diff[0]) < 0.02 && std::abs(pose_diff[1]) < 0.08 && std::abs(pose_diff[2]) < 0.05*M_PI) {
                twist_msg.linear.x = 0;
                twist_msg.linear.y = 0;
                twist_msg.angular.z = 0;
                cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                break;
            }
            ROS_INFO("Calculate the speed");
            speed = CalculateSpeed(pose_diff, speed, F, laser, robot_width, PointCloud);
            twist_msg.linear.x = speed[0];
            twist_msg.linear.y = speed[1];
            twist_msg.angular.z = speed[2];
            ROS_INFO("Publish the speed");
            cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
            ROS_INFO("Recalculate robot pose");
            robot_pose = {robot_pose[0]+speed[0]/F, robot_pose[1]+speed[1]/F, robot_pose[2]+speed[2]/F};

            loop_rate.sleep();
        }
        //---------------------------Drive to destination-------------------------------------------------------------------
        while (ros::ok())
        {
            ROS_INFO("Find the difference between the current pose and destination");
            std::vector<double> pose_diff = FindPoseDiff(robot_pose, destination);
            ROS_INFO("Difference is: x=%f, y=%f and theta=%f", pose_diff[0], pose_diff[1], pose_diff[2]);
            if (std::abs(pose_diff[0]) < 0.02 && std::abs(pose_diff[1]) < 0.08 && std::abs(pose_diff[2]) < 0.05*M_PI) {
                twist_msg.linear.x = 0;
                twist_msg.linear.y = 0;
                twist_msg.angular.z = 0;
                cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                break;
            }
            ROS_INFO("Calculate the speed");
            speed = CalculateSpeed(pose_diff, speed, F, laser, robot_width, PointCloud);
            ROS_INFO("Given speed: x: %f, y: %f, theta: %f", speed[0], speed[1], speed[2]);
            twist_msg.linear.x = speed[0];
            twist_msg.linear.y = speed[1];
            twist_msg.angular.z = speed[2];
            ROS_INFO("Publish the speed");
            cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
            ROS_INFO("Recalculate robot pose");
            robot_pose = {robot_pose[0]+speed[0]/F, robot_pose[1]+speed[1]/F, robot_pose[2]+speed[2]/F};

            loop_rate.sleep();
        }
        //-------------------------------------------Segmentation-----------------------------------------------------------
        //ros::spinOnce();
        //------------------------------------------Find cart---------------------------------------------------------------
        //Segment cart = FindCart(segments, area, facing);
        //------------------------------------------Find intermediate pose--------------------------------------------------
        //std::vector<double> int_pose = FindDesiredPose(cart, robot_width+0.5, robot_width+0.1);
        //------------------------------------------Find desired pose-------------------------------------------------------
        //std::vector<double> desired_pose = FindDesiredPose(cart, 0, robot_width+0.1);
        //------------------------------------------Filter the segments used for localization-------------------------------
        //std::vector<Segment> localization_segments = CertaintyFilter(segments, 5, cart);
        //------------------------------------------Find the desired distance to the objects--------------------------------
        //std::vector<Distance> object_distance = FindDistance(desired_pose, localization_segments);
    }
    return 0;
}