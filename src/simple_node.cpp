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
            std::vector<double> point = {msg->ranges[i] * std::cos(alpha) + 0.24, msg->ranges[i] * std::sin(alpha)};
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

    double F = 10;
    ros::Rate loop_rate(F);

    std::vector<double> speed = {0,0,0};
    robot_pose = {0,0,0};
    double robot_width = 0.266;
    Line area;
    area.p1 = {5.55, -0.75};
    area.p2 = {8.05, 1.35};
    Line facing;
    facing.p1 = {area.p1[0], area.p1[1]};
    facing.p2 = {area.p1[0], area.p2[1]};
    double amax = 1.05;
    double awmax = (3*M_PI)/68;

    std::vector<double> drive_goal;
    std::vector<double> area_pose;
    std::vector<Segment> object_distance;
    std::vector<Segment> localization_segments;
    std::vector<double> desired_pose;
    std::vector<double> int_pose;
    std::vector<double> entrance;
    Segment cart;
    std::vector<double> pose_diff;
    std::vector<double> indices;
    double i;
    bool driving;
    std::vector<double> init_error;
    std::vector<Segment2> localization2_segments;

    typedef enum {
        searchEntrance = 1,
        Stop,
        Drive,
        searchCart,
        driveAccurate,
    } state_t;
    state_t state = searchEntrance;
    int n = 0;


    while(nh.ok()){
        ros::spinOnce();
        if (!segments.empty()) {
            switch(state) {
                //---------------------Search for the entrance----------------------------------------------------------
                case searchEntrance:
                    ROS_INFO("%s", "Entrance");
                    entrance = FindEntrance(segments, robot_pose);
                    static const std::vector<double> entrance_goal = TransformPose({2, 0, 0}, entrance);
                    if (entrance[0] == 0 && entrance[1] == 0 && entrance[2] == 0) {
                        state = Stop;
                        break;
                    } else {
                        drive_goal = entrance_goal;
                        state = Drive;
                    }
                    ROS_INFO("The entrance is: (%f,%f,%f)", entrance[0], entrance[1], entrance[2]);
                    ROS_INFO("Past entrance goal is: %f %f %f", entrance_goal[0], entrance_goal[1], entrance_goal[2]);
                    area.p1 = TransformPosition(area.p1, entrance);
                    area.p2 = TransformPosition(area.p2, entrance);
                    facing.p1 = TransformPosition(facing.p1, entrance);
                    facing.p2 = TransformPosition(facing.p2, entrance);
                    FindAreaPose(facing, area_pose);
                    ROS_INFO("The destination is: (%f,%f,%f)", area_pose[0], area_pose[1], area_pose[2]);
                    break;
                //------------------Stop--------------------------------------------------------------------------------
                case Stop:
                    ROS_INFO("Stop the process.");
                    ros::shutdown();
                    break;
                //------------------Drive to goal: open loop------------------------------------------------------------
                case Drive:
                    ROS_INFO("Drive to next destination.");
                    pose_diff = PoseDifference(robot_pose, drive_goal);
                    ROS_INFO("Pose diff is (%f, %f, %f)", pose_diff[0], pose_diff[1], pose_diff[2]);
                    indices = CalculateIndices(pose_diff, amax, awmax, F);
                    ROS_INFO("indices %f %f %f", indices[0], indices[1], indices[2]);
                    i = 0;
                    driving = true;
                    n += 1;
                    while (ros::ok()) {
                        ROS_INFO("int %f", i);
                        if (!driving) {
                            SetTwistMessage2(twist_msg, {0,0,0}, pose_diff);
                            cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                            if( n == 1 ) {
                                ROS_INFO("New driving destination is set to be the area of the cart.");
                                drive_goal = area_pose;
                                state = Drive;
                            } else if ( n == 2 ) {
                                state = searchCart;
                            } else if ( n == 3 ) {
                                ROS_INFO("New driving destination is set to be the final desired pose before docking.");
                                drive_goal = desired_pose;
                                state = driveAccurate;
                            } else {
                                state = Stop;
                            }
                            break;
                        }
                        speed = VelocityOL(robot_vel, F, robot_width, PointCloud, area, i, indices[0], indices[1], indices[2], driving);
                        ROS_INFO("speed %f %f %f", speed[0], speed[1], speed[2]);
                        SetTwistMessage2(twist_msg, speed, pose_diff);
                        cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));

                        ros::spinOnce();
                        //----------------------Visualize segments------------------------------------------------------
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
                            point1.y = segment.p1[1];
                            point1.z = point2.z = 0;
                            point2.x = segment.p2[0];
                            point2.y = segment.p2[1];
                            line_list.points.push_back(point1);
                            line_list.points.push_back(point2);
                        }
                        marker_pub.publish(line_list);
                        //----------------------------------------------------------------------------------------------
                        CalculateNewRobotPose (robot_pose, robot_vel, F);
                        ROS_INFO("robot pose %f %f %f", robot_pose[0], robot_pose[1], robot_pose[2]);

                        loop_rate.sleep();
                    }
                    break;
                //---------------------------Search for the cart--------------------------------------------------------
                case searchCart:
                    ros::spinOnce();
                    ResetSegmentFrame(segments, robot_pose);
                    ROS_INFO("Search for cart");
                    cart = FindCart(segments, area, facing);
                    if (cart.p1[0] == 0 && cart.p1[1] == 0 && cart.p2[0] == 0 && cart.p2[1] == 0) {
                        state = Stop;
                        break;
                    }
                    ROS_INFO("Cart is found at (%f,%f) and (%f,%f)", cart.p1[0], cart.p1[1], cart.p2[0], cart.p2[1]);
                    int_pose = FindDesiredPose(cart, 0.5, robot_width + 0.6);
                    ROS_INFO("The intermediate pose is: (%f,%f,%f)", int_pose[0], int_pose[1], int_pose[2]);
                    desired_pose = FindDesiredPose(cart, 0, robot_width + 0.6);
                    ROS_INFO("The desired pose is: (%f,%f,%f)", desired_pose[0], desired_pose[1], desired_pose[2]);
                    localization_segments = FindObjects(segments, desired_pose, cart);
                    if (localization_segments.empty()) {
                        ROS_INFO("No useful surrounding object were identified.");
                        state = Stop;
                        break;
                    } else {
                        drive_goal = int_pose;
                        state = Drive;
                    }
                    object_distance = FindDistance(desired_pose, robot_pose, localization_segments);
                    break;
                //------------------------------Driving to goal: closed loop--------------------------------------------
                case driveAccurate:
                    ROS_INFO("Drive to the final desired pose before docking.");
                    init_error = CalculateInitError(object_distance, localization_segments, robot_pose);
                    while (ros::ok()) {
                        ros::spinOnce();
                        ResetSegmentFrame(segments, robot_pose);
                        localization2_segments = CompareSegments(localization_segments, segments);
                        ROS_INFO("Past segment comparison");
                        if (localization2_segments.empty()) {
                            ROS_INFO("No surrounding object recognised.");
                            SetTwistMessage(twist_msg, {0, 0, 0});
                            cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                            state = Stop;
                            break;
                        }
                        ROS_INFO("Start error calculation");
                        std::vector<double> error = CalculateError(object_distance, localization2_segments, robot_pose);
                        ROS_INFO("Drive to destination- Difference is: x=%f, y=%f and theta=%f", error[0], error[1], error[2]);
                        if (std::abs(error[0]) < 0.08 && std::abs(error[1]) < 0.02 && std::abs(error[2]) < 0.05 * M_PI) {
                            SetTwistMessage(twist_msg, {0, 0, 0});
                            cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));
                            state = Stop;
                            break;
                        }
                        speed = VelocityCL(error, robot_vel, F, error);
                        SetTwistMessage(twist_msg, speed);
                        cmd_vel_pub.publish(geometry_msgs::Twist(twist_msg));

                        CalculateNewRobotPose (robot_pose, robot_vel, F);

                        loop_rate.sleep();
                    }
                default:
                    ROS_INFO("Something went wrong");
                    ros::shutdown();
                    break;
            }
        }
/* // docking
docking_pub.publish(ropod_ros_msgs::DockingCommand::DOCKING_COMMAND_DOCK);*/

    }
    return 0;
}