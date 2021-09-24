#ifndef DANIQUE_FUNCTIONS_H
#define DANIQUE_FUNCTIONS_H
#include "Classes.h"

//Segmentation
std::vector<std::vector<double>> MakingPointCloud(sensor_msgs::LaserScan& laser, const std::vector<double>& robot_pose);
std::vector<Segment> MakingLineSegments(std::vector<std::vector<double>> pointcloud, int i, int n);
//Mathematic tools
double RotationDifference(std::vector<double> p1, std::vector<double> p2);
std::vector<double> TransformPosition(std::vector<double> old_point, std::vector<double> rel_pose);
std::vector<double> FindPoseDiff(std::vector<double> robot_pose, std::vector<double> destination);

std::vector<double> FindingEntrance(sensor_msgs::LaserScan& laser, const std::vector<double>& robot_pose);
std::vector<double> FindAreaPose(const std::vector<double>& entrance, const Line& area, Line facing);
//Driving
bool Wait(std::vector<std::vector<double>> pointcloud, double b, std::vector<double> pose_diff);
std::vector<double> CalculateSpeed(std::vector<double> pose_diff, std::vector<double> old_speed, double F, sensor_msgs::LaserScan& laser, double b);

std::vector<Segment> CompareSegments(std::vector<Segment> old_segments, std::vector<Segment> new_segments, const std::vector<double>& robot_pose);
std::vector<Segment> CertaintyFilter(std::vector<Segment> segments, double amount, const Segment& cart);
int FindHighestCertainty(std::vector<Segment> segments, Segment cart);
Segment FindCart(std::vector<Segment> segments, Line area, Line facing);
std::vector<Distance> FindDistance(const std::vector<double>& desired_pose, const std::vector<Segment>& certainty_segments);
std::vector<double> FindDesiredPose(Segment cart_segment, double range_x, double range_y);
std::vector<double> FindAreaPose(Line facing);
std::vector<double> CalculateError(std::vector<Distance> distances, std::vector<Segment> localization_segments);


#endif //DANIQUE_FUNCTIONS_H
