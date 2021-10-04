#ifndef DANIQUE_FUNCTIONS_H
#define DANIQUE_FUNCTIONS_H
#include "Classes.h"

//Segmentation
void MakingLineSegments(std::vector<Segment>& segments, std::vector<std::vector<double>> pointcloud, int i, int n, double lambda_old);

//Mathematic tools
double RotationDifference(std::vector<double> p1, std::vector<double> p2);
std::vector<double> TransformPositionB(std::vector<double> old_point, std::vector<double> rel_pose);
std::vector<double> FindPoseDiff(std::vector<double> robot_pose, std::vector<double> destination);
void ResetSegmentFrame(std::vector<Segment>& segments, std::vector<double> robot_pose);
std::vector<Distance> FindDistance(const std::vector<double>& desired_pose, const std::vector<double>& robot_pose, const std::vector<Segment>& certainty_segments);
std::vector<double> FindDesiredPose(Segment cart_segment, double range_x, double range_y);
std::vector<double> CalculateError(std::vector<Distance> distances, std::vector<Segment> localization_segments);

//Driving
bool Wait(std::vector<std::vector<double>> pointcloud, double b, std::vector<double> pose_diff);
std::vector<double> CalculateSpeed(std::vector<double> pose_diff, std::vector<double> old_speed, double F, double b, std::vector<std::vector<double>>& pointcloud);
std::vector<double> CalculatePositionSpeed(std::vector<double> pose_diff, std::vector<double> old_speed, double F);
void SetTwistMessage(geometry_msgs::Twist& twist_msg, std::vector<double> speed);

//EnvironmentOrientation
std::vector<Segment> CompareSegments(std::vector<Segment> old_segments, std::vector<Segment> new_segments, const std::vector<double>& robot_pose);
std::vector<Segment> FindObjects(std::vector<Segment>& segments, const std::vector<double>& desired_pose, const Segment& cart);
Segment FindCart(std::vector<Segment>& segments, Line area, Line facing);
std::vector<double> FindingEntrance(std::vector<Segment> segments, const std::vector<double>& robot_pose);
void FindAreaPose(Line facing, std::vector<double>& destination);

#endif //DANIQUE_FUNCTIONS_H
