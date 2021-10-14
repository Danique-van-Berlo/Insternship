#ifndef DANIQUE_FUNCTIONS_H
#define DANIQUE_FUNCTIONS_H
#include "Classes.h"

// Functions for segmentation
void MakingLineSegments(std::vector<Segment>& segments, std::vector<std::vector<double>> pointcloud, std::vector<int> index, int i, int n, double lambda_old);
void ResetSegmentFrame(std::vector<Segment>& segments, std::vector<double>& robot_pose);
// Functions for driving
bool Wait(std::vector<std::vector<double>> pointcloud, double b, Line area);
std::vector<double> CalculateIndices(std::vector<double> pose_diff, double amax, double awmax, double F);
std::vector<double> VelocityOL(std::vector<double> old_speed, double f, double b, std::vector<std::vector<double>>& pointcloud, Line area, double& i, double n, double m, double k, bool& driving);
std::vector<double> VelocityCL(std::vector<double> pose_diff, std::vector<double> old_speed, double F, std::vector<double> error);
std::vector<double> VelocityCL2(std::vector<double> pose_diff, std::vector<double> old_speed, double f, std::vector<double> error);
void SetTwistMessage2(geometry_msgs::Twist& twist_msg, std::vector<double> speed, std::vector<double> pose_diff);
void SetTwistMessage(geometry_msgs::Twist& twist_msg, std::vector<double> speed);
// Functions for orientation
std::vector<double> FindEntrance(std::vector<Segment>& segments, const std::vector<double>& robot_pose);
void FindAreaPose(Line& facing, std::vector<double>& destination);
Segment FindCart(std::vector<Segment>& segments, Line area, Line facing);
std::vector<double> FindDesiredPose(Segment cart_segment, double range_x, double range_y);
std::vector<Segment> FindObjects(std::vector<Segment>& segments, const std::vector<double>& desired_pose, const Segment& cart);
// Additional mathematical tools
double AngularDifference(std::vector<double> dv);
std::vector<double> TransformPosition(std::vector<double> old_point, std::vector<double> rel_pose);
std::vector<double> TransformPose(std::vector<double> old_point, std::vector<double> rel_pose);
std::vector<double> PoseDifference(std::vector<double> robot_pose, const std::vector<double>& destination);
void CalculateNewRobotPose (std::vector<double>& pose, std::vector<double>& vel, double F);
std::vector<Segment> FindDistance(const std::vector<double>& desired_pose, const std::vector<double>& robot_pose, const std::vector<Segment>& certainty_segments);
std::vector<Segment2> CompareSegments(std::vector<Segment>& old_segments, std::vector<Segment>& new_segments);
std::vector<double> CalculateError(std::vector<Segment>& distances, std::vector<Segment2>& localization_segments, std::vector<double>& robot_pose);
std::vector<double> CalculateInitError(std::vector<Segment>& distances, std::vector<Segment>& localization_segments, std::vector<double>& robot_pose);
#endif //DANIQUE_FUNCTIONS_H
