#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "Functions.h"
#include <utility>
#include "Classes.h"

//----------------------------------------------------------------------------------------------------------------------
// Functions for segmentation
//----------------------------------------------------------------------------------------------------------------------
void MakingLineSegments(std::vector<Segment>& segments, std::vector<std::vector<double>> pointcloud, std::vector<int> index, int i, int n, double lambda_old) { /* B */
    Segment segment;
    double xd;
    double yd;
    if (i+1+n < pointcloud.size()) {
        std::vector<double> p1 = pointcloud[i];
        std::vector<double> p2 = pointcloud[i+n+1];
        std::vector<double> p3 = pointcloud[i+n];
        double dist = std::sqrt((pointcloud[i+n][0]-pointcloud[i+n-1][0])*(pointcloud[i+n][0]-pointcloud[i+n-1][0])+(pointcloud[i+n][1]-pointcloud[i+n-1][1])*(pointcloud[i+n][1]-pointcloud[i+n-1][1]));
        double lambda = 0;
        xd = (1/(std::sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]))))*(p2[0]-p1[0]);
        yd = (1/(std::sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]))))*(p2[1]-p1[1]);
        for (int j = 0; j < 1+n; j++) {
            lambda = lambda + (std::abs(xd*(-pointcloud[i+j][1]+p1[1])+yd*(pointcloud[i+j][0]-p1[0])))/(5+n);
        }
        if ((lambda < 0.001 || n < 4) && dist < 1) {
            MakingLineSegments(segments,pointcloud, index, i, n + 1, lambda);
        } else if (lambda >= 0.001 && lambda_old < 0.001 && dist < 1) {
            segment.p1 = p1;
            segment.p2 = p3;
            xd = (1 / (std::sqrt((p3[0] - p1[0]) * (p3[0] - p1[0]) + (p3[1] - p1[1]) * (p3[1] - p1[1])))) *
                 (p3[0] - p1[0]);
            yd = (1 / (std::sqrt((p3[0] - p1[0]) * (p3[0] - p1[0]) + (p3[1] - p1[1]) * (p3[1] - p1[1])))) *
                 (p3[1] - p1[1]);
            segment.dv = {xd, yd};
            segments.push_back(segment);
            if (index[i+n+1] == index[i+n]+1) {
                MakingLineSegments(segments,pointcloud, index, i + n, 0, 0);
            } else {
                MakingLineSegments(segments,pointcloud, index, i + n + 1, 0, 0);
            }

        } else {
            MakingLineSegments(segments,pointcloud, index, i + 1, 0, 0);
        }
    } else {
        segment.p1 = pointcloud[i];
        segment.p2 = pointcloud[pointcloud.size()-1];
        xd = (1 / (std::sqrt((segment.p2[0] - segment.p1[0]) * (segment.p2[0] - segment.p1[0]) + (segment.p2[1] - segment.p1[1]) * (segment.p2[1] - segment.p1[1])))) *
             (segment.p2[0] - segment.p1[0]);
        yd = (1 / (std::sqrt((segment.p2[0] - segment.p1[0]) * (segment.p2[0] - segment.p1[0]) + (segment.p2[1] - segment.p1[1]) * (segment.p2[1] - segment.p1[1])))) *
             (segment.p2[1] - segment.p1[1]);
        segment.dv = {xd, yd};
        segments.push_back(segment);
        /*std::vector<int> news;
        for (int j = 1; j < pointcloud.size()-1-i; j++) {
            double dist = std::sqrt((pointcloud[i+j][0]-pointcloud[i+j-1][0])*(pointcloud[i+j][0]-pointcloud[i+j-1][0])+(pointcloud[i+j][0]-pointcloud[i+j-1][0])*(pointcloud[i+j][0]-pointcloud[i+j-1][0]));
            if (dist >= 0.8) {
                news.push_back(i+j);
            }
        }
        if (news.empty()){
            xd = (1 / (std::sqrt((segment.p2[0] - segment.p1[0]) * (segment.p2[0] - segment.p1[0]) + (segment.p2[1] - segment.p1[1]) * (segment.p2[1] - segment.p1[1])))) *
                 (segment.p2[0] - segment.p1[0]);
            yd = (1 / (std::sqrt((segment.p2[0] - segment.p1[0]) * (segment.p2[0] - segment.p1[0]) + (segment.p2[1] - segment.p1[1]) * (segment.p2[1] - segment.p1[1])))) *
                 (segment.p2[1] - segment.p1[1]);
            segment.dv = {xd, yd};
            segments.push_back(segment);
        } else {
            news.push_back(int(pointcloud.size()-1));
            segment.p2 = pointcloud[i];
            for (int j = 0; j < news.size()-1;i++) {
                segment.p1 = segment.p2;
                segment.p2 = pointcloud[j];
                xd = (1 / (std::sqrt((segment.p2[0] - segment.p1[0]) * (segment.p2[0] - segment.p1[0]) + (segment.p2[1] - segment.p1[1]) * (segment.p2[1] - segment.p1[1])))) *
                     (segment.p2[0] - segment.p1[0]);
                yd = (1 / (std::sqrt((segment.p2[0] - segment.p1[0]) * (segment.p2[0] - segment.p1[0]) + (segment.p2[1] - segment.p1[1]) * (segment.p2[1] - segment.p1[1])))) *
                     (segment.p2[1] - segment.p1[1]);
                segment.dv = {xd, yd};
                segments.push_back(segment);
            }
        }*/

    }
}
void ResetSegmentFrame(std::vector<Segment>& segments, std::vector<double>& robot_pose) {
    for(auto & segment : segments) {
        double p1x = std::cos(robot_pose[2])*segment.p1[0]-std::sin(robot_pose[2])*segment.p1[1]+robot_pose[0];
        double p1y = std::sin(robot_pose[2])*segment.p1[0]+std::cos(robot_pose[2])*segment.p1[1]+robot_pose[1];
        double p2x = std::cos(robot_pose[2])*segment.p2[0]-std::sin(robot_pose[2])*segment.p2[1]+robot_pose[0];
        double p2y = std::sin(robot_pose[2])*segment.p2[0]+std::cos(robot_pose[2])*segment.p2[1]+robot_pose[1];
        segment.p1[0] = p1x;
        segment.p1[1] = p1y;
        segment.p2[0] = p2x;
        segment.p2[1] = p2y;
        segment.dv[0] = 1/std::sqrt((p2x-p1x)*(p2x-p1x)+(p2y-p1y)*(p2y-p1y))*(p2x-p1x);
        segment.dv[1] = 1/std::sqrt((p2x-p1x)*(p2x-p1x)+(p2y-p1y)*(p2y-p1y))*(p2y-p1y);
    }
}
//----------------------------------------------------------------------------------------------------------------------
// Functions for driving
//----------------------------------------------------------------------------------------------------------------------
bool Wait(std::vector<std::vector<double>> pointcloud, double b, Line area) { /* G */
    double d = 0.5;
    double delta = 0.1;
    int n = 0;
    bool wait;
    for (auto & i : pointcloud) {
        if (std::abs(i[1]) < d*i[0]/(2*b+delta) && area.p1[0] > i[0] > area.p2[0] && area.p1[1] > i[1] < area.p2[1]) {
            n += 1;
        }
    }
    if (n > 20) {
        wait = true;
        ROS_INFO("Now waiting");
    } else {
        wait = false;
        ROS_INFO("Now driving");
    }
    return wait;
}
std::vector<double> CalculateIndices(std::vector<double> pose_diff, double amax, double awmax, double F) {
    double vmax, tmax, n, m, k;
    std::vector<double> indices;
    vmax = std::sqrt(std::abs(pose_diff[0])*amax);
    tmax = vmax/amax;
    n = tmax*F;
    vmax = std::sqrt(std::abs(pose_diff[1])*amax);
    tmax = vmax/amax;
    m = tmax*F;
    vmax = std::sqrt(std::abs(pose_diff[2])*awmax);
    tmax = vmax/awmax;
    k = tmax*F;
    indices = {n, m, k};
    return indices;
}
std::vector<double> VelocityOL(std::vector<double> old_speed, double f, double b, std::vector<std::vector<double>>& pointcloud, Line area, double& i, double n, double m, double k, bool& driving) { /* F */
    double amax = 1.05;
    double awmax = (3*M_PI)/68;
    bool wait = Wait(pointcloud, b, std::move(area));
    std::vector<double> speed;
    if (!wait) {
        if ( i < 1.4*n ) {
            ROS_INFO("acc x");
            i += 1;
            speed = {old_speed[0]+amax/f, 0, 0};
        } else if ( i < 2.8*n ) {
            ROS_INFO("dec x");
            i += 1;
            speed = {old_speed[0]-amax/f, 0, 0};
        } else  if ( i < (2.8*n+1.4*m) ) {
            ROS_INFO("acc y");
            i += 1;
            speed = {0, old_speed[1]+amax/f, 0};
        } else if ( i < (2.8*n+2.8*m) ) {
            ROS_INFO("dec y");
            i += 1;
            speed = {0, old_speed[1]-amax/f, 0};
        } else if ( i < (2.8*n+2.8*m+k) ) {
            ROS_INFO("acc theta");
            i += 1;
            speed = {0, 0, old_speed[2]+awmax/f};
        } else if ( i < (2.8*n+2.8*m+2*k) ){
            ROS_INFO("dec theta");
            i += 1;
            speed = {0, 0, old_speed[2]-awmax/f};
        } else {
            driving = false;
            speed = {0, 0, 0};
        }
    } else {
        speed = {0, 0, 0};
    }
    return speed;
}
std::vector<double> VelocityCL(std::vector<double> old_speed, double f, std::vector<double> error) { /* F */
    double amax = 0.82;
    double awmax = (3*M_PI)/70;
    std::vector<double> speed;
    if ( error[2] > 0.03*M_PI ) {
        ROS_INFO("acc theta");
        speed = {0, 0, old_speed[2]+awmax/f};
    } else if ( error[2] < -0.03*M_PI ) {
        ROS_INFO("dec theta");
        speed = {0, 0, old_speed[2]-awmax/f};
    } else if ( error[0] > 0.08 ) {
        ROS_INFO("acc x");
        speed = {old_speed[0]+amax/f, 0, 0};
    } else if ( error[0] < -0.08 ) {
        ROS_INFO("dec x");
        speed = {old_speed[0]-amax/f, 0, 0};
    } else if ( error[1] > 0.02 ) {
        ROS_INFO("acc y");
        speed = {0, old_speed[1]+amax/f, 0};
    } else if ( error[1] < -0.02 ) {
        ROS_INFO("dec y");
        speed = {0, old_speed[1]-amax/f, 0};
    } else {
        speed = {0, 0, 0};
    }
    return speed;
}
void SetTwistMessageOL(geometry_msgs::Twist& twist_msg, std::vector<double> speed, std::vector<double> pose_diff) {
    if (pose_diff[0] > 0) {
        twist_msg.linear.x = speed[0];
    } else {
        twist_msg.linear.x = -speed[0];
    }
    if (pose_diff[1] > 0) {
        twist_msg.linear.y = speed[1];
    } else {
        twist_msg.linear.y = -speed[1];
    }
    if (pose_diff[2] > 0) {
        twist_msg.angular.z = speed[2];
    } else {
        twist_msg.angular.z = -speed[2];
    }
}
void SetTwistMessage(geometry_msgs::Twist& twist_msg, std::vector<double> speed) {
    twist_msg.linear.x = speed[0];
    twist_msg.linear.y = speed[1];
    twist_msg.angular.z = speed[2];
}
//----------------------------------------------------------------------------------------------------------------------
// Functions for orientation
//----------------------------------------------------------------------------------------------------------------------
std::vector<double> FindEntrance(std::vector<Segment>& segments, const std::vector<double>& robot_pose) { /* C */
    std::vector<double> pose = {0,0,0};
    for (int i =0; i < segments.size(); i++) {
        for (int j=i+1; j < (segments.size()); j++) {
            if ((std::abs(segments[i].dv[0])-0.05)<std::abs(segments[j].dv[0])<(std::abs(segments[i].dv[0])+0.05) && (std::abs(segments[i].dv[1])-0.05)<std::abs(segments[j].dv[1])<(std::abs(segments[i].dv[1])+0.05) && (segments[i].p2[0] != segments[j].p1[0] && segments[i].p2[1] != segments[j].p1[1])) {
                double seg_midx = (segments[i].p1[0]+segments[i].p2[0])/2;
                double seg_midy = (segments[i].p1[1]+segments[i].p2[1])/2;
                double rvx = - segments[i].dv[1];
                double rvy = segments[i].dv[0];
                double dvx = segments[j].dv[0];
                double dvy = segments[j].dv[1];
                double px = segments[j].p1[0];
                double py = segments[j].p1[1];
                double lambda = (dvy*rvx)/(dvy*rvx-rvy*dvx)*((px-seg_midx)/rvx+(dvx/rvx)*(seg_midy-py)/dvy);
                double ww = std::sqrt(lambda*rvx*lambda*rvx + lambda*rvy*lambda*rvy);
                if ((0.82 < ww < 0.89 || 0.82 < 0.5*ww < 0.89) && pose[0] == 0) {
                    double x = 0.5*(segments[i].p2[0]-segments[j].p1[0]);
                    double y = 0.5*(segments[i].p2[1]-segments[j].p1[1]);
                    double alpha = AngularDifference(segments[i].dv)+robot_pose[2];
                    pose = {x, y, alpha};
                }

            }
        }
    }
    return pose;
}
void FindAreaPose(Line& facing, std::vector<double>& destination) { /* O */
    std::vector<double> dv;
    double dx = facing.p2[0]-facing.p1[0];
    double dy = facing.p2[1]-facing.p1[1];
    dv.push_back(1/(std::sqrt(dx*dx+dy*dy))*dx);
    dv.push_back(1/(std::sqrt(dx*dx+dy*dy))*dy);
    double facing_angle = AngularDifference(dv);//-M_PI_2;
    destination.push_back((facing.p1[0]+facing.p2[0])/2);
    destination.push_back((facing.p1[1]+facing.p2[1])/2);
    destination.push_back(facing_angle);
}
Segment FindCart(std::vector<Segment>& segments, Line area, Line facing) { /* J */
    Segment cart_segment;
    for (auto & segment : segments) {
        if ( area.p1[0] < segment.p1[0] && segment.p1[0] < area.p2[0] && area.p1[1] < segment.p1[1] && segment.p1[1] < area.p2[1]
        && area.p1[0] < segment.p2[0] && segment.p2[0] < area.p2[0] && area.p1[1] < segment.p2[1] && segment.p2[1] < area.p2[1]) {
            double dxs = segment.p2[0]-segment.p1[0];
            double dys = segment.p2[1]-segment.p1[1];
            double dxf = facing.p2[0]-facing.p1[0];
            double dyf = facing.p2[1]-facing.p1[1];
            double alpha = std::acos((dxs*dxf+dys*dyf)/(std::sqrt(dxs*dxs+dys*dys)*std::sqrt(dxf*dxf+dyf*dyf)));
            double length = std::sqrt(dxs*dxs+dys*dys);
            if ( std::abs(alpha) < M_PI_4 && std::abs(length-0.7) < 0.2) {
                cart_segment = segment;
            }
        }
    }
    if (cart_segment.p1.empty()) {
        ROS_INFO("No cart Segment could be found");
        Segment empty_segment;
        empty_segment.p1 = {0,0};
        empty_segment.p2 = {0,0};
        empty_segment.dv = {0,0};
        cart_segment = empty_segment;
    }

    return cart_segment;
}
std::vector<double> FindDesiredPose(Segment cart_segment, double range_x, double range_y) { /* L */
    double dx = cart_segment.p2[0]-cart_segment.p1[0];
    double dy = cart_segment.p2[1]-cart_segment.p1[1];
    std::vector<double> dv = {1/(std::sqrt(dx*dx+dy*dy))*dx, 1/(std::sqrt(dx*dx+dy*dy))*dy};
    // range x is with normal vector and range y is with direction vector.
    double x = 0.5*(cart_segment.p1[0]+cart_segment.p2[0]) - range_x*dv[1] - range_y*dv[0];
    double y = 0.5*(cart_segment.p1[1]+cart_segment.p2[1]) + range_x*dv[0] - range_y*dv[1];
    double alpha = AngularDifference(cart_segment.dv)+M_PI_2;
    std::vector<double> pose = {x, y, alpha};
    return pose;
}
std::vector<Segment> FindObjects(std::vector<Segment>& segments, const std::vector<double>& desired_pose, const Segment& cart) { /* I */ // Kijken of hier iets van evenwijdig aan cart toe te voegen is
    std::vector<Segment> certain_segments;
    double Mx = 0.5*(cart.p1[0]+cart.p2[0]);
    double My = 0.5*(cart.p1[1]+cart.p2[1]);
    double dx = cart.dv[0];
    double dy = cart.dv[1];
    double d1 = dy*Mx - dx*My;
    double d2 = dy*(Mx-4*dy) - dx*(My-4*dx);
    double d3 = dx*(Mx-4*dx) - dy*(My-4*dy);
    double d4 = dx*(Mx+4*dx) - dy*(My+4*dy);
    for (auto & segment : segments) {
        ROS_INFO("point 1: %f < %f < %f & %f < %f < %f", d2, dy*cart.dv[1]*segment.p1[0] + dx*segment.p1[1], d1, d4, dx*cart.dv[1]*segment.p1[0] - dy*segment.p1[1], d3);
        ROS_INFO("point 2: %f < %f < %f & %f < %f < %f", d2, dy*cart.dv[1]*segment.p2[0] + dx*segment.p2[1], d1, d4, dx*cart.dv[1]*segment.p2[0] - dy*segment.p2[1], d3);
        if (dy*cart.dv[1]*segment.p1[0] + dx*segment.p1[1] < d1 && dy*cart.dv[1]*segment.p1[0] + dx*segment.p1[1] > d2 &&
            dx*cart.dv[1]*segment.p1[0] - dy*segment.p1[1] < d3 && dx*cart.dv[1]*segment.p1[0] - dy*segment.p1[1] > d4 &&
            dy*cart.dv[1]*segment.p2[0] + dx*segment.p2[1] < d1 && dy*cart.dv[1]*segment.p2[0] + dx*segment.p2[1] > d2 &&
            dx*cart.dv[1]*segment.p2[0] - dy*segment.p2[1] < d3 && dx*cart.dv[1]*segment.p2[0] - dy*segment.p2[1] > d4) {
            ROS_INFO("The complete segments will be visible while positioning");
            double sdvl = std::sqrt(segment.dv[0]*segment.dv[0]+segment.dv[1]*segment.dv[1]);
            double cdvl = std::sqrt(cart.dv[0]*cart.dv[0]+cart.dv[1]*cart.dv[1]);
            double alpha = std::acos((segment.dv[0]*cart.dv[0]+segment.dv[1]*cart.dv[1])/(sdvl*cdvl));
            ROS_INFO("The angle between two segments is calculated");
            ROS_INFO("%f < %f", std::abs(alpha) - M_PI_2, M_PI_4);
            if (std::abs(std::abs(alpha) - M_PI_2) < M_PI_4) {
                ROS_INFO("Segment found that is about perpendicular to the cart segment");
                certain_segments.push_back(segment);
            }
        }

    }
    return certain_segments;
}
//----------------------------------------------------------------------------------------------------------------------
// Additional mathematical tools
//----------------------------------------------------------------------------------------------------------------------
double AngularDifference(std::vector<double> dv) { /* E */
    double dx = dv[0];
    double dy = dv[1];
    double alpha = std::acos((dx)/(std::sqrt(dx*dx+dy*dy)));

    return alpha;
}
std::vector<double> TransformPosition(std::vector<double> old_point, std::vector<double> rel_pose) { /* D */
    double x = std::cos(rel_pose[2])*old_point[0]-std::sin(rel_pose[2])*old_point[1] + rel_pose[0];
    double y = std::sin(rel_pose[2])*old_point[0]+std::cos(rel_pose[2])*old_point[1] + rel_pose[1];
    std::vector<double> point = {x, y};
    return point;
}
std::vector<double> TransformPose(std::vector<double> old_point, std::vector<double> rel_pose) { /* D */
    double x = std::cos(rel_pose[2])*old_point[0]-std::sin(rel_pose[2])*old_point[1] + rel_pose[0];
    double y = std::sin(rel_pose[2])*old_point[0]+std::cos(rel_pose[2])*old_point[1] + rel_pose[1];
    double t = rel_pose[2];
    std::vector<double> pose = {x, y, t};
    return pose;
}
std::vector<double> PoseDifference(std::vector<double> robot_pose, const std::vector<double>& destination) {
    double t = destination[2]-robot_pose[2];
    double x = std::cos(robot_pose[2])*(destination[0]-robot_pose[0])+std::sin(robot_pose[2])*(destination[1]-robot_pose[1]); //change
    double y = -std::sin(robot_pose[2])*(destination[0]-robot_pose[0])+std::cos(robot_pose[2])*(destination[1]-robot_pose[1]); //change
    std::vector<double> pose_diff = {x, y, t};
    return pose_diff;
}
void CalculateNewRobotPose (std::vector<double>& pose, std::vector<double>& vel, double F) {
    double t = pose[2]+vel[2]/F;
    double x = std::cos(pose[2])*vel[0]/F - std::sin(pose[2])*vel[1]/F + pose[0];
    double y = std::cos(pose[2])*vel[1]/F + std::sin(pose[2])*vel[0]/F + pose[1];
    pose = {x, y, t};
}
std::vector<Segment> FindDistance(const std::vector<double>& desired_pose, const std::vector<double>& robot_pose, const std::vector<Segment>& certain_segments) { /* M */
    Segment distance;
    std::vector<Segment> distances;
    for (auto & certain_segment : certain_segments) {
        distance.p1 = TransformPosition(certain_segment.p1, desired_pose);
        distance.p2 = TransformPosition(certain_segment.p2, desired_pose);
        double xd = (1/(std::sqrt((distance.p2[0]-distance.p1[0])*(distance.p2[0]-distance.p1[0])+(distance.p2[1]-distance.p1[1])*(distance.p2[1]-distance.p1[1]))))*(distance.p2[0]-distance.p1[0]);
        double yd = (1/(std::sqrt((distance.p2[0]-distance.p1[0])*(distance.p2[0]-distance.p1[0])+(distance.p2[1]-distance.p1[1])*(distance.p2[1]-distance.p1[1]))))*(distance.p2[1]-distance.p1[1]);
        distance.dv = {xd, yd};
        distances.push_back(distance);
    }
    return distances;
}
std::vector<double> CalculateError(std::vector<Segment>& distances, std::vector<Segment2>& localization_segments, std::vector<Segment>& segments, std::vector<double>& robot_pose) {
    std::vector<double> error = {0,0,0};
    int ex = 0, ey = 0, et = 0;
    for (auto & localization_segment : localization_segments) {
        int j = localization_segment.index;
        int k = localization_segment.number;
        double alpha = std::acos(segments[k].dv[0])-std::acos(distances[j].dv[0]);
        et += 1;
        error[2] += alpha;
        if (localization_segment.cat == 0) { //both point on the line
            double x1d = std::cos(alpha)*distances[j].p1[0] - std::sin(alpha)*distances[j].p1[1];
            double y1d = std::sin(alpha)*distances[j].p1[0] + std::cos(alpha)*distances[j].p1[1];
            error[0] += segments[k].p1[0] - x1d;
            error[1] += segments[k].p1[1] - y1d;
            double x2d = std::cos(alpha)*distances[j].p2[0] - std::sin(alpha)*distances[j].p2[1];
            double y2d = std::sin(alpha)*distances[j].p2[0] + std::cos(alpha)*distances[j].p2[1];
            error[0] += segments[k].p2[0] - x2d;
            error[1] += segments[k].p2[1] - y2d;
            ex += 2;
            ey += 2;
            ROS_INFO("error 1: %f, %f, %f", segments[k].p1[0] - x1d, segments[k].p1[1] - y1d, alpha);
            ROS_INFO("error 1: %f, %f, %f", segments[k].p2[0] - x2d, segments[k].p2[1] - y2d, alpha);
        } else if (localization_segment.cat == 1) { //begin point on the line
            double x1d = std::cos(alpha)*distances[j].p1[0] - std::sin(alpha)*distances[j].p1[1];
            double y1d = std::sin(alpha)*distances[j].p1[0] + std::cos(alpha)*distances[j].p1[1];
            error[0] += segments[k].p1[0] - x1d;
            error[1] += segments[k].p1[1] - y1d;
            ex += 1;
            ey += 1;
            ROS_INFO("error 1: %f, %f, %f", segments[k].p1[0] - x1d, segments[k].p1[1] - y1d, alpha);
        } else { //end point on the line
            double x2d = std::cos(alpha)*distances[j].p2[0] - std::sin(alpha)*distances[j].p2[1];
            double y2d = std::sin(alpha)*distances[j].p2[0] + std::cos(alpha)*distances[j].p2[1];
            error[0] += segments[k].p2[0] - x2d;
            error[1] += segments[k].p2[1] - y2d;
            ex += 1;
            ey += 1;
            ROS_INFO("error 1: %f, %f, %f", segments[k].p2[0] - x2d, segments[k].p2[1] - y2d, alpha);
        }
       error[0] = error[0]/ex;
       error[1] = error[1]/ey;
       error[2] = error[2]/et;
    }
    return error;
}
std::vector<Segment2> CompareSegments(std::vector<Segment>& old_segments, std::vector<Segment>& new_segments) { /* H */
    std::vector<Segment2> match;
    Segment2 segment;
    for( int i = 0; i < old_segments.size(); i++) {
        for ( int j = 0; j < new_segments.size(); j++) {
            double dxb = old_segments[i].p1[0]-new_segments[j].p1[0];
            double dxe = old_segments[i].p2[0]-new_segments[j].p2[0];
            double dyb = old_segments[i].p1[1]-new_segments[j].p1[1];
            double dye = old_segments[i].p2[1]-new_segments[j].p2[1];
            double rb2 = dxb*dxb+dyb*dyb;
            double re2 = dxe*dxe+dye*dye;
            double xdi = old_segments[i].dv[0];
            double ydi = old_segments[i].dv[1];
            double xdj = new_segments[j].dv[0];
            double ydj = new_segments[j].dv[1];
            if (rb2 < 0.003 && re2 < 0.1) {
                segment.p1 = new_segments[j].p1;
                segment.p2 = new_segments[j].p2;
                segment.dv = new_segments[j].dv;
                segment.index = i;
                segment.cat = 0;
                segment.number = j;
                match.push_back(segment);
                new_segments[j].p1 = {1000,1000};
            } else if (rb2 < 0.1 && re2 < 0.003) {
                segment.p1 = new_segments[j].p1;
                segment.p2 = new_segments[j].p2;
                segment.dv = new_segments[j].dv;
                segment.index = i;
                segment.cat = 0;
                segment.number = j;
                match.push_back(segment);
                new_segments[j].p1 = {1000,1000};
            } else if (rb2 < 0.003 && std::abs(xdi-xdj) < 0.01 && std::abs(ydi-ydj) < 0.01) {
                segment.p1 = new_segments[j].p1;
                segment.p2 = new_segments[j].p2;
                segment.dv = new_segments[j].dv;
                segment.index = i;
                segment.cat = 1;
                segment.number = j;
                match.push_back(segment);
                new_segments[j].p1 = {1000,1000};
            } else if (re2 < 0.003 && std::abs(xdi-xdj) < 0.01 && std::abs(ydi-ydj) < 0.01) {
                segment.p1 = new_segments[j].p1;
                segment.p2 = new_segments[j].p2;
                segment.dv = new_segments[j].dv;
                segment.index = i;
                segment.cat = 2;
                segment.number = j;
                match.push_back(segment);
                new_segments[j].p1 = {1000,1000};
            } else if (rb2 < 0.1 && std::abs(xdi-xdj) < 0.01 && std::abs(ydi-ydj) < 0.01) {
                segment.p1 = new_segments[j].p1;
                segment.p2 = new_segments[j].p2;
                segment.dv = new_segments[j].dv;
                segment.index = i;
                segment.cat = 1;
                segment.number = j;
                match.push_back(segment);
                new_segments[j].p1 = {1000,1000};
            } else if (re2 < 0.1 && std::abs(xdi-xdj) < 0.01 && std::abs(ydi-ydj) < 0.01) {
                segment.p1 = new_segments[j].p1;
                segment.p2 = new_segments[j].p2;
                segment.dv = new_segments[j].dv;
                segment.index = i;
                segment.cat = 2;
                segment.number = j;
                match.push_back(segment);
                new_segments[j].p1 = {1000,1000};
            } else if (rb2 < 0.1 && std::abs(xdi-xdj) < 0.01 && std::abs(ydi-ydj) < 0.1) {
                segment.p1 = new_segments[j].p1;
                segment.p2 = new_segments[j].p2;
                segment.dv = new_segments[j].dv;
                segment.index = i;
                segment.cat = 1;
                segment.number = j;
                match.push_back(segment);
                new_segments[j].p1 = {1000,1000};
            } else if (re2 < 0.1 && std::abs(xdi-xdj) < 0.1 && std::abs(ydi-ydj) < 0.01) {
                segment.p1 = new_segments[j].p1;
                segment.p2 = new_segments[j].p2;
                segment.dv = new_segments[j].dv;
                segment.index = i;
                segment.cat = 2;
                segment.number = j;
                match.push_back(segment);
                new_segments[j].p1 = {1000,1000};
            }
        }
    }
    return match;
}
