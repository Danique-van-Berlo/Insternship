#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "Functions.h"

#include <utility>
#include "Classes.h"


void MakingLineSegments(std::vector<Segment>& segments, std::vector<std::vector<double>> pointcloud, std::vector<int> index, int i, int n, double lambda_old) { /* B */
    Segment segment;
    double xd;
    double yd;
    if (i+6+n < pointcloud.size()) {
        std::vector<double> p1 = pointcloud[i];
        std::vector<double> p2 = pointcloud[i+5+n];
        std::vector<double> p3 = pointcloud[i+4+n];
        double lambda = 0;
        xd = (1/(std::sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]))))*(p2[0]-p1[0]);
        yd = (1/(std::sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]))))*(p2[1]-p1[1]);
        for (int j = 0; j < 6+n; j++) {
            //ROS_INFO("Segment %zu consists of cloud number %d", segments.size(), index[i+j]);
            lambda = lambda + (std::abs(xd*(-pointcloud[i+j][1]+p1[1])+yd*(pointcloud[i+j][0]-p1[0])))/(5+n);
        }
        if (lambda < 0.0001 && index[i+n+5]-index[i] == n+5) {
            ROS_INFO("adding: %d - %d = %d", index[i+n+5], index[i], n+5);
            MakingLineSegments(segments,pointcloud, index, i, n + 1, lambda);
        } else if (lambda >= 0.0001 || index[i+n+4]-index[i] == n+4) {
            ROS_INFO("saving: %d - %d = %d", index[i+n+4], index[i], n+4);
            segment.p1 = p1;
            segment.p2 = p3;
            xd = (1 / (std::sqrt((p3[0] - p1[0]) * (p3[0] - p1[0]) + (p3[1] - p1[1]) * (p3[1] - p1[1])))) *
                 (p3[0] - p1[0]);
            yd = (1 / (std::sqrt((p3[0] - p1[0]) * (p3[0] - p1[0]) + (p3[1] - p1[1]) * (p3[1] - p1[1])))) *
                 (p3[1] - p1[1]);
            segment.dv = {xd, yd};
            segments.push_back(segment);
            if (index[i+n+5] == index[i+n+4]+1) {
                MakingLineSegments(segments,pointcloud, index, i + n + 4, 0, 0);
            } else {
                MakingLineSegments(segments,pointcloud, index, i + n + 5, 0, 0);
            }

        } else {
            ROS_INFO("%d - %d = %d", index[i+n+5], index[i], n+5);
            ROS_INFO("%d - %d = %d", index[i+n+4], index[i], n+4);
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
    }
}

/*------------------------------------------------------------------------------------------------------------------------*/
double RotationDifference(std::vector<double> dv) { /* E */
    double dx = dv[0];
    double dy = dv[1];
    double alpha = std::acos((dx)/(std::sqrt(dx*dx+dy*dy)));

    return alpha;
}
/*-----------------------------------------------------------------------------------------------------------------------*/
std::vector<double> FindingEntrance(std::vector<Segment>& segments, const std::vector<double>& robot_pose) { /* C */
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
                    double alpha = RotationDifference(segments[i].dv)+robot_pose[2];
                    ROS_INFO("direction vector: x %f y %f", segments[i].dv[0], segments[i].dv[1]);
                    pose = {x, y, alpha};
                }

            }
        }
    }
    return pose;
}
/*-----------------------------------------------------------------------------------------------------------------------*/
std::vector<double> TransformPositionB(std::vector<double> old_point, std::vector<double> rel_pose) { /* D */
    double x = std::cos(rel_pose[2])*old_point[0]-std::sin(rel_pose[2])*old_point[1] + rel_pose[0];
    double y = std::sin(rel_pose[2])*old_point[0]+std::cos(rel_pose[2])*old_point[1] + rel_pose[1];
    std::vector<double> point = {x, y};
    return point;
}
/*-----------------------------------------------------------------------------------------------------------------------*/
std::vector<double> TransformPose(std::vector<double> old_point, std::vector<double> rel_pose) { /* D */
    double x = std::cos(rel_pose[2])*old_point[0]-std::sin(rel_pose[2])*old_point[1] + rel_pose[0];
    double y = std::sin(rel_pose[2])*old_point[0]+std::cos(rel_pose[2])*old_point[1] + rel_pose[1];
    double t = rel_pose[2];
    std::vector<double> pose = {x, y, t};
    return pose;
}
/*------------------------------------------------------------------------------------------------------------------------*/
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
    if (n > 300) {
        wait = true;
        ROS_INFO("Now waiting");
    } else {
        wait = false;
        ROS_INFO("Now driving");
    }
    return wait;
}
/*------------------------------------------------------------------------------------------------------------------------*/
std::vector<double> CalculateSpeed(std::vector<double> pose_diff, std::vector<double> old_speed, double f, double b, std::vector<std::vector<double>>& pointcloud, Line area) { /* F */
    /* Velocity constraints */
    double vmax = 3.5;
    double wmax = (3*M_PI)/32;
    /* Acceleration constraints */
    double amax = 1.05;
    double awmax = (3*M_PI)/68;
    bool wait = Wait(pointcloud, b, std::move(area));
    ROS_INFO("Calculating speed");
    std::vector<double> speed;
    if ((std::abs(pose_diff[2]) > 0.03*M_PI) && !wait) {
        ROS_INFO("Calculate the speed in theta");
        double vt = f*pose_diff[2];
        double at = 0.5*f*(vt-old_speed[2]);
        if (at > awmax){
            at = awmax;
        } else if (at < -awmax){
            at = -awmax;
        }
        vt = old_speed[2] + at/f;
        if (vt > wmax){
            vt = wmax;
        } else if (vt < -wmax){
            vt = -wmax;
        }
        /* if (vt*vt/awmax >= pose_diff[2]) { Improve for deceleration
            at = -awmax;
            vt = old_speed[2] + at/f;
        } */
        speed = {0,0,vt};
    } else if ((std::abs(pose_diff[1]) > 0.08) && !wait) {
        ROS_INFO("Calculate the speed in y");
        double vy = f*pose_diff[1];
        double ay = 0.5*f*(vy-old_speed[1]);
        if (ay > amax){
            ay = amax;
        } else if (ay < -amax){
            ay = -amax;
        }
        vy = old_speed[1] + ay/f;
        if (vy > vmax){
            vy = vmax;
        } else if (vy < -vmax){
            vy = -vmax;
        }
        /* if (vy*vy/amax >= pose_diff[1]) { Improve for deceleration
            ay = -amax;
            vy = old_speed[1] + ay/f;
        } */
        speed = {0,vy,0};
    } else if ((std::abs(pose_diff[0]) > 0.08) && !wait) {
        ROS_INFO("Calculate the speed in x");
        double vx = f*pose_diff[0];
        double ax = 0.5*f*(vx-old_speed[0]);
        if (ax > amax){
            ax = amax;
        } else if (ax < -amax){
            ax = -amax;
        }
        vx = old_speed[0] + ax/f;
        if (vx > vmax){
            vx = vmax;
        } else if (vx < -vmax){
            vx = -vmax;
        }
        /* if (vx*vx/amax >= pose_diff[0]) { Improve for deceleration
            ax = -amax;
            vx = old_speed[0] + ax/f;
        } */
        speed = {vx,0,0};
    } else {
        speed = {0, 0, 0};
    }
    return speed;
}
/*----------------------------------------------------------------------------------------------------------------------*/
std::vector<double> CalculatePositionSpeed(std::vector<double> pose_diff, std::vector<double> old_speed, double f) { /* F */
    /* Velocity constraints */
    double vmax = 1.0;
    double wmax = (3*M_PI)/32;
    /* Acceleration constraints */
    double amax = 0.22;
    double awmax = (3*M_PI)/68;
    std::vector<double> speed;
    ROS_INFO("Calculating speed");
    if (std::abs(pose_diff[2]) > 0.01*M_PI) {
        ROS_INFO("Calculate the speed in theta");
        double vt = f*pose_diff[2];
        double at = f*(vt-old_speed[2]);
        if (at > awmax){
            at = awmax;
        } else if (at < -awmax){
            at = -awmax;
        }
        vt = old_speed[2] + at/f;
        if (vt > wmax){
            vt = wmax;
        } else if (vt < -wmax){
            vt = -wmax;
        }
        /* if (vt*vt/awmax >= pose_diff[2]) { Improve for deceleration
            at = -awmax;
            vt = old_speed[2] + at/f;
        } */
        speed = {0,0,vt};
    } else if (std::abs(pose_diff[1]) > 0.08) {
        ROS_INFO("Calculate the speed in y");
        double vy = f*pose_diff[1];
        double ay = f*(vy-old_speed[1]);
        if (ay > amax){
            ay = amax;
        } else if (ay < -amax){
            ay = -amax;
        }
        vy = old_speed[1] + ay/f;
        if (vy > vmax){
            vy = vmax;
        } else if (vy < -vmax){
            vy = -vmax;
        }
        /* if (vy*vy/amax >= pose_diff[1]) { Improve for deceleration
            ay = -amax;
            vy = old_speed[1] + ay/f;
        } */
        speed = {0,vy,0};
    } else if (std::abs(pose_diff[0]) > 0.02) {
        ROS_INFO("Calculate the speed in x");
        double vx = f*pose_diff[0];
        double ax = f*(vx-old_speed[0]);
        if (ax > amax){
            ax = amax;
        } else if (ax < -amax){
            ax = -amax;
        }
        vx = old_speed[0] + ax/f;
        if (vx > vmax){
            vx = vmax;
        } else if (vx < -vmax){
            vx = -vmax;
        }
        /* if (vx*vx/amax >= pose_diff[0]) { Improve for deceleration
            ax = -amax;
            vx = old_speed[0] + ax/f;
        } */
        speed = {vx,0,0};
    } else {
        ROS_INFO("theta: |%f| < 0.05pi", pose_diff[2]);
        ROS_INFO("y: |%f| < 0.05pi", pose_diff[1]);
        ROS_INFO("x: |%f| < 0.05pi", pose_diff[0]);
        speed = {0, 0, 0};
    }
    return speed;
}
/*------------------------------------------------------------------------------------------------------------------------*/
std::vector<double> FindPoseDiff2(std::vector<double> robot_pose, std::vector<double> destination) {
    double t = destination[2]-robot_pose[2];
    double x = -std::cos(robot_pose[2])*(destination[0]-robot_pose[0])+std::sin(robot_pose[2])*(destination[1]-robot_pose[1]);
    double y = -std::sin(robot_pose[2])*(destination[0]-robot_pose[0])-std::cos(robot_pose[2])*(destination[1]-robot_pose[1]);
    std::vector<double> pose_diff = {x, y, t};
    return pose_diff;
}
/*------------------------------------------------------------------------------------------------------------------------*/
std::vector<double> FindPoseDiff(std::vector<double> robot_pose, const std::vector<double>& destination) {
    double t = destination[2]-robot_pose[2];
    double x = std::cos(robot_pose[2])*(destination[0]-robot_pose[0])-std::sin(robot_pose[2])*(destination[1]-robot_pose[1]); //change
    double y = std::sin(robot_pose[2])*(destination[0]-robot_pose[0])+std::cos(robot_pose[2])*(destination[1]-robot_pose[1]); //change
    std::vector<double> pose_diff = {x, y, t};
    return pose_diff;
}
/*------------------------------------------------------------------------------------------------------------------------*/
std::vector<Segment> CompareSegments(std::vector<Segment>& old_segments, std::vector<Segment>& new_segments, const std::vector<double>& robot_pose) { /* H */
    std::vector<Segment> match;
    for( int i = 0; i < old_segments.size(); i++) {
        for ( int j = 0; j < new_segments.size(); j++) {
            double dxb = old_segments[i].p1[0]-new_segments[i].p1[0];
            double dxe = old_segments[i].p2[0]-new_segments[i].p2[0];
            double dyb = old_segments[i].p1[1]-new_segments[i].p1[1];
            double dye = old_segments[i].p2[1]-new_segments[i].p2[1];
            double li = std::sqrt((old_segments[i].p1[0]-old_segments[i].p2[0])*(old_segments[i].p1[0]-old_segments[i].p2[0]) + (old_segments[i].p1[1]-old_segments[i].p2[1])*(old_segments[i].p1[1]-old_segments[i].p2[1]));
            double lj = std::sqrt((new_segments[i].p1[0]-new_segments[i].p2[0])*(new_segments[i].p1[0]-new_segments[i].p2[0]) + (new_segments[i].p1[1]-new_segments[i].p2[1])*(new_segments[i].p1[1]-new_segments[i].p2[1]));
            double xdi = old_segments[i].dv[0];
            double ydi = old_segments[i].dv[1];
            double xdj = new_segments[j].dv[0];
            double ydj = new_segments[j].dv[1];
            double rb2 = dxb*dxb+dyb*dyb;
            double re2 = dxe*dxe+dye*dye;
            double angle = std::acos((xdi*xdj+ydi*ydj)/(li*lj));
            double xp = 0.5*(old_segments[i].p1[0] + new_segments[i].p2[0]);
            double yp = 0.5*(old_segments[i].p1[1] + new_segments[i].p2[1]);
            double xi = old_segments[i].p1[0]+(xdj*(xp-old_segments[i].p1[0])+ydi*(yp-new_segments[i].p1[1]))/(xdj*xdj-ydi*ydj)*xdj;
            double yi = new_segments[i].p1[1]+(xdj*(xp-old_segments[i].p1[0])+ydi*(yp-new_segments[i].p1[1]))/(xdj*xdj-ydi*ydj)*ydj;
            double d = std::sqrt((xi-xp)*(xi-xp)+(yi-yp)*(yi-yp));
            if ( (rb2 < 0.0001 && re2 < 0.0001) || (angle < 0.1*M_PI && d < 0.01) ) {
                match[i] = new_segments[j];
            }
        }
    }
    return match;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
std::vector<Segment> FindObjects(std::vector<Segment>& segments, const std::vector<double>& desired_pose, const Segment& cart) { /* I */ // Kijken of hier iets van evenwijdig aan cart toe te voegen is
    std::vector<Segment> certain_segments;
    double R = 3.5;
    for (auto & segment : segments) {
        if ((segment.p1[0]-desired_pose[0])*(segment.p1[0]-desired_pose[0])+(segment.p1[1]-desired_pose[1])*(segment.p1[1]-desired_pose[1])<R*R &&
        (segment.p2[0]-desired_pose[0])*(segment.p2[0]-desired_pose[0])+(segment.p2[1]-desired_pose[1])*(segment.p2[1]-desired_pose[1])<R*R &&
        std::abs(-std::sin(desired_pose[2])*segment.p1[0]+std::cos(desired_pose[2])*segment.p1[1]) > -std::sin(desired_pose[2])*segment.p1[0]-std::cos(desired_pose[2])*segment.p1[1] &&
        std::abs(-std::sin(desired_pose[2])*segment.p2[0]+std::cos(desired_pose[2])*segment.p2[1]) > -std::sin(desired_pose[2])*segment.p2[0]-std::cos(desired_pose[2])*segment.p2[1]){
            ROS_INFO("The complete segments will be visible while positioning");
            double sdvl = std::sqrt(segment.dv[0]*segment.dv[0]+segment.dv[1]*segment.dv[1]);
            double cdvl = std::sqrt(cart.dv[0]*cart.dv[0]+cart.dv[1]*cart.dv[1]);
            double alpha = std::acos((segment.dv[0]*cart.dv[0]+segment.dv[1]*cart.dv[1])/(sdvl*cdvl));
            ROS_INFO("The angle between two segments is calculated");
            if (std::abs(alpha) - M_PI_2 < 0.25*M_PI) { //huh
                ROS_INFO("Segment found that is about perpendicular to the cart segment");
                certain_segments.push_back(segment);
            }
        }
    }
    if (certain_segments.empty()) {
        ROS_INFO("No segments found that are perpendicular to the cart segment");
        Segment empty_segment;
        empty_segment.p1 = {0,0};
        empty_segment.p2 = {0,0};
        empty_segment.dv = {0,0};
        certain_segments.push_back(empty_segment);
    }
    return certain_segments;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
void SetTwistMessage(geometry_msgs::Twist& twist_msg, std::vector<double> speed) {
    twist_msg.linear.x = speed[0];
    twist_msg.linear.y = speed[1];
    twist_msg.angular.z = speed[2];
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
Segment FindCart(std::vector<Segment>& segments, Line area, Line facing) { /* J */
    Segment cart_segment;
    for (auto & segment : segments) {
        if ( area.p1[0] < segment.p1[0] < area.p2[0] && area.p1[1] < segment.p1[1] < area.p2[1] && area.p1[0] < segment.p2[0] < area.p2[0] && area.p1[1] < segment.p2[1] < area.p2[1]) {
            double xds = (1/(std::sqrt((segment.p2[0]-segment.p1[0])*(segment.p2[0]-segment.p1[0])+(segment.p2[1]-segment.p1[1])*(segment.p2[1]-segment.p1[1]))))*(segment.p2[0]-segment.p1[0]);
            double yds = (1/(std::sqrt((segment.p2[0]-segment.p1[0])*(segment.p2[0]-segment.p1[0])+(segment.p2[1]-segment.p1[1])*(segment.p2[1]-segment.p1[1]))))*(segment.p2[1]-segment.p1[1]);
            double dxs = segment.p1[0]-segment.p2[0];
            double dys = segment.p1[1]-segment.p2[1];
            double xdf = (1/(std::sqrt((facing.p2[0]-facing.p1[0])*(facing.p2[0]-facing.p1[0])+(facing.p2[1]-facing.p1[1])*(facing.p2[1]-facing.p1[1]))))*(facing.p2[0]-facing.p1[0]);
            double ydf = (1/(std::sqrt((facing.p2[0]-facing.p1[0])*(facing.p2[0]-facing.p1[0])+(facing.p2[1]-facing.p1[1])*(facing.p2[1]-facing.p1[1]))))*(facing.p2[1]-facing.p1[1]);
            double dxf = facing.p1[0]-facing.p2[0];
            double dyf = facing.p1[1]-facing.p2[1];
            double alpha = std::acos((xds*xdf+yds*ydf)/(std::sqrt(dxs*dxs+dys*dys)*std::sqrt(dxf*dxf+dyf*dyf)));
            if ( -M_PI_4 < alpha < M_PI_4 ) {
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
/*---------------------------------------------------------------------------------------------------------------------------*/
std::vector<Distance> FindDistance(const std::vector<double>& desired_pose, const std::vector<double>& robot_pose, const std::vector<Segment>& certain_segments) { /* M */
    Distance distance;
    std::vector<Distance> distances;
    for (auto & certain_segment : certain_segments) {
        distance.p1 = TransformPositionB(certain_segment.p1, desired_pose);
        distance.p2 = TransformPositionB(certain_segment.p2, desired_pose);
        ROS_INFO("Distance is (%f,%f) and (%f,%f)", distance.p1[0], distance.p1[1], distance.p2[0], distance.p2[1]);
        double xd = (1/(std::sqrt((distance.p2[0]-distance.p1[0])*(distance.p2[0]-distance.p1[0])+(distance.p2[1]-distance.p1[1])*(distance.p2[1]-distance.p1[1]))))*(distance.p2[0]-distance.p1[0]);
        double yd = (1/(std::sqrt((distance.p2[0]-distance.p1[0])*(distance.p2[0]-distance.p1[0])+(distance.p2[1]-distance.p1[1])*(distance.p2[1]-distance.p1[1]))))*(distance.p2[1]-distance.p1[1]);
        distance.dv = {xd, yd};
        distances.push_back(distance);
    }
    return distances;
}
/*-------------------------------------------------------------------------------------------------------------------------*/
std::vector<double> CalculateError(std::vector<Distance>& distances, std::vector<Segment>& localization_segments) {
    std::vector<double> error = {0,0,0};
    for (int i = 0; i < distances.size(); i++) {
        error[0] = error[0] + ((localization_segments[i].p1[0]-distances[i].p1[0]) + (localization_segments[i].p2[0]-distances[i].p2[0]))/double(2*distances.size());
        error[1] = error[1] + ((localization_segments[i].p1[1]-distances[i].p1[1]) + (localization_segments[i].p2[1]-distances[i].p2[1]))/double(2*distances.size());
        error[2] = error[2] + (std::acos(localization_segments[i].dv[0]*distances[i].dv[0]+localization_segments[i].dv[1]*distances[i].dv[1])/(std::sqrt(localization_segments[i].dv[0]*localization_segments[i].dv[0]+localization_segments[i].dv[1]*localization_segments[i].dv[1])*std::sqrt(distances[i].dv[0]*distances[i].dv[0]+distances[i].dv[1]*distances[i].dv[1])))/double(distances.size());
    }
    return error;
}
/*---------------------------------------------------------------------------------------------------------------------------*/
std::vector<double> FindDesiredPose(Segment cart_segment, double range_x, double range_y) { /* L */
    // range x is with normal vector and range y is with direction vector.
    double x = cart_segment.p2[0] - range_x*cart_segment.dv[1] + range_y*cart_segment.dv[0];
    double y = cart_segment.p2[1] + range_x*cart_segment.dv[0] + range_y*cart_segment.dv[1];
    double alpha = RotationDifference(cart_segment.dv);
    std::vector<double> pose = {x, y, alpha};
    return pose;
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void FindAreaPose(Line& facing, std::vector<double>& destination) { /* O */
    std::vector<double> dv;
    double dx = facing.p2[0]-facing.p1[0];
    double dy = facing.p2[1]-facing.p1[1];
    dv.push_back(1/(std::sqrt(dx*dx+dy*dy))*dx);
    dv.push_back(1/(std::sqrt(dx*dx+dy*dy))*dy);
    double facing_angle = RotationDifference(dv);
    destination.push_back((facing.p1[0]+facing.p2[0])/2);
    destination.push_back((facing.p1[1]+facing.p2[1])/2);
    destination.push_back(facing_angle);
}
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
void ResetSegmentFrame(std::vector<Segment>& segments, std::vector<double>& robot_pose) {
    for(auto & segment : segments) {
        segment.p1[0] = std::cos(robot_pose[2])*segment.p1[0]+std::sin(robot_pose[2])*segment.p1[1]+robot_pose[0];
        segment.p1[1] = std::sin(robot_pose[2])*segment.p1[0]-std::cos(robot_pose[2])*segment.p1[1]+robot_pose[1];
        segment.p2[0] = std::cos(robot_pose[2])*segment.p2[0]+std::sin(robot_pose[2])*segment.p2[1]+robot_pose[0];
        segment.p2[1] = std::sin(robot_pose[2])*segment.p2[0]-std::cos(robot_pose[2])*segment.p2[1]+robot_pose[1];
    }
}