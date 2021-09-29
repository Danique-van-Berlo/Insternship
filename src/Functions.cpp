#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "Functions.h"
#include "Classes.h"

void MakingPointCloud(const std::vector<double>& robot_pose, std::vector<std::vector<double>> pointcloud, sensor_msgs::LaserScan& laser) { /* A */
    pointcloud.clear();
    for (int i = 0; i < laser.ranges.size(); i++) {
        double alpha = laser.angle_min + (laser.angle_increment * float(i));
        std::vector<double> point_int = {laser.ranges[i] * std::cos(alpha)+0.38, laser.ranges[i] * std::sin(alpha)};
        std::vector<double> point = TransformPosition(point_int, robot_pose);
        pointcloud.push_back(point);
    }
}
/*----------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------------------------------------------------*/

void MakingLineSegments(std::vector<Segment>& segments, std::vector<std::vector<double>> pointcloud, int i, int n, double lambda_old) { /* B */
    Segment segment;
    //ROS_INFO("begin point %i, end point %i", i, i+n+5);
    std::vector<double> p1 = pointcloud[i]; //begin point
    std::vector<double> p2 = pointcloud[i+5+n]; //end point
    std::vector<double> p3 = pointcloud[i+4+n]; //one before end point
    double lambda = 0;
    double xd = (1/(std::sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]))))*(p2[0]-p1[0]);
    double yd = (1/(std::sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]))))*(p2[1]-p1[1]);
    for (int j = 0; j < 5+n; j++) {
        lambda = lambda + (std::abs(xd*(-pointcloud[i+j][1]+p1[1])+yd*(pointcloud[i+j][0]-p1[0])))/(5+n);
    }
   // ROS_INFO("lambda: %f", lambda);

    if (i+6+n < pointcloud.size()) { //If it is the last point turn into a segment.
        if (lambda < 0.0001) { // segment binnen threshold
            //ROS_INFO("%s", "Add next point");
            MakingLineSegments(segments,pointcloud, i, n + 1, lambda);
        } else if (lambda >= 0.0001) { /* segment buiten threshold */
            //ROS_INFO("%s", "Save segments: in threshold");
            segment.p1 = p1;
            segment.p2 = p3;
            segment.sigma = 1 - lambda_old / 0.0001;
            xd = (1 / (std::sqrt((p3[0] - p1[0]) * (p3[0] - p1[0]) + (p3[1] - p1[1]) * (p3[1] - p1[1])))) *
                 (p3[0] - p1[0]);
            yd = (1 / (std::sqrt((p3[0] - p1[0]) * (p3[0] - p1[0]) + (p3[1] - p1[1]) * (p3[1] - p1[1])))) *
                 (p3[1] - p1[1]);
            segment.dv = {xd, yd};
            //ROS_INFO("push back segment sigma: %f", segment.sigma);
            segments.push_back(segment);
            //ROS_INFO("number of segments: %zu", segments.size());
            MakingLineSegments(segments,pointcloud, i + n + 4, 0, 0);
        }
    } else {
        segment.p1 = p1;
        segment.p2 = p2;
        segment.sigma = 1 - lambda / 0.03;
        segment.dv = {xd, yd};
        segments.push_back(segment);
    }
}

/*------------------------------------------------------------------------------------------------------------------------*/

double RotationDifference(std::vector<double> p1, std::vector<double> p2) { /* E */
    double y1 = p1[1];
    double dx = p2[0]-p1[0];
    double dy = p2[1]-p1[1];
    double alpha;
    if (y1 < 0) {
        alpha = -std::acos((dx)/(std::sqrt(dx*dx+dy*dy)));
    } else {
        alpha = std::acos((dx)/(std::sqrt(dx*dx+dy*dy)));
    }
    return alpha;
}

/*-----------------------------------------------------------------------------------------------------------------------*/

std::vector<double> FindingEntrance(std::vector<Segment> segments, const std::vector<double>& robot_pose) { /* C */
    std::vector<double> pose = {0,0,0};
    for (int i =0; i < segments.size(); i++) {
        for (int j=i+1; j < (segments.size()); j++) {
            double w = std::sqrt((segments[i].p1[0]-segments[j].p2[0])*(segments[i].p1[0]-segments[j].p2[0])+(segments[i].p1[1]-segments[j].p1[1])*(segments[i].p2[1]-segments[j].p2[1]));
            if ((0.82 < w < 0.89 || 0.82 < 0.5*w < 0.89) && (segments[i].dv[0]-0.05)<segments[j].dv[0]<(segments[i].dv[0]+0.05) && (segments[i].dv[1]-0.05)<segments[j].dv[1]<(segments[i].dv[1]+0.05) && (segments[i].p2[0] != segments[j].p1[0] && segments[i].p2[1] != segments[j].p1[1])) {
                double x = 0.5*(segments[i].p2[0]-segments[j].p1[0]);
                double y = 0.5*(segments[i].p2[1]-segments[j].p1[1]);
                double alpha = RotationDifference(segments[i].p1,segments[i].p2)+M_PI_2; //nog ff checken
                pose = {x, y, alpha};
            }
        }
    }
    return pose;
}
/*-----------------------------------------------------------------------------------------------------------------------*/
std::vector<double> TransformPosition(std::vector<double> old_point, std::vector<double> rel_pose) { /* D */
    double x = std::sin(rel_pose[2])*old_point[0]+std::cos(rel_pose[2])*old_point[1] + rel_pose[0];
    double y = std::cos(rel_pose[2])*old_point[0]-std::sin(rel_pose[2])*old_point[1] + rel_pose[1];
    std::vector<double> point = {x, y};
    return point;

}
/*-----------------------------------------------------------------------------------------------------------------------*/
std::vector<double> TransformPositionB(std::vector<double> old_point, std::vector<double> rel_pose) { /* D */
    double x = std::cos(rel_pose[2])*old_point[0]-std::sin(rel_pose[2])*old_point[1] + rel_pose[0];
    double y = std::sin(rel_pose[2])*old_point[0]+std::cos(rel_pose[2])*old_point[1] + rel_pose[1];
    std::vector<double> point = {x, y};
    return point;

}
/*------------------------------------------------------------------------------------------------------------------------*/
bool Wait(std::vector<std::vector<double>> pointcloud, double b, std::vector<double> pose_diff) { /* G */
    double d = 0.5;
    double delta = 0.1;
    int n = 0;
    bool wait;
    for (auto & i : pointcloud) {
        if (std::abs(i[1]) > d*i[0]/(2*b+delta) && pose_diff[0] > 0.5 && pose_diff[1] > 0.5) {
            n += 1;
        }
    }
    if (n > 5) {
        wait = true;
        ROS_INFO("Now waiting");
    } else {
        wait = false;
        ROS_INFO("Now driving");
    }
    return wait;
}
/*------------------------------------------------------------------------------------------------------------------------*/
std::vector<double> CalculateSpeed(std::vector<double> pose_diff, std::vector<double> old_speed, double f, sensor_msgs::LaserScan& laser, double b, std::vector<std::vector<double>> pointcloud) { /* F */
    /* Velocity constraints */
    double vmax = 3.8;
    double wmax = (3*M_PI)/32;
    /* Acceleration constraints */
    double amax = 1.68;
    double awmax = (3*M_PI)/68;
    MakingPointCloud({0,0,0}, pointcloud, laser);
    bool wait = Wait(pointcloud, b, pose_diff);
    std::vector<double> speed;
    if ((-0.05*M_PI < pose_diff[2] < 0.05*M_PI) && (!wait)) {
        ROS_INFO("Calculate the speed in theta for %f", pose_diff[2]);
        double vt = f*pose_diff[2];
        ROS_INFO("Initial speed %f", vt);
        double at = f*(vt-old_speed[2]);
        ROS_INFO("Initial acceleration %f", at);
        if (at > awmax){
            at = awmax;
        } else if (at < -awmax){
            ROS_INFO("max acceleration %f", -awmax);
            at = -awmax;
            ROS_INFO("max acceleration %f", at);
        }
        ROS_INFO("Adjusted acceleration %f", at);
        ROS_INFO("old speed %f", old_speed[2]);
        vt = old_speed[2] + at/f;
        ROS_INFO("Intermediate speed %f", vt);
        if (vt > wmax){
            vt = wmax;
        } else if (vt < -wmax){
            vt = -wmax;
        }
        ROS_INFO("Final speed %f", vt);
        if (vt*vt/awmax >= pose_diff[2]) {
            at = -awmax;
            vt = old_speed[2] + at/f;
        }
        speed = {0,0,vt};
    } else if ((-0.08 < pose_diff[1] < 0.08) && (!wait)) {
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
        /* if (vy*vy/amax >= pose_diff[1]) { Find solution for this
            ay = -amax;
            vy = old_speed[1] + ay/f;
        } */
        speed = {0,vy,0};
    } else if ((-0.02 < pose_diff[0] < 0.02) && (!wait)) {
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
        if (vx*vx/amax >= pose_diff[0]) {
            ax = -amax;
            vx = old_speed[0] + ax/f;
        }
        speed = {vx,0,0};
    } else {
        speed = {0, 0, 0};
    }
    return speed;
}
/*----------------------------------------------------------------------------------------------------------------------*/
std::vector<double> FindPoseDiff(std::vector<double> robot_pose, std::vector<double> destination) {
    double x = destination[0]-robot_pose[0];
    double y = destination[1]-robot_pose[1];
    double t = destination[2]-robot_pose[2];
    std::vector<double> pose_diff = {x, y, t};
    return pose_diff;
}
/*------------------------------------------------------------------------------------------------------------------------*/
std::vector<Segment> CompareSegments(std::vector<Segment> old_segments, std::vector<Segment> new_segments, const std::vector<double>& robot_pose) { /* H */
    std::vector<Segment> match;
    for( int i = 0; i < old_segments.size(); i++) {
        for ( int j = 0; j < new_segments.size(); j++) {
            std::vector<double> p1 = TransformPosition(new_segments[i].p1, robot_pose);
            std::vector<double> p2 = TransformPosition(new_segments[i].p2, robot_pose);
            double dxb = old_segments[i].p1[0]-p1[0];
            double dxe = old_segments[i].p2[0]-p2[0];
            double dyb = old_segments[i].p1[1]-p1[1];
            double dye = old_segments[i].p2[1]-p2[1];
            double li = std::sqrt((old_segments[i].p1[0]-old_segments[i].p2[0])*(old_segments[i].p1[0]-old_segments[i].p2[0]) + (old_segments[i].p1[1]-old_segments[i].p2[1])*(old_segments[i].p1[1]-old_segments[i].p2[1]));
            double lj = std::sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]));
            double xdi = old_segments[i].dv[0];
            double ydi = old_segments[i].dv[1];
            double xdj = new_segments[j].dv[0];
            double ydj = new_segments[j].dv[1];
            double rb2 = dxb*dxb+dyb*dyb;
            double re2 = dxe*dxe+dye*dye;
            double angle = std::acos((xdi*xdj+ydi*ydj)/(li*lj));
            double xp = 0.5*(old_segments[i].p1[0] + p2[0]);
            double yp = 0.5*(old_segments[i].p1[1] + p2[1]);
            double xi = old_segments[i].p1[0]+(xdj*(xp-old_segments[i].p1[0])+ydi*(yp-p1[1]))/(xdj*xdj-ydi*ydj)*xdj;
            double yi = p1[1]+(xdj*(xp-old_segments[i].p1[0])+ydi*(yp-p1[1]))/(xdj*xdj-ydi*ydj)*ydj;
            double d = std::sqrt((xi-xp)*(xi-xp)+(yi-yp)*(yi-yp));
            if ( (rb2 < 0.0001 && re2 < 0.0001) || (angle < 0.1*M_PI && d < 0.01) ) {
                match[i] = new_segments[j];
            }
        }
    }
    return match;
}
/*-------------------------------------------------------------------------------------------------------------------------------*/
std::vector<Segment> CertaintyFilter(std::vector<Segment> segments, double amount, const Segment& cart) { /* I */ // Kijken of hier iets van evenwijdig aan cart toe te voegen is
    std::vector<Segment> certain_segments;
    int n;
    for (int i=0; i < amount; i++) {
        n = FindHighestCertainty(segments, cart);
        certain_segments.push_back(segments[n]);
        segments[n].sigma = 0;
    }
    return certain_segments;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
int FindHighestCertainty(std::vector<Segment> segments, Segment cart) {
    double sigma = 0;
    int n;
    for(int i=0; i < segments.size(); i++) {
        double alpha = std::acos((segments[i].dv[0]*cart.dv[0]+segments[i].dv[1]*cart.dv[1])/(std::sqrt(segments[i].dv[0]*segments[i].dv[0]+segments[i].dv[1]*segments[i].dv[1])*std::sqrt(cart.dv[0]*cart.dv[0]+cart.dv[1]*cart.dv[1])));
        if (segments[i].sigma > sigma && std::abs(alpha) < M_PI_4) {
            sigma = segments[i].sigma;
            n = i;
        }
    }
    return n;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
Segment FindCart(std::vector<Segment> segments, Line area, Line facing) { /* J */
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
    return cart_segment;
}
/*---------------------------------------------------------------------------------------------------------------------------*/
std::vector<Distance> FindDistance(const std::vector<double>& desired_pose, const std::vector<Segment>& certain_segments) { /* M */
    Distance distance;
    std::vector<Distance> distances;
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
/*-------------------------------------------------------------------------------------------------------------------------*/
std::vector<double> CalculateError(std::vector<Distance> distances, std::vector<Segment> localization_segments) {
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
    double dx = cart_segment.p2[0]-cart_segment.p1[0];
    double dy = cart_segment.p2[1]-cart_segment.p1[1];
    double mu = 1/(std::sqrt(dx*dx+dy*dy));
    double x = cart_segment.p2[0] + range_x*mu*dx + range_y*mu*dy;
    double y = cart_segment.p2[1] + range_x*mu*dy - range_y*mu*dx;
    double alpha = RotationDifference(cart_segment.p1, cart_segment.p2);
    std::vector<double> pose = {x, y, alpha};
    return pose;
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void FindAreaPose(Line facing, std::vector<double>& destination) { /* O */
    double facing_angle = RotationDifference(facing.p1, facing.p2);
    if (std::sqrt(facing.p1[0]*facing.p1[0]+facing.p1[1]*facing.p1[1]) < std::sqrt(facing.p2[0]*facing.p2[0]+facing.p2[1]*facing.p2[1]) ) {
        ROS_INFO("Point 1 was closer");
        destination.push_back(facing.p1[0]);
        destination.push_back(facing.p1[1]);
        destination.push_back(facing_angle);
    } else {
        ROS_INFO("Point 2 was closer");
        destination.push_back(facing.p1[0]);
        destination.push_back(facing.p1[1]);
        destination.push_back(facing_angle);
    }
}