#pragma once

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <eigen3/Eigen/Dense>

#include "visual_mtt/Tracks.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Point.h"

namespace geolocator {

    class Geolocator
    {
    public:
        Geolocator();

    private:
        // ROS
        ros::NodeHandle nh_;
        image_transport::CameraSubscriber sub_cam_;
        ros::Subscriber sub_tracks_;
        ros::Subscriber sub_imu_;
        ros::Publisher pub_tracks_;

        // ROS callbacks
        void cb_cam(const sensor_msgs::ImageConstPtr& frame, const sensor_msgs::CameraInfoConstPtr& cinfo);
        void cb_tracks(const visual_mtt::TracksPtr& msg);
        void cb_imu(const visual_mtt::TracksPtr& msg);

        // geolocation algorithm
        void transform(std::vector<geometry_msgs::Point>& measurements,
                double pn, double pe, double pd,        // uav position north, east, down
                double phi, double theta, double psi,   // uav roll, pitch, yaw
                double gr, double gp, double gy);       // gimbal roll, pitch, yaw

        // Rotations for coordinate frame conversions
        Eigen::Matrix3d R_v2_to_b(double phi);
        Eigen::Matrix3d R_v1_to_v2(double theta);
        Eigen::Matrix3d R_v_to_v1(double psi);
        Eigen::Matrix3d R_v_to_b(double phi, double theta, double psi);
        Eigen::Matrix3d R_g_to_b(double r, double p, double y);
        Eigen::Matrix3d R_g_to_c();
        Eigen::Vector3d t_b_to_g();
    };

}