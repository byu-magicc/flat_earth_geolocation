#pragma once

#include <iostream>
#include <stdint.h>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen3/Eigen/Dense>

#include "visual_mtt/Tracks.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

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
        ros::Publisher pub_tracks_;

        // Camera parameters
        Eigen::Matrix3d cam_matrix_;
        bool is_cam_matrix_set_ = false;

        // ROS tf listener and broadcaster
        tf::TransformListener tf_listener_;

        // ROS callbacks
        void cb_cam(const sensor_msgs::ImageConstPtr& frame, const sensor_msgs::CameraInfoConstPtr& cinfo);
        void cb_tracks(const visual_mtt::TracksPtr& msg);

        // geolocation algorithm
        void transform(Eigen::MatrixX3d& measurements, tf::StampedTransform T);

        // Rotations for coordinate frame conversions
        Eigen::Vector3d t_b_to_g();
    };

}