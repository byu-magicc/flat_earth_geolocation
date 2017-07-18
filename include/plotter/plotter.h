#pragma once

#include <stdint.h>
#include <algorithm>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visual_mtt/Tracks.h>

namespace plotter {

    class Plotter
    {
    public:
        Plotter();

    private:
        // Node handles, publishers, subscribers
        ros::NodeHandle nh_;
        ros::Subscriber sub_tracks_;
        ros::Publisher pub_tracks_;

        // Functions
        void cb_tracks(const visual_mtt::TracksPtr& msg);
        
    };

}