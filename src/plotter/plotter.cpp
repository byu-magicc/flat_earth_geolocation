#include "plotter/plotter.h"

namespace plotter {

Plotter::Plotter() :
    nh_(ros::NodeHandle()),
    nh_private_("~")
{
    // Set up Publishers and Subscribers
    sub_uav_    = nh_.subscribe("uav_pose", 1, &Plotter::cb_uav, this);
    sub_tracks_ = nh_.subscribe("tracks3d", 1, &Plotter::cb_tracks, this);

    pub_uav_    = nh_.advertise<geometry_msgs::PoseStamped>( "visualization/uav", 1 );
    pub_tracks_ = nh_.advertise<visualization_msgs::MarkerArray>( "visualization/tracks", 1 );
    // mav0_label_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization/mav0/label", 1 );
    // atv0_label_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization/atv0/label", 1 );
    // atv1_label_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization/atv1/label", 1 );
}

// ----------------------------------------------------------------------------

void Plotter::cb_uav(const geometry_msgs::PoseStampedPtr& msg)
{
    // get mav0 pose and convert to NWU for display
    geometry_msgs::PoseStamped uav_msg;
    uav_msg.header = msg->header;
    uav_msg.pose.position.x = msg->pose.position.x;
    uav_msg.pose.position.y = msg->pose.position.y;
    uav_msg.pose.position.z = msg->pose.position.z;
    uav_msg.pose.orientation.w = msg->pose.orientation.w;
    uav_msg.pose.orientation.x = msg->pose.orientation.x;
    uav_msg.pose.orientation.y = msg->pose.orientation.y;
    uav_msg.pose.orientation.z = msg->pose.orientation.z;

    // Publish the messages
    pub_uav_.publish(uav_msg);
}

// ----------------------------------------------------------------------------

void Plotter::cb_tracks(const visual_mtt::TracksPtr& msg)
{
    visualization_msgs::MarkerArray mtracks;

    for (int i=0; i<msg->tracks.size(); i++) {
        auto track = msg->tracks[i]; // for convenience
        
        visualization_msgs::Marker mtrack;

        mtrack.header = msg->header_update;
        mtrack.header.frame_id = "fcu";
        mtrack.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        // mtrack.action = visualization_msgs::Marker::ADD;
        mtrack.text = std::to_string(track.id);
        mtrack.id = track.id;
        mtrack.pose.position.x = track.position.x;
        mtrack.pose.position.y = track.position.y;
        mtrack.pose.position.z = track.position.z;
        mtrack.scale.x = 1;
        mtrack.scale.y = 1;
        mtrack.scale.z = 1;
        mtrack.color.a = 1.0; // Don't forget to set the alpha!
        mtrack.color.r = 1.0;
        mtrack.color.g = 1.0;
        mtrack.color.b = 1.0;



        mtracks.markers.push_back(mtrack);
    }

    pub_tracks_.publish(mtracks);
}

}