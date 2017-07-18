#include "plotter/plotter.h"

namespace plotter {

Plotter::Plotter() :
    nh_(ros::NodeHandle())
{
    // Set up Publishers and Subscribers
    sub_tracks_ = nh_.subscribe("tracks3d", 1, &Plotter::cb_tracks, this);
    pub_tracks_ = nh_.advertise<visualization_msgs::MarkerArray>( "visualization/tracks", 1 );
}

// ----------------------------------------------------------------------------

void Plotter::cb_tracks(const visual_mtt::TracksPtr& msg)
{

    static std::vector<uint32_t> recent_ids;
    std::vector<uint32_t> current_ids;

    visualization_msgs::MarkerArray mtracks_add;
    for (int i=0; i<msg->tracks.size(); i++) {
        auto track = msg->tracks[i]; // for convenience
        
        visualization_msgs::Marker mtrack;

        // id book-keeping
        current_ids.push_back(track.id);
        recent_ids.erase(std::remove(recent_ids.begin(), recent_ids.end(), track.id), recent_ids.end());

        // new or existing tracks to publish
        mtrack.header = msg->header_update;
        mtrack.header.frame_id = "fcu";
        mtrack.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mtrack.action = visualization_msgs::Marker::ADD;
        mtrack.text = std::to_string(track.id);
        mtrack.id = track.id;
        mtrack.pose.position.x = track.position.x;
        mtrack.pose.position.y = track.position.y;
        mtrack.pose.position.z = track.position.z;
        mtrack.scale.x = 1;
        mtrack.scale.y = 1;
        mtrack.scale.z = 1;
        mtrack.color.r = 1.0;
        mtrack.color.g = 1.0;
        mtrack.color.b = 1.0;
        mtrack.color.a = 1.0;

        mtracks_add.markers.push_back(mtrack);
    }

    // Now delete the recent_ids that did not show up this iteration
    visualization_msgs::MarkerArray mtracks_delete;
    for (auto&& id : recent_ids) {
        visualization_msgs::Marker mtrack;
        mtrack.action = visualization_msgs::Marker::DELETE;
        mtrack.id = id;
        mtracks_delete.markers.push_back(mtrack);
    }

    // Save the current_ids for the next iteration
    recent_ids.swap(current_ids);

    pub_tracks_.publish(mtracks_add);
    pub_tracks_.publish(mtracks_delete);
}

}