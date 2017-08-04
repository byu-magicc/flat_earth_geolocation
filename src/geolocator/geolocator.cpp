#include "geolocator/geolocator.h"

namespace geolocator {

Geolocator::Geolocator()
{
    // ROS stuff
    image_transport::ImageTransport it(nh_);
    sub_cam_    = it.subscribeCamera("camera/image_raw", 1, &Geolocator::cb_cam, this);
    sub_tracks_ = nh_.subscribe("tracks", 1, &Geolocator::cb_tracks, this);
    pub_tracks_ = nh_.advertise<visual_mtt::Tracks>("tracks3d", 1);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void Geolocator::cb_cam(const sensor_msgs::ImageConstPtr& frame, const sensor_msgs::CameraInfoConstPtr& cinfo)
{
    // convert rosmsg vectors to Eigen::Matrix3d
    for(int i=0; i<9; i++)
        cam_matrix_(i/3, i%3) = cinfo->K[i];

    is_cam_matrix_set_ = true;

    ROS_WARN("Got camera intrinsic parameters -- shutting down CameraSubscriber");

    // unregister the subscriber
    sub_cam_.shutdown();
}

// ----------------------------------------------------------------------------

void Geolocator::cb_tracks(const visual_mtt::TracksPtr& msg)
{
    // we cannot geolocate until we have the camera model
    if (!is_cam_matrix_set_) {
        ROS_WARN("Camera parameters not yet set -- won't perform geolocation.");
        return;
    }

    // Find the transform between the `source` and the camera. This transform
    // puts measurements from the camera frame into the `source` frame.

    tf::StampedTransform T;
    std::string frame_source = "map_ned";
    try {
        // Get the most recent transform
        tf_listener_.lookupTransform(frame_source, "camera", ros::Time(0), T);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("[geolocator]: %s", ex.what());
        ROS_WARN("Pose not yet set -- won't perform geolocation.");
        return;
    }


    // Create a new tracks message for 3D points
    visual_mtt::Tracks new_msg;

    // skip if there are no tracks
    if (msg->tracks.size() == 0) {
        pub_tracks_.publish(new_msg);
        return;
    }

    // Extract the position measurement of each track
    Eigen::MatrixX3d measurements(msg->tracks.size(), 3);
    for (int i=0; i<msg->tracks.size(); i++) {
        auto track = msg->tracks[i]; // for convenience
        measurements.row(i) << track.position.x, track.position.y, 1;
    }

    //
    // Geolocate
    //
    
    transform(measurements, T);

    //
    // Publish 3D Tracks
    //

    // Create a new message and copy relevant information from old message
    new_msg.header_frame = msg->header_frame;
    new_msg.header_update.stamp = ros::Time::now();
    new_msg.header_update.frame_id = frame_source;
    new_msg.util = msg->util;
    for (int i=0; i<msg->tracks.size(); i++) {
        visual_mtt::Track track;

        // Keep the same track id and inlier ratio
        track.id = msg->tracks[i].id;
        track.inlier_ratio = msg->tracks[i].inlier_ratio;

        // Measurements are made in the `frame_source`
        track.position.x = measurements(i, 0);
        track.position.y = measurements(i, 1);
        track.position.z = measurements(i, 2);

        new_msg.tracks.push_back(track);
    }

    pub_tracks_.publish(new_msg);
}

// ----------------------------------------------------------------------------

void Geolocator::transform(Eigen::MatrixX3d& measurements, tf::StampedTransform T)
{
    // Incoming measurements are assumed to be in the normalized image plane

    // how many measurements are there?
    uint32_t N = measurements.rows();

    // UAV Position
    double pn = T.getOrigin().x();
    double pe = T.getOrigin().y();
    double pd = T.getOrigin().z();

    // ------------------------------------------------------------------------
    // Compute equation (13.9) in UAV book      (pts == ell_unit_c)
    // ------------------------------------------------------------------------

    // norm each row of the pts matrix
    Eigen::VectorXd F = measurements.rowwise().norm();

    // Make the Nx1 vector an Nx3 matrix so we can element-wise divide
    Eigen::MatrixX3d Fdiv = F.replicate(1, 3);

    // divide to normalize and create unit vectors in the camera frame
    Eigen::MatrixXd pts = (measurements.array() / Fdiv.array());

    // ========================================================================


    // Create the rotation from vehicle frame to camera frame
    // Eigen::Matrix3d R_c_to_v = (R_g_to_c() * R_b_to_g(gr, gp, gy) * R_v_to_b(phi, theta, psi)).transpose();
    Eigen::Affine3d e;
    tf::transformTFToEigen(T, e);
    Eigen::Matrix3d R_c_to_v = e.rotation();

    // Rotate camera frame unit vectors (ell_unit_c) into vehicle frame
    // (see the numerator of RHS of (13.18) in UAV book)
    Eigen::Matrix3Xd ell_unit_v = R_c_to_v * pts.transpose();

    // ------------------------------------------------------------------------
    // Compute equation (13.17) in UAV book     (target's range estimate)
    // ------------------------------------------------------------------------

    // cosine of the angle between the the ki axis and ell_unit_v
    // (the unit vector that points at each target)
    //
    // Instead of creating a ki_unit << (0 0 1) and doing a dot product,
    // just grab the last row of ell_unit_v, since that's what
    // <ki_unit, ell_unit_v> would have done anyways.
    Eigen::VectorXd cos_psi = ell_unit_v.row(2);

    // The UAV's height above ground (well, actually the gimbal/camera)
    float h = -pd;

    // 1 x N vector
    Eigen::VectorXd L = h / cos_psi.array();

    // ========================================================================


    // ------------------------------------------------------------------------
    // Compute equation (13.18) in UAV book   (target's pos in intertial frame)
    // ------------------------------------------------------------------------

    // Construct a vector for the UAV's inertial position
    Eigen::Vector3d tmp; tmp << pn, pe, pd;
    Eigen::Matrix3Xd P_mav_i = tmp.replicate(1, N);

    // Replicate L vector to a matrix so dimensions are happy
    Eigen::Matrix3Xd Lmat = L.transpose().replicate(3, 1);

    // Based on current attitude, find offset from the body to vehicle frame.
    // This is because the camera (on the gimbal) sees targets, not the UAV.
    // Mat offset = rot_v2b(phi,theta,psi).t() * cv::repeat(d_b2g(), 1, N);
    Eigen::Matrix3Xd offset = Eigen::Matrix3Xd::Zero(3, N);

    // Based on the line-of-sight vector, find the inertial object points
    // Mat P_obj_i = P_mav_i + offset + L.mul(ell_unit_v);
    Eigen::Matrix3Xd P_obj_i = P_mav_i + offset + (Lmat.array() * ell_unit_v.array()).matrix();

    // ========================================================================

    // Copy object points back to measurements for the caller
    measurements = P_obj_i.transpose();
}

// ----------------------------------------------------------------------------

Eigen::Vector3d Geolocator::t_b_to_g()
{
    // translation from body to gimbal
    // TODO: abstract out and make a parameter
    Eigen::Vector3d t;
    t << 0.2,
         0.0,
         0.1;

    return t;
}

}