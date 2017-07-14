#include "geolocator/geolocator.h"

namespace geolocator {

Geolocator::Geolocator()
{
    // ROS stuff
    image_transport::ImageTransport it(nh_);
    sub_cam_    = it.subscribeCamera("video", 1, &Geolocator::cb_cam, this);
    sub_pose_   = nh_.subscribe("pose", 1, &Geolocator::cb_pose, this);
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

void Geolocator::cb_pose(const geometry_msgs::PoseStampedPtr& msg)
{
    pose_ = msg;
}

// ----------------------------------------------------------------------------

void Geolocator::cb_tracks(const visual_mtt::TracksPtr& msg)
{
    // we cannot geolocate until we have the camera model
    if (!is_cam_matrix_set_) {
        ROS_WARN("Camera parameters not yet set -- won't perform geolocation.");
        return;
    }

    // we cannot geolocate until we have a robot pose
    if (pose_ == nullptr) {
        ROS_WARN("Pose not yet set -- won't perform geolocation.");
        return;
    }

    // skip if there are no tracks
    if (msg->tracks.size() == 0)
        return;

    // Extract the position measurement of each track
    Eigen::MatrixX3d measurements(msg->tracks.size(), 3);
    for (int i=0; i<msg->tracks.size(); i++) {
        auto track = msg->tracks[i]; // for convenience
        measurements.row(i) << track.position.x, track.position.y, 1;
    }

    //
    // Geolocate
    //

    // UAV Position
    double pn =  pose_->pose.position.x;
    double pe = -pose_->pose.position.y;
    double pd = -pose_->pose.position.z;

    // UAV Orientation: quaternion to euler angles
    double phi, theta, psi;
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(pose_->pose.orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(phi, theta, psi);

    // Gimbal Orientation (NASA has a fixed 45 degree camera)
    double az    = 0;
    double el    = -M_PI/4;
    double groll = 0;
    
    transform(measurements, pn, pe, pd, phi, theta, psi, groll, el, az);

    //
    // Publish 3D Tracks
    //

    // Create a new message and copy relevant information from old message
    visual_mtt::Tracks new_msg;
    new_msg.header_frame = msg->header_frame;
    new_msg.header_update.stamp = ros::Time::now();
    new_msg.util = msg->util;
    for (int i=0; i<msg->tracks.size(); i++) {
        visual_mtt::Track track;

        // Keep the same track id and inlier ratio
        track.id = msg->tracks[i].id;
        track.inlier_ratio = msg->tracks[i].inlier_ratio;

        track.position.x = measurements(i, 0);
        track.position.y = measurements(i, 1);
        track.position.z = measurements(i, 2);

        new_msg.tracks.push_back(track);
    }

    pub_tracks_.publish(new_msg);
}

// ----------------------------------------------------------------------------

void Geolocator::transform(Eigen::MatrixX3d& measurements,
        double pn, double pe, double pd,        // uav position north, east, down
        double phi, double theta, double psi,   // uav roll, pitch, yaw
        double gr, double gp, double gy)        // gimbal roll, pitch, yaw
{
    // Incoming measurements are assumed to be in the normalized image plane

    // how many measurements are there?
    uint32_t N = measurements.rows();

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
    Eigen::Matrix3d R_v_to_c = R_g_to_c() * R_b_to_g(gr, gp, gy) * R_v_to_b(phi, theta, psi);

    // Rotate camera frame unit vectors (ell_unit_c) into vehicle frame
    // (see the numerator of RHS of (13.18) in UAV book)
    Eigen::Matrix3Xd ell_unit_v = R_v_to_c.transpose() * pts.transpose();

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

// rotation from vehicle-2 to body frame
Eigen::Matrix3d Geolocator::R_v2_to_b(double phi)
{
  Eigen::Matrix3d R_v22b;
  R_v22b << 1,         0,        0,
            0,  cos(phi), sin(phi),
            0, -sin(phi), cos(phi);
  return R_v22b;
}

// ----------------------------------------------------------------------------

// rotation from vehicle-1 to vehicle-2 frame
Eigen::Matrix3d Geolocator::R_v1_to_v2(double theta)
{
  Eigen::Matrix3d R_v12v2;
  R_v12v2 << cos(theta), 0, -sin(theta),
                      0, 1,           0,
             sin(theta), 0,  cos(theta);
  return R_v12v2;
}

// ----------------------------------------------------------------------------

// rotation from vehicle to vehicle-1 frame
Eigen::Matrix3d Geolocator::R_v_to_v1(double psi)
{
  Eigen::Matrix3d R_v2v1;
  R_v2v1 <<  cos(psi), sin(psi), 0,
            -sin(psi), cos(psi), 0,
                    0,        0, 1;
  return R_v2v1;
}

// ----------------------------------------------------------------------------

// rotation from vehicle to body frame
Eigen::Matrix3d Geolocator::R_v_to_b(double phi, double theta, double psi)
{
  return R_v2_to_b(phi) * R_v1_to_v2(theta) * R_v_to_v1(psi);
}

// ----------------------------------------------------------------------------

Eigen::Matrix3d Geolocator::R_b_to_g(double r, double p, double y)
{
    Eigen::Matrix3d R_g2_to_g = R_v1_to_v2(p); // gimbal-2 to gimbal
    Eigen::Matrix3d R_g1_to_g2 = R_v2_to_b(r); // gimbal-1 to gimbal-2
    Eigen::Matrix3d R_b_to_g1 = R_v_to_v1(y);  // body     to gimbal-1

    return R_g2_to_g * R_g1_to_g2 * R_b_to_g1;
}

// ----------------------------------------------------------------------------

Eigen::Matrix3d Geolocator::R_g_to_c()
{
    // Euler Rotation from the gimbal frame to camera frame
    Eigen::Matrix3d R;
    R << 0, 1, 0,
         0, 0, 1,
         1, 0, 0;

    return R;
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