#include "geolocator/geolocator.h"

namespace geolocator {

Geolocator::Geolocator()
{
    // ROS stuff
    image_transport::ImageTransport it(nh_);
    sub_cam_    = it.subscribeCamera("video", 10, &Geolocator::cb_cam, this);
    sub_tracks_ = nh_.subscribe("tracks", 1, &Geolocator::cb_tracks, this);
    sub_imu_    = nh_.subscribe("imu", 1, &Geolocator::cb_imu, this);
    pub_tracks_ = nh_.advertise<visual_mtt::Tracks>("tracks3d", 1);
}

// ----------------------------------------------------------------------------

void Geolocator::cb_cam(const sensor_msgs::ImageConstPtr& frame, const sensor_msgs::CameraInfoConstPtr& cinfo)
{
    
}

// ----------------------------------------------------------------------------

void Geolocator::cb_tracks(const visual_mtt::TracksPtr& msg)
{

}

// ----------------------------------------------------------------------------

void Geolocator::cb_imu(const visual_mtt::TracksPtr& msg)
{

}

// ----------------------------------------------------------------------------

void Geolocator::transform(std::vector<geometry_msgs::Point>& measurements,
        double pn, double pe, double pd,        // uav position north, east, down
        double phi, double theta, double psi,   // uav roll, pitch, yaw
        double gr, double gp, double gy)        // gimbal roll, pitch, yaw
{
    // // shift measurements so that the optical axis is through the center
    // for (auto&& point : measurements) {
    //     point.x -= _cameraModel->optical_x_offset;
    //     point.y -= _cameraModel->optical_y_offset;
    // }

    // // how many measurements are there?
    // unsigned int N = measurements.size();

    // // Convert Point2f vector to CV_32FC2 matrix, then reshape to CV_32FC1
    // Mat tmp = Mat(measurements).reshape(1);

    // // Add a third column for the focal length to the matrix, called pts
    // Mat pts(N, 3, CV_32FC1, Scalar(_cameraModel->focal_length_px));
    // tmp.copyTo(pts(Rect(0, 0, tmp.cols, tmp.rows))); // deep copy


    // // ------------------------------------------------------------------------
    // // Compute equation (13.9) in UAV book      (pts == ell_unit_c)
    // // ------------------------------------------------------------------------

    // // norm each row of the pts matrix
    // Mat F = pts.mul(pts);
    // cv::reduce(F, F, 1, CV_REDUCE_SUM); // sum along rows, Nx1 matrix
    // cv::sqrt(F, F);

    // // Make the Nx1 matrix an Nx3 so we can divide
    // F = cv::repeat(F, 1, 3);

    // // divide to normalize and create unit vectors in the camera frame
    // cv::divide(pts, F, pts);

    // // ========================================================================


    // // Create the rotation from vehicle frame to camera frame
    // // TODO: Use quaternions for this?
    // Mat rot_v2c = rot_g2c() * rot_b2g(gr, gp, gy) * rot_v2b(phi, theta, psi);

    // // Rotate camera frame unit vectors (ell_unit_c) into vehicle frame
    // // (see the numerator of RHS of (13.18) in UAV book)
    // Mat ell_unit_v = rot_v2c.t() * pts.t();

    // // ------------------------------------------------------------------------
    // // Compute equation (13.17) in UAV book     (target's range estimate)
    // // ------------------------------------------------------------------------

    // // The UAV's height above ground (well, actually the gimbal/camera)
    // float h = -(pd + d_b2g().at<float>(2));

    // // cosine of the angle between the the ki axis and ell_unit_v (the unit
    // // vector that points at each target on the ground; using flat earth)
    // vector<float> cos_psi;

    // // Instead of creating a ki_unit << (0 0 1) and doing a dot product,
    // // just grab the last row of ell_unit_v, since that's what
    // // <ki_unit, ell_unit_v> would have done anyways.
    // ell_unit_v.row(2).copyTo(cos_psi);

    // Mat L;
    // cv::divide(h, cos_psi, L);

    // // ========================================================================


    // // ------------------------------------------------------------------------
    // // Compute equation (13.18) in UAV book   (target's pos in intertial frame)
    // // ------------------------------------------------------------------------

    // // Construct a vector for the UAV's inertial position
    // Mat P_mav_i = (Mat_<float>(3, 1) << pn, pe, pd);

    // // Repeat matrices so dimensions are happy
    // P_mav_i = cv::repeat(P_mav_i, 1, N);
    // L = cv::repeat(L, 3, 1);

    // // Based on current attitude, find offset from the body to vehicle frame.
    // // This is because the camera (on the gimbal) sees targets, not the UAV.
    // Mat offset = rot_v2b(phi,theta,psi).t() * cv::repeat(d_b2g(), 1, N);

    // // Based on the line-of-sight vector, find the inertial object points
    // Mat P_obj_i = P_mav_i + offset + L.mul(ell_unit_v);

    // // ========================================================================


    // // Process matrix back into a vector of Point2f (since flat-earth model)
    // P_obj_i = P_obj_i.t();

    // Mat sub_pts = P_obj_i(Rect(0,0,P_obj_i.cols-1,P_obj_i.rows));
    // sub_pts = sub_pts.reshape(2);

    // sub_pts.copyTo(measurements);
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

Eigen::Matrix3d Geolocator::R_g_to_b(double r, double p, double y)
{
    Eigen::Matrix3d R_g2_to_b = R_v1_to_v2(p); // gimbal-2 to body
    Eigen::Matrix3d R_g1_to_g2 = R_v2_to_b(r); // gimbal-1 to gimbal-2
    Eigen::Matrix3d R_g_to_g1 = R_v_to_v1(y);  // gimbal   to gimbal-1

    return R_g2_to_b * R_g1_to_g2 * R_g_to_g1;
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