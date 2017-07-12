#pragma once

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>

namespace geolocator {

    class Geolocator
    {
    public:
        Geolocator(cv::Ptr<CameraModel> camModel);

        void transform(std::vector<cv::Point2f>& measurements,
                                double pn, double pe, double pd,
                                double phi, double theta, double psi,
                                double gr, double gp, double gy);

    private:
        cv::Ptr<CameraModel> _cameraModel;

        cv::Mat rot_v2b(double phi, double theta, double psi);
        cv::Mat rot_b2g(double r, double p, double y);
        cv::Mat rot_g2c();
        cv::Mat d_b2g();
    };

}