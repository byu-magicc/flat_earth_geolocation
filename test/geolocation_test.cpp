#include <iostream>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>

#include "cameraModel.h"
#include "geolocator.h"

namespace visual_mtt {

struct params_t {
    double pn, pe, pd;
    double phi, theta, psi;
    double az, el;
    double f;
} ;

class GeolocationTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    // Set our params (from MATLAB)
    params.pn = -5;
    params.pe = 1;
    params.pd = -10;
    params.phi = 0;
    params.theta = 0;
    params.psi = 0;
    params.az = 0;
    params.el = -1.0472;

    // Camera model params
    cameraModel = cv::Ptr<CameraModel>(new CameraModel());
    cameraModel->focal_length_px = 2666.66666666667;
    cameraModel->im_width_px = 1600;
    cameraModel->im_height_px = 1200;
    cameraModel->optical_x_offset = 1600/2;
    cameraModel->optical_y_offset = 1200/2;

    // What is "close enough" for float equality?
    abs_err = 0.0001;

    // Create inputs
    measurements = {
        cv::Point2f(1523.40049420479, 455.668727672321),
        cv::Point2f(1561.36635349151, 1085.28683579092),
        cv::Point2f(49.4632618694492, 816.421998077622),
        cv::Point2f(790.210950005499, 454.587756871404),
        cv::Point2f(1380.35540502213, 758.345652842801),
        cv::Point2f(388.601866068976, 291.923426964708),
        cv::Point2f(1334.83226195008, 685.695685924676),
        cv::Point2f(1301.74590784459, 1178.07452853171),
        cv::Point2f(1006.35696652101, 1019.61842268662),
        cv::Point2f(3.58111769733327, 340.133022826873)
    };

    // MATLAB outputs
    outputs = {
        cv::Point2f(1.51843724298416   ,4.23345715465917  ),
        cv::Point2f(-1.42223085322801  ,3.98335846058492  ),
        cv::Point2f(-0.260172645016205 ,-2.10445478678803 ),
        cv::Point2f(1.52419779691091   ,0.956234309145427 ),
        cv::Point2f(0.00801743584376968,3.42971508752357  ),
        cv::Point2f(2.42397279538234   ,-0.908718735396983),
        cv::Point2f(0.352829293581993  ,3.27370605834331  ),
        cv::Point2f(-1.79535937721773  ,2.93095175637991  ),
        cv::Point2f(-1.14985249952439  ,1.81913354949219  ),
        cv::Point2f(2.15030012663345   ,-2.65419007203807 )
    };

    // Create a ptr to our geolocator
    g = std::make_shared<Geolocator>(cameraModel);
  }

  virtual void TearDown() { }

  std::shared_ptr<Geolocator> g;
  params_t params;
  cv::Ptr<CameraModel> cameraModel;
  std::vector<cv::Point2f > measurements;
  std::vector<cv::Point2f > outputs;
  float abs_err;
};

// Using MATLAB 'geolocation_generate.m' output, check if transform is correct
TEST_F(GeolocationTest, TransformsCorrectly) {

    std::vector<cv::Point2f> unused;
    g->transform(measurements, unused,
                    params.pn, params.pe, params.pd,
                    params.phi, params.theta, params.psi,
                    params.az, params.el);

    for (int i = 0; i < measurements.size(); ++i) {
        EXPECT_NEAR(outputs[i].x, measurements[i].x, abs_err) << std::endl;
        EXPECT_NEAR(outputs[i].y, measurements[i].y, abs_err) << std::endl;
    }
}

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}