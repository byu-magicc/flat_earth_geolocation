#include <ros/ros.h>
#include "geolocator/geolocator.h"

int main(int argc, char** argv) {
  // start node
  ros::init(argc, argv, "geolocator");

  // instantiate an object
  geolocator::Geolocator geo;

  ros::spin();
  return 0;
}