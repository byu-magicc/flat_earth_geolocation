#include <ros/ros.h>
#include "plotter/plotter.h"

int main(int argc, char** argv) {
  // start node
  ros::init(argc, argv, "plotter");

  // instantiate an object
  plotter::Plotter plt;

  ros::spin();
  return 0;
}