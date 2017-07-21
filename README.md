Flat Earth Geolocation
======================

This node performs geolocation with a flat earth assumption. The geolocation algorithm uses line-of-sight vectors and folows the derivation from Chapter 13 of the UAV Book.

Note that all coordinate transformations are found using the ROS `tf` package.

## Dependencies ##

```bash
$ sudo apt install ros-kinetic-geodesy
```

## Helpful Hints ##

If you have GPS "truth" of ground targets, you can view them in Rviz alongside the geolocated tracks by running the `fix_plotter.py` node and hooking up the ground target GPS to it:

```bash
$ rosrun flat_earth_geolocation fix_plotter.py __name:=target_name gps_fix:=/htc_one_m9/fix
```

This will publish a `visualization_msgs::Marker` message that can be viewed in Rviz with the `Marker` type. It is expected that there is an appropriate UTM origin defined for the zone/band that both the UAV and the targets are in. This assumption should basically always hold true since everything is done locally.