#!/usr/bin/env python
import rospy

import message_filters

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import HomePosition
from visual_mtt.msg import Tracks
"""Visual MTT Tracks
    It is expected that tracks are stuffed with lat/lon data:
        # Change the 3D position to LLA
        track.position.x = longitude
        track.position.y = latitude
        track.position.z = 0 # flat earth assumption
"""

from geodesy import utm

import matplotlib.pyplot as plt
import numpy as np


class Anaylze(object):
    """Analyze

        ROS class that subscribes to two topics with Lat/Lon (LLA) information
        and performs an error analysis on the data.
    """
    def __init__(self):
        super(Anaylze, self).__init__()

        # origin, from LLA to UTM
        self.origin = None

        # Get the list of IDs that are associated
        # with the ground truth NavSatFix topic.
        self.associated_ids = rospy.get_param('~track_ids', [])

        # data
        self.samples = []

        # ROS connections
        self.sub_tracks = message_filters.Subscriber('tracks_lla', Tracks)
        self.sub_truth = message_filters.Subscriber('truth_lla', NavSatFix)
        self.sub_origin = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_cb)

        # Synchronize estimate and ground truth lat/lon
        queue_size = 10
        slop_sec = 0.5
        self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_tracks, self.sub_truth], queue_size, slop_sec, allow_headerless=True)
        self.sync.registerCallback(self.data_cb)

        # Plotting
        self.plot_timer = rospy.Timer(rospy.Duration(0.01), self.plot_cb)

        # Initialization
        self.fig_error = None
        self.fig_2d = None


    def data_cb(self, tracks, truth):

        # convert truth LLA to UTM
        truth_utm = utm.fromLatLong(truth.latitude, truth.longitude)

        sample = {
            'estimate_lla': {},
            'truth_lla': [truth.latitude, truth.longitude],
            'estimate_utm': {},
            'truth_utm': [truth_utm.easting, truth_utm.northing],
        }
        
        # find if there are any relevant IDs in this tracks message
        for track in tracks.tracks:
            if track.id in self.associated_ids:
                # Latitude and Longitude
                sample['estimate_lla'][track.id] = [track.position.y, track.position.x]

                # UTM Easting and Northing
                pt = utm.fromLatLong(track.position.y, track.position.x)
                sample['estimate_utm'][track.id] = [pt.easting, pt.northing]


        # Make sure there was a track estimate sample
        if sample['estimate_lla']:
            self.samples.append(sample)


    def home_cb(self, msg):
        self.origin = utm.fromLatLong(latitude=msg.geo.latitude, longitude=msg.geo.longitude)


    def plot_cb(self, event):

        self.plot_2d()

        if self.origin is not None:
            self.plot_error()


    def plot_2d(self):
        return
        if not self.fig_2d:
            plt.ion()
            self.fig_2d = plt.figure()
            self.fig_2d_e = self.fig_2d.add_subplot(311)
            self.fig_2d_n = self.fig_2d.add_subplot(312)
            self.fig_2d_norm = self.fig_2d.add_subplot(313)

        self.fig_2d_e.clear()
        self.fig_2d_n.clear()
        self.fig_2d_norm.clear()

        estimate = [s['estimate_utm'].values()[0][0] for s in self.samples]
        truth = [s['truth_utm'][0] for s in self.samples]
        error_e = np.array(truth) - np.array(estimate)
        self.fig_2d_e.plot(error_e)
        self.fig_2d_e.grid()

        estimate = [s['estimate_utm'].values()[0][1] for s in self.samples]
        truth = [s['truth_utm'][1] for s in self.samples]
        error_n = np.array(truth) - np.array(estimate)
        self.fig_2d_n.plot(error_n)
        self.fig_2d_n.grid()

        # self.fig_2d_norm.plot(np.linalg.norm(np.array([error_e, error_n])))


        self.fig_2d.canvas.draw()


    def plot_error(self):
        if not self.fig_error:
            plt.ion()
            self.fig_error = plt.figure()
            self.fig_error_e = self.fig_error.add_subplot(311)
            self.fig_error_n = self.fig_error.add_subplot(312)
            self.fig_error_norm = self.fig_error.add_subplot(313)

        self.fig_error_e.clear()
        self.fig_error_n.clear()
        self.fig_error_norm.clear()

        estimate = [s['estimate_utm'].values()[0][0] for s in self.samples]
        truth = [s['truth_utm'][0] for s in self.samples]
        error_e = np.array(truth) - np.array(estimate)
        self.fig_error_e.plot(error_e)
        self.fig_error_e.grid()

        estimate = [s['estimate_utm'].values()[0][1] for s in self.samples]
        truth = [s['truth_utm'][1] for s in self.samples]
        error_n = np.array(truth) - np.array(estimate)
        self.fig_error_n.plot(error_n)
        self.fig_error_n.grid()

        # self.fig_error_norm.plot(np.linalg.norm(np.array([error_e, error_n])))


        self.fig_error.canvas.draw()


if __name__ == '__main__':
    rospy.init_node('analyze', anonymous=True)
    node = Anaylze()
    rospy.spin()
