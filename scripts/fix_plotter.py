#!/usr/bin/env python

""" 
    Subscribe to a NavSatFix message, convert to UTM and then publish
    an Rviz marker. The marker is published with a frame_id based on
    the UTM zone and band, e.g., `utm_origin_12T`.
"""

import rospy

from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix

from geodesy import utm

class FixPlotter:
    def __init__(self):
        self.marker_pub = rospy.Publisher('visualization' + rospy.get_name(), Marker, queue_size=1)

        rospy.Subscriber('gps_fix', NavSatFix, self.cb_fix, queue_size=1)


    def cb_fix(self, msg):
        """cb_fix
            
            Callback for NavSatFix message. Takes the lat/lon geodetic coords
            and converts them to UTM.
        """
        
        # Convert to UTM
        utm_pt = utm.fromMsg(msg)

        # Create Rviz marker
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_from_utm(utm_pt)

        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = utm_pt.easting
        marker.pose.position.y = utm_pt.northing
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)



    def frame_from_utm(self, utm_pt):
        return 'utm_origin_{}{}'.format(utm_pt.zone, utm_pt.band)




if __name__ == '__main__':
    rospy.init_node('fix_plotter', anonymous=True)

    plot = FixPlotter()

    rospy.spin()