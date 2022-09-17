#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import math

class LaserCartesian:
    def __init__(self):
        self.counter = 0
        self.can_cart_marker_pub = rospy.Publisher("~scan_cartesian", Marker, queue_size=1)
        self.number_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        self.laser_frame_id_ = None # Holds laser frame id

    def scan_cb(self, scan:LaserScan):
        ############################## SCAN CALLBACK CODE HERE ##################################################
        self.laser_frame_id_ = scan.header.frame_id

        scan_cart = []
        for i in range(0, len(scan.ranges)):
            d_m = scan.ranges[i]
            # if there are any ranges larger then range max, ignore them
            if d_m>scan.range_max or d_m<scan.range_min:
                continue
            ang_r = i*scan.angle_increment+scan.angle_min
            x = math.cos(ang_r)*d_m
            y = math.sin(ang_r)*d_m
            scan_cart.append(Point(x, y, 0))

        if len(scan_cart) == 0:
            rospy.logwarn("There are no points left after converting to cartesian.")
            return

        self.create_and_pub_point_list_marker(scan_cart, self.can_cart_marker_pub, self.laser_frame_id_, (0,1,0))

        ############################## END OF SCAN CALLBACK CODE HERE ###########################################

    def create_and_pub_point_list_marker(self, marker_point_list, publisher, frame, color_rgb):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "scan_cartesian points"
        marker.id = 0
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.points = marker_point_list
        marker.pose.orientation.w= 1.0
        
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 0.4
        # Points are green
        marker.color.r = color_rgb[0]
        marker.color.g = color_rgb[1]
        marker.color.b = color_rgb[2]
        publisher.publish(marker)

if __name__ == '__main__':
    rospy.init_node('laser_cartesian', anonymous=True)
    LaserCartesian()
    rospy.spin()