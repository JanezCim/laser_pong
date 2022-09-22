#!/usr/bin/python3

import rospy
from interactive_marker_tutorials.msg import pongControl
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import math

class LaserPong:
    def __init__(self):
        self.counter = 0
        self.pong_contr_pub = rospy.Publisher("/pong_control", pongControl, queue_size=1)
        self.can_cart_marker_pub = rospy.Publisher("/scan_cartesian", Marker, queue_size=1)
        self.number_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        self.laser_frame_id_ = None # Holds laser frame id

        self.actual_field_width_m = 0.5 # the width of the projected field
        self.actual_field_lenght_m = 0.5 # the length of the projected field

        self.output_field_width_m = 10 # the width of the field as pong output wants it 
        self.nr_min_valid_paddle_points = 10


    def scan_cb(self, msg):
        scan = LaserScan()
        scan = msg
        self.laser_frame_id_ = scan.header.frame_id
        ############################## SCAN CALLBACK CODE HERE ##################################################

        ##### INSERT SCAN MESSAGE CONVERTED TO CARTESIAN INTO scan_cart
        scan_cart = []

        ##### INSERT LIMITED CARTESIAN SCAN INTO scan_cart_lim
        scan_cart_lim = []
        
        self.create_and_pub_point_list_marker(scan_cart_lim, self.can_cart_marker_pub, self.laser_frame_id_, (0,1,0))

        if len(scan_cart_lim) == 0:
            rospy.logwarn_throttle(10, "There are no points inside of the field.")
            pc_msg = pongControl()
            self.pong_contr_pub.publish(pc_msg)
            return
        
        for p in scan_cart_lim:
            if p.x < 0: 
                p1_p_count+=1
                p1_y_sum+=p.y
            else:
                p2_p_count+=1
                p2_y_sum+=p.y

        ##### CREATE A PONG CONTROL MESSAGE FROM THE SCAN DATA
        pc_msg = pongControl()

        self.pong_contr_pub.publish(pc_msg)

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
    rospy.init_node('laser_pong')
    LaserPong()
    rospy.spin()