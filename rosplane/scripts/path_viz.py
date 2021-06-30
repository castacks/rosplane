#!/usr/bin/env python
import rospy

import tf
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rosplane_msgs.msg as rosplane_msgs
from visualization_msgs.msg import Marker

import numpy as np

class PathViz:
    def __init__(self):
        '''Subscribe to the state of the plane and commanded waypoints'''
        self.stateSub = rospy.Subscriber("/fixedwing/truth", rosplane_msgs.State, self.state_callback)
        self.waypointSub = rospy.Subscriber("/fixedwing/waypoint_path", rosplane_msgs.Waypoint, self.waypoint_callback)

        '''Convert state and waypoint to odometry for visualization'''
        self.odomVizPub = rospy.Publisher("/fixedwing/odom_viz", Odometry, queue_size=10)
        self.waypointVizPub = rospy.Publisher("/fixedwing/waypoint_viz", Odometry, queue_size=10)
        self.waypointLinePub = rospy.Publisher("/fixedwing/waypoint_line_viz", Marker, queue_size=10)

        rospy.Timer(period=rospy.Duration(0.1), callback=self.visualizer_update)
        rospy.Timer(period=rospy.Duration(1.0), callback=self.waypoint_viz_update)
        
        self.odom_viz = Odometry()
        '''Maintain a list of waypoints
        TODO : Clear the waypoint list if flag is true
        '''
        self.waypoints = []
        '''Put origin as a waypoint ; incase the aircraft is following a flight pattern'''
        self.initial_odom = Odometry()
        self.initial_odom.header.stamp = rospy.Time.now()
        self.initial_odom.header.frame_id = "world"
        self.initial_odom.pose.pose.position.x = 0.0
        self.initial_odom.pose.pose.position.y = 0.0
        self.initial_odom.pose.pose.position.z = 0.0

        self.initial_odom.pose.pose.orientation.w = 1
        self.initial_odom.pose.pose.orientation.x = 0
        self.initial_odom.pose.pose.orientation.y = 0
        self.initial_odom.pose.pose.orientation.y = 0
        self.waypoints.append(self.initial_odom)

        '''Line strip to visualize an approximate path'''
        self.line_strip = Marker()
        self.line_strip.header.frame_id = "world"
        self.line_strip.ns = "rudimentary_waypoint_path"
        self.line_strip.id = 1
        self.line_strip.type = Marker.LINE_STRIP
        self.line_strip.scale.x = 1.0
        self.line_strip.color.b = 1.0 # colour of lines will be blue
        self.line_strip.color.a = 1.0 # Opaque
        self.line_strip.pose.orientation.w = 1

        '''Add a point at origin'''
        p = Point()
        p.x = p.y = p.z = 0
        self.line_strip.points.append(p)

    
    def state_callback(self, msg):
        self.odom_viz.header.stamp = rospy.Time.now()
        self.odom_viz.header.frame_id = "world"

        pn = msg.position[0]
        pe = msg.position[1]
        pd = msg.position[2]

        # Convert NED to rviz RGB
        self.odom_viz.pose.pose.position.x = pn
        self.odom_viz.pose.pose.position.y = -pe
        self.odom_viz.pose.pose.position.z = -pd

        # Pitch and Yaw reverse sign when changing from NED to XYZ
        r = msg.phi
        p = -msg.theta
        y = -msg.psi
        quat = tf.transformations.quaternion_from_euler(r,p,y)
        self.odom_viz.pose.pose.orientation.x = quat[0]
        self.odom_viz.pose.pose.orientation.y = quat[1]
        self.odom_viz.pose.pose.orientation.z = quat[2]
        self.odom_viz.pose.pose.orientation.w = quat[3]
    
    def waypoint_callback(self, msg):
        waypoint_viz = Odometry()
        waypoint_viz.header.stamp = rospy.Time.now()
        waypoint_viz.header.frame_id = "world"

        # Convert NED to XYZ
        waypoint_viz.pose.pose.position.x = msg.w[0]
        waypoint_viz.pose.pose.position.y = -msg.w[1]
        waypoint_viz.pose.pose.position.z = -msg.w[2]

        r = 0.0
        p = 0.0
        y = -msg.chi_d

        quat = tf.transformations.quaternion_from_euler(r,p,y)
        waypoint_viz.pose.pose.orientation.x = quat[0]
        waypoint_viz.pose.pose.orientation.y = quat[1]
        waypoint_viz.pose.pose.orientation.z = quat[2]
        waypoint_viz.pose.pose.orientation.w = quat[3]

        self.waypoints.append(waypoint_viz)

        point = Point()
        point.x = waypoint_viz.pose.pose.position.x
        point.y = waypoint_viz.pose.pose.position.y
        point.z = waypoint_viz.pose.pose.position.z
        self.line_strip.points.append(point)
    
    def visualizer_update(self, event=None):
        self.odomVizPub.publish(self.odom_viz)
        n = len(self.waypoints)
        if n >= 2:
            self.waypointLinePub.publish(self.line_strip)
    
    def waypoint_viz_update(self, event=None):
        n = len(self.waypoints)
        if n > 0: 
            for i in range(0,n):
                self.waypointVizPub.publish(self.waypoints[i])
        
if __name__=='__main__':
    rospy.init_node("rosplane_path_visualizer", anonymous=True)
    pathViz = PathViz()
    rospy.spin()