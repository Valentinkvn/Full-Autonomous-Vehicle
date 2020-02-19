#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree    # imported for searching for the closest waypoint to our base
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 80 # Number of waypoints we will publish. You can change this number
RATE_IN_HZ = 10
MAX_DECEL = .5
MAX_VELOCITY = 11.11
# MAX_DECEL = 5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        
        self.current_velocity = None
        self.prev_velocity = 0
        self.prev_waypoints = None 
        self.first_waypoints = False
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints_2d = None
        self.waypoint_tree = None
        self.pose = None
        self.base_lane = None
        self.stopline_wp_idx = -1
        
        self.is_car_stopped = True

        self.loop()

    def loop(self):
        rate = rospy.Rate(RATE_IN_HZ)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                # Get the closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        
        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        # Verify if the dot product is pos or neg to see
        # if the waypoint is in front or in the back of the car
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        
#         rospy.logwarn(self.stopline_wp_idx)
        
#         if self.current_velocity < 0.1:
#             self.is_car_stopped = True
#         else:
#             self.is_car_stopped = False
        if self.first_waypoints == True:
            lane.waypoints = self.waypoint_initialization(base_waypoints, closest_idx)
            self.prev_waypoints = lane.waypoints
            self.first_waypoints = False
        else:
            if (self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx)) and self.current_velocity > 11:
    #             rospy.logwarn("Plateau reached")
                lane.waypoints = base_waypoints

            elif self.stopline_wp_idx != -1:
    #             rospy.logwarn("Deccelerate")
                lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

            else:
                rospy.logwarn("Accelerate!")
                lane.waypoints = self.accelerate_waypoints(self.prev_waypoints, closest_idx)
       
        return lane

    def waypoint_initialization(self, waypoints, closest_idx):
        temp = []
        ref_velocity = 0
        
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # get the current waypoint acceleration
            # it is proportional to the 
            if i == 0:
                vel = self.prev_velocity + .06
                ref_velocity = vel
                self.prev_velocity = ref_velocity
            else:
                vel = (i + 1) * (1.0/LOOKAHEAD_WPS) * wp.twist.twist.linear.x + ref_velocity

            p.twist.twist.linear.x = min(vel, MAX_VELOCITY)
#             rospy.logwarn("At %d moment the velocity is %lf", i, p.twist.twist.linear.x)
            
            temp.append(p)
       
        return temp

    def accelerate_waypoints(self, waypoints, closest_idx):
        
        ref_velocity = 0
        
        
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # get the current waypoint acceleration
            # it is proportional to the 
            if i == 0:
                vel = self.prev_velocity + .06
                ref_velocity = vel
                self.prev_velocity = ref_velocity
            else:
                vel = (i + 1) * (1.0/LOOKAHEAD_WPS) * wp.twist.twist.linear.x + ref_velocity

            p.twist.twist.linear.x = min(vel, MAX_VELOCITY)
#             rospy.logwarn("At %d moment the velocity is %lf", i, p.twist.twist.linear.x)
            
            temp.append(p)
       
        return temp
    
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # see where the stop position is regarding the current position
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # 5 waypoints back so the car stops at the line
            
#             stop_idx = self.stopline_wp_idx - 5
    
            if stop_idx >= 0:
#                 rospy.logwarn(stop_idx)
#                 dist = self.custom_distance(waypoints, i, stop_idx)
#                 rospy.logwarn("We are currently at %u", closest_idx)
#                 rospy.logwarn("The stopline is at %u", self.stopline_wp_idx)
#                 rospy.logwarn("The distance to stop idx is %u", stop_idx)
#                 
#                 vel = math.sqrt(2 * MAX_DECEL * stop_idx)
                vel = 2 * MAX_DECEL * stop_idx
#                 rospy.logwarn("Proposed velocity is %lf", vel)
                if vel < 1.0:
                    vel = 0.0

                # set the speed limit
                p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                temp.append(p)
            
#             rospy.logwarn("\n")
        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        # rospy.logwarn("Pose callback")
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # rospy.logwarn("Waypoint callback")
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # rospy.logwarn("Traffic callback")
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    
    def velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    def custom_distance(self, waypoints, a, n):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
