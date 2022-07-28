#!/usr/bin/python3

import math
import json
import random
import rospy
import copy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped,PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class LocalGoalCreator:
    def __init__(self):
        rospy.init_node('WaypointCreator', anonymous=True)

        # param
        self.HZ = rospy.get_param("~HZ", 10)
        self.WORLD_FRAME = rospy.get_param("~WORLD_FRAME", 'map')
        self.ROBOT_FRAME = rospy.get_param("~ROBOT_FRAME", 'base_link')
        # self.GOAL_DIS_TOLERANCE = rospy.get_param("~GOAL_DIS_TOLERANCE", 0.3)
        # self.GOAL_YAW_TOLERANCE = rospy.get_param("~GOAL_YAW_TOLERANCE", 1.0)
        # self.TIMEOUT = rospy.get_param("~TIMEOUT", 180)

        self.estimated_pose_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.estimated_pose_call_back)
        # self.global_goal_sub = rospy.Subscriber('/goal_reach',Bool,self.goal_reach_call_back)

        WAYPOINTS_PATH = rospy.get_param("~WAYPOINTS_PATH",'/home/amsl/catkin_ws/src/simple_waypoint_creator/waypoints/waypoints.json')
        with open(WAYPOINTS_PATH) as f:
            waypoints_data = json.load(f)
        self.waypoints = []
        for wp in waypoints_data["WAYPOINTS"]:
            self.waypoints.append([wp["x"], wp["y"], wp["yaw"]])
        self.idx = 0

        # publisher
        self.waypoint_pub = rospy.Publisher('/waypoint', Path, queue_size=10)

        self.start_time = rospy.Time.now()
        self.goal_reach = True
        self.estimated_pose = PoseWithCovarianceStamped()
        self.global_goal_x = 0
        self.global_goal_y = 0

        self.split = 10
        self.global_path = Path()

    def estimated_pose_call_back(self,msg):

        self.estimated_pose = msg

    def local_goal_creator(self):
        self.global_path = Path()
        path_point = PoseStamped()
        path_point.header.frame_id = "map"
        for i in range(len(self.waypoints)-1):
            x1 = self.waypoints[i][0]
            x2 = self.waypoints[i+1][0]
            y1 = self.waypoints[i][1]
            y2 = self.waypoints[i+1][1]

            if (x2 - x1) != 0:
                a = (y2 - y1) / (x2 - x1)

            b = y1 - a*x1
            split = (x2 - x1)/self.split
            path_point.pose.position.x = self.waypoints[i][0]
            path_point.pose.position.y = self.waypoints[i][1]
            self.global_path.poses.append(copy.deepcopy(path_point))
            add_split = split
            for i in range(self.split):
                y = a * (add_split + x1) + b
                x = add_split + x1
                path_point.pose.position.x = x
                path_point.pose.position.y = y
                self.global_path.poses.append(copy.deepcopy(path_point))
                add_split += split
        # path_point.pose.position.x = self.waypoints[-1]][0]
        # path_point.pose.position.y = self.waypoints[-1]][1]
        self.global_path.poses.append(copy.deepcopy(path_point))
        self.global_path.header.frame_id = "map"
        self.global_goal_x = path_point.pose.position.x
        self.global_goal_y = path_point.pose.position.y

    def check_goal_reach(self):
        cx = self.estimated_pose.pose.pose.position.x
        cy = self.estimated_pose.pose.pose.position.y
        gx = self.global_goal_x
        gy = self.global_goal_y
        dis = math.sqrt((gx - cx) ** 2 + (gy -cy) ** 2)
        # print(dis)
        if dis < 1.0:
            self.goal_reach = True
        else:
            self.goal_reach = False

    def process(self):
        r = rospy.Rate(self.HZ)
        while not rospy.is_shutdown():
            # self.check_goal_reach()
            self.local_goal_creator()
            self.waypoint_pub.publish(self.global_path)
            r.sleep()

if __name__ == '__main__':
    local_goal_creator = LocalGoalCreator()
    try:
        local_goal_creator.process()
    except rospy.ROSInterruptException:
        pass
