#!/usr/bin/env python
import rospy
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin
from std_msgs.msg import Int16
from arm_rrt.msg import config_point
import numpy as np
import random
import time

#from geometry_msgs import PoseStamped
# dof upper_limit lower_limit global_list

#global varaibles
dof = 2
step_local_planner = 0.05
upper_limit_distance = 0.5
lower_limit_distance = 0.1
upper_angle_limit_config = 6.2
lower_angle_limit_config = 0.1
max_number_of_node = 200

class nodes:
    
    def __init__(self,point,parent):
        self.config=np.empty(dof,dtype=float)
        self.config=point
        self.parent_node=parent

class path_planner:

    def __init__(self,start):
        self.rate=rospy.Rate(0.1)
        self.random_point=np.empty(dof,dtype=float)
        self.list_of_nodes=[start]
        self.point_to_reach=[]
        self.laser_scan_data=   # idhar kuch toh daalo 
        self.new_point_recieved= False
        self.current_pose=[]
        self.sub_laser_scan_data=rospy.Subscriber("chatter",Int16,self.laser_data_callback,queue_size=1)
        self.sub_goal=rospy.Subscriber("goal_point",config_point,self.goal_point_callback,queue_size=1)
        self.sub_current_pose=rsopy.Subscriber()
        # self.pub_test=rospy.Publisher('chatter',Int16,queue_size=10)
     
    def goal_point_callback(self,msg) :
        self.point_to_reach=msg.point
        self.new point_recieved= True
    def laser_data_callback(self,msg) :


    def generate_point(self):        
        point_found= False
        while not point_found :
            for i in range(dof):
                self.random_point[i]=random.uniform(lower_angle_limit_config,upper_angle_limit_config)
            if(self.collision_free()):
                if(self.near_to_points()):                    
                    point_found= True
        print('point generated')

    def distance_with_random(self,point):
        #checks distance of the randomly generated point and if in bound returns true
        distance=0.0
        for i in range(dof):
            distance+=(point[i]-self.random_point[i])*(point[i]-self.random_point[i])
        if(distance<upper_limit_distance and distance >lower_limit_distance):
            return True
        return False

    def local_path_exist(self,point) :
        # I need to move from point to random_point along a straight line
        

    def near_to_points_with_local_path(self):
        #checks thr global list for a near neighbour
        found_a_neighbour= False
        i=0
        while not found_a_neighbour:
            if(self.distance_with_random(self.list_of_nodes[i].config)):
                if(self.local_path_exist(self.list_of_nodes[i])):
                    found_a_neighbour= True
            if(i==(len(self.list_of_nodes)-1)):
                return False
            i=i+1
        node_obj=nodes(self.random_point,(i-1))
        self.list_of_nodes.append(node_obj)
        return True 

    def collision_free(self):
        # returns true if the given point is collision free else return false

    def plan(self):
        # introduced a delay to allow varaibles to get assigned busing call backs
        time.sleep(5)
        while not rospy.is_shutdown():   
            if (self.new_point_recieved and self.path_being_not_planned):


            self.msg_test.data=self.i
            self.i=self.i+1
            self.pub_test.publish(self.msg_test)
            print('I published:' , self.i)
            self.rate.sleep()
     

if __name__ == '__main__':
    global dof
    global upper_limit_distance 
    global lower_limit_distance 
    global upper_angle_limit_config 
    global lower_angle_limit_config 
    global max_number_of_node 
    rospy.init_node('rrt_node')
    rospy.loginfo("rrt_node created")
    try:
    dof = rospy.get_param("dof")
    upper_limit_distance = rospy.get_param("upper_limit_distance")
    lower_limit_distance = rospy.get_param("lower_limit_distance")
    upper_angle_limit_config = rospy.get_param("upper_angle_limit_config")
    lower_angle_limit_config =rospy.get_param("lower_angle_limit_config")
    max_number_of_node = rospy.get_param("max_number_of_node")      
    while (rospy.get_time()==0):
      pass
    rospy.loginfo("starting to acccept end goal to generate path")
    path_planner_obj=path_planner()
    path_planner_obj.plan()

    except rospy.ROSInterruptException:
    pass
