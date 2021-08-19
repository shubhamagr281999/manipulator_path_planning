#!/usr/bin/env python
import rospy
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler , euler_matrix
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin, ceil
from std_msgs.msg import Int16, Float64
from arm_rrt.msg import config_point
import numpy as np
import random
import time
from sensor_msgs.msg import JointState , LaserScan
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt

#global varaibles
dof = 2
number_of_point_local_planner = 10
upper_limit_distance = 0.5
lower_limit_distance = 0.15
upper_angle_limit_config = 6.2
lower_angle_limit_config = 0.0
max_number_of_node = 40
link_lengths=[1.0,1.0]
link_offsets=[0,0.025]
pi=3.14159265358


class nodes:    
    def __init__(self,point,parent):
        self.config=np.array(point,dtype=float)
        self.parent_node=parent

class path_planner:
    def __init__(self):
        self.rate_move=rospy.Rate(0.5)
        self.obstacles=np.full((900,900),False,dtype=bool)
        self.obstacles[645:720,645:720]=np.full((75,75),True,dtype=bool)
        self.obstacles[645:825,645:825]=np.full((180,180),True,dtype=bool)
        self.obstacles[0:180,360:540]=np.full((180,180),True,dtype=bool)
        self.obstacles[190:370,75:255]=np.full((180,180),True,dtype=bool)
        self.points_sampled_x=[]
        self.points_sampled_y=[]
        self.path=[]
        self.rate=rospy.Rate(0.1)
        self.random_point=np.empty(dof,dtype=float)
        self.list_of_nodes=[]
        self.point_to_reach=[]
        self.laser_scan_data= [] 
        self.new_point_recieved= False
        self.path_not_being_planned = True
        self.current_pose=[]
        # self.sub_goal=rospy.Subscriber("goal_point",config_point,self.goal_point_callback,queue_size=1)
        # self.sub_joint_state=rospy.Subscriber("me604/joint_states", JointState, self.joint_states_callback, queue_size=1)
        # self.sub_joint_state=rospy.Subscriber("tf",TFMessage, self.transforms_callback, queue_size=1)
        # self.transforms=np.zeros((dof,4,4),dtype=float)  # stores i to i+1 transformations
        # self.pub_joint2=rospy.Publisher('/me604/joint2_position_controller/command',Float64,queue_size=10)
        # self.pub_joint1=rospy.Publisher('/me604/joint1_position_controller/command',Float64,queue_size=10)
        # self.sub_laser_data=rospy.Subscriber("/me604/laser/scan1",LaserScan, self.Laserscan_callback, queue_size=1)

    def collision_free (self,point) :
        for i in range (20) :
            point_to_check_x=450 + int(cos(point[0])*(i/20.0)*(900/5.0)*link_lengths[0])
            point_to_check_y=450 + int(sin(point[0])*(i/20.0)*(900/5.0)*link_lengths[0])
            print(point[0],point[1])
            if(self.obstacles[point_to_check_x,point_to_check_y]) :
                print("obstacle hai bhai",point[0],point[1])
                return False
            point_to_check_x=int(450 + cos(point[1]+point[0])*(i/20.0)*(900/5.0)*link_lengths[1] + cos(point[0])*(link_lengths[0]-link_offsets[1])*(720/4))
            point_to_check_y=int(450 + sin(point[1]+point[0])*(i/20.0)*(900/5.0)*link_lengths[1] + sin(point[0])*(link_lengths[0]-link_offsets[1])*(720/4))
            if(self.obstacles[point_to_check_x,point_to_check_y]) :
                print("obstacle hai bhai")
                return False
                
        # print("obstacle free")
        return True


    def plan(self):
        # introduced a delay to allow varaibles to get assigned using call backs
        time.sleep(5)

        for i in range(30):
            self.points_sampled_x.append(lower_angle_limit_config +i/30.0*upper_angle_limit_config)
            self.points_sampled_y.append(lower_angle_limit_config +i/30.0*upper_angle_limit_config)
        for i in range(30):
            for j in range(30):
                if(self.collision_free([self.points_sampled_x[i],self.points_sampled_y[j]])):
                    plt.scatter(self.points_sampled_x[i],self.points_sampled_y[j], color= "green", s=10)
                else :
                    plt.scatter(self.points_sampled_x[i],self.points_sampled_y[j], color= "red", s=10)

        plt.show()


if __name__ == '__main__':
    rospy.init_node('rrt_node')
    rospy.loginfo("rrt_node created")
    try:
        while (rospy.get_time()==0):
          pass
        rospy.loginfo("starting to acccept end goal to generate path")
        path_planner_obj=path_planner()
        path_planner_obj.plan()

    except rospy.ROSInterruptException:
        pass
