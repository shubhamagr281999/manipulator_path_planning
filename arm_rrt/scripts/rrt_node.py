#!/usr/bin/env python
import rospy
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler , euler_matrix
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin, ceil
from std_msgs.msg import Int16
from arm_rrt.msg import config_point
import numpy as np
import random
import time
from sensor_msgs.msg import JointState , LaserScan
from tf2_msgs.msg import TFMessage

#global varaibles
dof = 2
number_of_point_local_planner = 10
upper_limit_distance = 0.6
lower_limit_distance = 0.3
upper_angle_limit_config = 6.2
lower_angle_limit_config = 0.1
max_number_of_node = 500
link_lengths=[1,1]
link_offsets=[0,0.025]
pi=3.14159265358


class nodes:    
    def __init__(self,point,parent):
        self.config=np.array(point,dtype=float)
        self.parent_node=parent

class path_planner:
    def __init__(self):
        self.obstacles=np.full((720,720),False,dtype=bool)
        self.obstacles[100:200,100:200]=np.full((100,100),True,dtype=bool)
        self.path=[]
        self.rate=rospy.Rate(0.1)
        self.random_point=np.empty(dof,dtype=float)
        self.list_of_nodes=[]
        self.point_to_reach=[]
        self.laser_scan_data= [] 
        self.new_point_recieved= False
        self.path_not_being_planned = True
        self.current_pose=[]
        self.sub_goal=rospy.Subscriber("goal_point",config_point,self.goal_point_callback,queue_size=1)
        self.sub_joint_state=rospy.Subscriber("me604/joint_states", JointState, self.joint_states_callback, queue_size=1)
        # self.sub_joint_state=rospy.Subscriber("tf",TFMessage, self.transforms_callback, queue_size=1)
        self.transforms=np.zeros((dof,4,4),dtype=float)  # stores i to i+1 transformations
        # self.sub_laser_data=rospy.Subscriber("/me604/laser/scan1",LaserScan, self.Laserscan_callback, queue_size=1)

    # def Laserscan_callback(self,msg) :
    #    self.laser_scan_data=msg.ranges
        #need to be slective in the data we process
        #threshold for dixcrete

    # def transforms_callback(self,msg):
    #     for x in range(dof):
    #         [r,p,y]=euler_from_quaternion(msg.transforms[x].rotation)
    #         rotation=euler_matrix(r,p,y)
    #         [x,y,z]=msg.transforms[x].translationFals
    #         self.transforms[x][:3,:3]=rotation
    #         self.transforms[x][:4,3]=[x,y,z,1]

    def joint_states_callback(self,msg) :
        self.current_pose=msg.position


    def goal_point_callback(self,msg) :
        self.point_to_reach=msg.point
        self.new_point_recieved= True  

    def generate_point(self):        
        point_found= False
        while not point_found :
            #creating random point
            # print(len(self.list_of_nodes))
            # print("looking for a new valid node")
            for i in range(dof):
                self.random_point[i]=random.uniform(lower_angle_limit_config,upper_angle_limit_config)
            self.random_point=np.array(self.random_point)

            # creating transforms corresponding to random point
            # for i in range(dof):
            #     rotation=np.array([[cos(self.random_point[0]),-sin(self.random_point[1]),0],[sin(self.random_point[1]),cos(self.random_point[1]),0],[0,0,1]])
            #     self.transforms[i][:3,:3]=rotation
            #     self.transforms[i][:4,3]=[0,0,0,1]
            #     self.transforms[i]=np.array(self.transforms[i])
            # for i in range(1,dof):
            #     self.transforms[i][1][3]=link_lengths[i-1]-link_offsets[i-1]
            #     self.transforms[i]=np.array(self.transforms[i])
            #     self.transforms[i]=np.dot(self.transforms[i-1],self.transforms[i])

            #checking for a valid point
            if(self.collision_free(self.random_point)):
                if(self.near_to_points_with_local_path()):
                    print("yoo I found a point time to break")                    
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
        self.random_point=np.array(self.random_point)
        point=np.array(point)
        for i in range(1,number_of_point_local_planner):           
            new_point_to_investigate=self.random_point + (point-self.random_point)*i/number_of_point_local_planner
            if not self.collision_free(new_point_to_investigate):
                return False
        return True        

    def near_to_points_with_local_path(self):
        #checks the global list for a near neighbour
        found_a_neighbour= False
        i=0
        while not found_a_neighbour:
            if(i==len(self.list_of_nodes)):
                return False
            if(self.distance_with_random(self.list_of_nodes[i].config)):
                if(self.local_path_exist(self.list_of_nodes[i].config)):
                    found_a_neighbour= True
            i=i+1
        node_obj=nodes(self.random_point,(i-1))
        self.list_of_nodes.append(node_obj)
        return True 

    # def collision_free(self,point):
    #     # returns true if the given point is collision free else return false
    #     sensor_angle=pi/2+self.random_point[0]   # need to modify this also
    #     element_to_check=snesor_angle/(2*pi)*720
    #     #collision check for link1 can be further optimised or made better for now it is an approx
    #     if(self.laser_scan_data[element_to_check]<link_lengths[0]+0.1):
    #         return False            
    #     for i in range(1,dof):
    #         for r in range(1,20):
    #             position_to_check=np.array([0,link_lengths[i]*r/20,0,1])
    #             position_to_check=np.dot(self.transforms[i],position_to_check)
    #             sensor_angle=-(pi/2-math.atan(position_to_check[1],position_to_check[0]))   #needs to modify this
    def collision_free (self,point) :
        for i in range (20) :
            point_to_check_x=360 + int(cos(point[0])*(i/20)*(720/4)*link_lengths[0])
            point_to_check_y=360 + int(sin(point[0])*(i/20)*(720/4)*link_lengths[0])
            if(self.obstacles[point_to_check_x,point_to_check_y]) :
                return False
            point_to_check_x=int(360 + cos(point[1])*(i/20)*(720/4)*link_lengths[1] + cos(point[0])*(link_lengths[0]-link_offsets[1])*(720/4))
            point_to_check_y=int(360 + sin(point[1])*(i/20)*(720/4)*link_lengths[1] + sin(point[0])*(link_lengths[0]-link_offsets[1])*(720/4))
            if(self.obstacles[point_to_check_x,point_to_check_y]) :
                return False
        # print("obstacle free")
        return True


    def plan(self):
        # introduced a delay to allow varaibles to get assigned using call backs
        time.sleep(5)
        while not rospy.is_shutdown():   
            if self.new_point_recieved :
                self.new_point_recieved= False
                node_obj=nodes(self.current_pose,999)
                self.list_of_nodes=[ node_obj ] 
                print(len(self.list_of_nodes))
                for i in range(max_number_of_node):
                    self.generate_point()
                time.sleep(3)
                print("200 points generated now need to connect to end point")

                # find a node neaar to goal point
                found_near_to_endPoint= False
                path_exist = True
                i=0
                self.random_point=self.point_to_reach
                while not found_near_to_endPoint :
                    if(i==len(self.list_of_nodes)):
                        print('path doesnt exist')
                        path_exist = False
                        break                     
                    if(self.distance_with_random(self.list_of_nodes[i].config)):
                        if(self.local_path_exist(self.list_of_nodes[i].config)):
                            found_near_to_endPoint=True
                    i=i+1

                if path_exist:                         
                    way_point=i-1
                    self.path=[]
                    while (way_point<300) :
                        self.path.append(way_point)
                        way_point=self.list_of_nodes[way_point].parent_node
                    print("path generated")
                    print(self.path)


            # self.msg_test.data=self.i
            # self.i=self.i+1
            # self.pub_test.publish(self.msg_test)
            # print('I published:' , self.i)
            # self.rate.sleep()
     

if __name__ == '__main__':
    # global dof
    # global upper_limit_distance 
    # global lower_limit_distance 
    # global upper_angle_limit_config 
    # global lower_angle_limit_config 
    # global max_number_of_node 
    rospy.init_node('rrt_node')
    rospy.loginfo("rrt_node created")
    try:
        # dof = rospy.get_param("dof")
        # upper_limit_distance = rospy.get_param("upper_limit_distance")
        # lower_limit_distance = rospy.get_param("lower_limit_distance")
        # upper_angle_limit_config = rospy.get_param("upper_angle_limit_config")
        # lower_angle_limit_config =rospy.get_param("lower_angle_limit_config")
        # max_number_of_node = rospy.get_param("max_number_of_node")      
        while (rospy.get_time()==0):
          pass
        rospy.loginfo("starting to acccept end goal to generate path")
        path_planner_obj=path_planner()
        path_planner_obj.plan()

    except rospy.ROSInterruptException:
        pass
