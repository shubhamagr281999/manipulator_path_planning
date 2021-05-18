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
upper_limit_distance = 0.3
lower_limit_distance = 0.15
upper_angle_limit_config = 6.2
lower_angle_limit_config = 0.01
max_number_of_node = 180
link_lengths=[1.05,1.05]
link_offsets=[0,0.025]
pi=3.14159265358


class nodes:
    def __init__(self,point,parent):
        self.config=np.array(point,dtype=float)
        self.parent_node=parent

class path_planner:
    def __init__(self):
        self.obstacles_points_x=[]
        self.obstacles_points_y=[]
        self.rate_move=rospy.Rate(0.5)
        self.obstacles=np.full((900,900),False,dtype=bool)
        self.obstacles[600:870,600:870]=np.full((270,270),True,dtype=bool)
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
        self.pub_joint2=rospy.Publisher('/me604/joint2_position_controller/command',Float64,queue_size=10)
        self.pub_joint1=rospy.Publisher('/me604/joint1_position_controller/command',Float64,queue_size=10)
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
            # self.random_point[0]=random.uniform(lower_angle_limit_config,upper_angle_limit_config)
            # self.random_point[1]=random.uniform(-1*upper_angle_limit_config/2,upper_angle_limit_config/2)
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
                    # print("yoo I found a point time to break")                    
                    point_found= True
        # print('point generated')

    def distance_with_random(self,point):
        #checks distance of the randomly generated point and if in bound returns true
        distance=0.0
        for i in range(dof):
            distance+=(point[i]-self.random_point[i])*(point[i]-self.random_point[i])
        if(distance<upper_limit_distance and distance >lower_limit_distance):
            return True
        return False
    def distance_with_random_compare(self,point1,point2):
        #checks distance of the randomly generated point and if in bound returns true
        distance1=0.0
        distance2=0.0
        for i in range(dof):
            distance1+=(point1[i]-self.random_point[i])*(point1[i]-self.random_point[i])
        for i in range(dof):
            distance2+=(point2[i]-self.random_point[i])*(point2[i]-self.random_point[i])
        if(distance1>distance2):
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
        #checks the global list for the nearest neighbour
        least_distance_node=0
        for i in range(len(self.list_of_nodes)) :
            if(self.distance_with_random_compare(self.list_of_nodes[least_distance_node].config,self.list_of_nodes[i].config)) :
                least_distance_node=i
        if(self.distance_with_random(self.list_of_nodes[least_distance_node].config)) :
            node_obj=nodes(self.random_point,least_distance_node)
            self.list_of_nodes.append(node_obj)
            print(len(self.list_of_nodes))
            return True 
        return False

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
            point_to_check_x=450 + int(cos(point[0])*(i/20.0)*(900/5.0)*link_lengths[0])
            point_to_check_y=450 + int(sin(point[0])*(i/20.0)*(900/5.0)*link_lengths[0])
            if(self.obstacles[point_to_check_x,point_to_check_y]) :
                print("obstacle hai bhai")
                self.obstacles_points_x.append(point[0])
                self.obstacles_points_y.append(point[1])
                return False
            point_to_check_x=int(450 + cos(point[1])*(i/20.0)*(900/5.0)*link_lengths[1] + cos(point[0])*(link_lengths[0]-link_offsets[1])*(720/4))
            point_to_check_y=int(450 + sin(point[1])*(i/20.0)*(900/5.0)*link_lengths[1] + sin(point[0])*(link_lengths[0]-link_offsets[1])*(720/4))
            if(self.obstacles[point_to_check_x,point_to_check_y]) :
                print("obstacle hai bhai")
                self.obstacles_points_x.append(point[0])
                self.obstacles_points_y.append(point[1])
                return False
                
        # print("obstacle free")
        return True


    def plan(self):
        # introduced a delay to allow varaibles to get assigned using call backs
        time.sleep(5)
        while not rospy.is_shutdown():   
            if self.new_point_recieved :
                self.new_point_recieved= False
                print("task initiated")
                node_obj=nodes(self.current_pose,999)
                self.list_of_nodes=[ node_obj ] 
                for i in range(max_number_of_node):
                    self.generate_point()
                time.sleep(3)
                print("200 points generated now need to connect to end point")
                joint1_angles=[]
                joint2_angles=[]
                for i in range(max_number_of_node):
                    joint1_angles.append(self.list_of_nodes[i].config[0])
                    joint2_angles.append(self.list_of_nodes[i].config[1])
                plt.scatter(joint1_angles, joint2_angles, label= "stars", color= "green",marker= "*", s=30)
                plt.scatter(self.obstacles_points_x, self.obstacles_points_y, label= "stars", color= "red",marker= "*", s=30)
                print("plot bana")
                plt.show()
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
                    while (way_point<(max_number_of_node+20)) :
                        self.path.append(way_point)
                        # print("append")
                        way_point=self.list_of_nodes[way_point].parent_node
                    print("path generated")
                    print(self.path)
                    x=[]
                    y=[]
                    for i in range(len(self.path)):
                        x.append(self.list_of_nodes[self.path[i]].config[0])
                        y.append(self.list_of_nodes[self.path[i]].config[1])
                    plt.plot(x,y)
                    plt.show()
                    self.random_point=[]


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
