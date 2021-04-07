#!/usr/bin/env python
import rospy
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin
from nav_msgs.msg import *
from drone_path_planner.msg import teleopData
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from drdo_exploration.msg import direction
#from geometry_msgs import PoseStamped
class navigation:
    def __init__(self):
        self.rate=rospy.Rate(10)
        self.x_pose=0.0
        self.y_pose=0.0
        self.z_pose=0.0
        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0
        self.delta=0.0
        self.decision=0
        self.i=1
        self.msg_test=direction()
        # self.pub_set_point_local=rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
        # self.sub1=rospy.Subscriber("/mavros/global_position/local",Odometry, self.gps_data_callback)
        # self.subl2=rospy.Subscriber("/drone/teleop",teleopData,self.decision_calback)
        self.subl3=rospy.Subscriber("chatter",direction,self.test_callback,queue_size=1)
        self.pub_test=rospy.Publisher('chatter',direction,queue_size=10)
        self.msgp=PoseStamped()
    def test_callback(self,msg):
        print("I heard:",msg.vec_x[1])
        rospy.sleep(0.5)
    def nav(self):
        while not rospy.is_shutdown():            
            pub_config=[]
            pub_config.append(1.0)
            pub_config.append(3.0)
            pub_config.append(2.0)
            self.msg_test.vec_x=pub_config
            self.pub_test.publish(self.msg_test)
            # print('I published:' , self.i)
            self.rate.sleep()

        

if __name__ == '__main__':
  rospy.init_node('aruco_navigator_node')
  rospy.loginfo("navigator_node created")
  try:
    navigation_obj = navigation()  
    while (rospy.get_time()==0):
      pass
    rospy.loginfo("Beginning to accept the commands for teleop...")
    navigation_obj.nav()

  except rospy.ROSInterruptException:
    pass
