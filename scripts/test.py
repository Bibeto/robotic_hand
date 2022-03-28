#!/usr/bin/env python3
import sympy
import rospy
from std_msgs.msg import Float64
import sympy 
import pickle
from robot import robot_IK
from robot import initial_position








    
    
with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/x.pickle', 'rb') as inf:    
    x = pickle.loads(inf.read())

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/y.pickle', 'rb') as inf:    
    y = pickle.loads(inf.read())

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/z.pickle', 'rb') as inf:    
    z = pickle.loads(inf.read())


theta1, theta2, theta3, theta4 = sympy.symbols("theta1 theta2 theta3 theta4")

actual_position = sympy.Matrix([x.copy(), y.copy(), z.copy()])


x_given = float(input("Give x: "))
y_given = float(input("Give y: "))
z_given = float(input("Give z: "))


next_position = sympy.Matrix([x_given, y_given, z_given])


my_robot = robot_IK.Robot_IK(initial_position.INITIAL_POSITION_0, initial_position.INITIAL_ANGLES_0)
angle1, angle2, angle3, angle4 = my_robot.calculate_joint_rotation(next_position)
my_robot.set_q0(my_robot.calculate_joint_rotation(next_position))
my_robot.set_X0(next_position)


actual_position = actual_position.subs(theta1, angle1)
actual_position = actual_position.subs(theta2, angle2)
actual_position = actual_position.subs(theta3, angle3)
actual_position = actual_position.subs(theta4, angle4)

print("actual position: ", actual_position)
print("wanted position: ", next_position)


rospy.init_node('test', anonymous=True)

rate = rospy.Rate(1) 

pub1 = rospy.Publisher('/robotic_hand/joint1_position_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/robotic_hand/joint2_position_controller/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/robotic_hand/joint3_position_controller/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/robotic_hand/joint4_position_controller/command', Float64, queue_size=10)

theta1_p = Float64()
theta2_p = Float64()
theta3_p = Float64()
theta4_p = Float64()

theta1_p.data =  float(angle1)
theta2_p.data =  float(angle2)
theta3_p.data =  float(angle3)
theta4_p.data =  float(angle4)

while not rospy.is_shutdown():   
    pub1.publish(theta1_p)
    pub2.publish(theta2_p)
    pub3.publish(theta3_p)
    pub4.publish(theta4_p)
    rate.sleep()