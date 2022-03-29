#!/usr/bin/env python3
import sympy
import rospy
from std_msgs.msg import Float64
import sympy 
from robot import robot_IK
from robot import initial_position
import tf 
import time
# from robot import correct_position





rospy.init_node('test', anonymous=True)

# A listener to get the coordinates of the end effector    
listener = tf.TransformListener()

theta1, theta2, theta3, theta4 = sympy.symbols("theta1 theta2 theta3 theta4")







pub1 = rospy.Publisher('/robotic_hand/joint1_position_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/robotic_hand/joint2_position_controller/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/robotic_hand/joint3_position_controller/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/robotic_hand/joint4_position_controller/command', Float64, queue_size=10)

theta1_p = Float64()
theta2_p = Float64()
theta3_p = Float64()
theta4_p = Float64()

my_robot = robot_IK.Robot_IK(initial_position.INITIAL_POSITION_0, initial_position.INITIAL_ANGLES_0)


def move(x, y, z): 
    next_position = sympy.Matrix([x, y, z])
    angle1, angle2, angle3, angle4 = my_robot.calculate_joint_rotation(next_position)

    theta1_p.data =  float(angle1)
    theta2_p.data =  float(angle2)
    theta3_p.data =  float(angle3)
    theta4_p.data =  float(angle4)

    
    pub1.publish(theta1_p)
    pub2.publish(theta2_p)
    pub3.publish(theta3_p)
    pub4.publish(theta4_p)

    # just enough time to get to the final position correctly
    time.sleep(1.5) 

    
    trans, rot = listener.lookupTransform('/base_link', '/end_effector', rospy.Time())
    
    
    actual_position = sympy.Matrix([float(trans[0]*1000), float(trans[1]*1000), float(trans[2]*1000)])

    print("actual position  x :", actual_position[0], "\t y : ", actual_position[1], "\t z: ", actual_position[2])
    print("wanted position  x :", next_position[0], "\t y : ", next_position[1], "\t z: ", next_position[2])

    my_robot.set_q0(sympy.Matrix([angle1, angle2, angle3, angle4]))
    my_robot.set_X0(actual_position)


def main(): 
    while not rospy.is_shutdown():   
        
        x_given = float(input("Give x: "))
        y_given = float(input("Give y: "))
        z_given = float(input("Give z: "))

        next_position = sympy.Matrix([x_given, y_given, z_given])

        
        # while (True): 
        move(x_given, y_given, z_given)
    

main()