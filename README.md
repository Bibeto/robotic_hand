# robotic_hand

This project is based on a 4DOFs robotic hand. The objective is to write an Inverse Kinematic Solver from scratch based on the Jacobian Matrix of the said robot in ROS. 

The robot's tree is as follows: 

base_link-->link_01-->link_02-->link_03-->end_effector

The URDF for robot was generated through SolidWorks's plugin.

The transformation matrices that link base_link to end_effector are based on Hartenberg Convention

Sympy was used for the analytical calculation of the inverse and forward kinemtics. The aftermentioned equations were saved as pickle files(in folder matrices) to be used every instance of calculation of FK and IK.  

A little demo of the robot when given (x, y, z) coordinates: 




https://user-images.githubusercontent.com/43536377/199139052-184f5e2b-044c-434f-80bf-ae2927f6ca89.mp4

