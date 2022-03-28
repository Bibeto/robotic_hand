# @brief
# calculating the jacobian and savin it for later use 
# The determinant of h h_det tells us if we're facing a singularity in q0
import pickle
import sympy 






with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/x.pickle', 'rb') as inf:    
    x = pickle.loads(inf.read())

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/y.pickle', 'rb') as inf:    
    y = pickle.loads(inf.read())

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/z.pickle', 'rb') as inf:    
    z = pickle.loads(inf.read())


theta1, theta2, theta3, theta4 = sympy.symbols("theta1 theta2 theta3 theta4")

J  = sympy.Matrix([[sympy.diff(x, theta1), sympy.diff(x, theta2), sympy.diff(x, theta3), sympy.diff(x, theta4)], 
                   [sympy.diff(y, theta1), sympy.diff(y, theta2), sympy.diff(y, theta3), sympy.diff(y, theta4)], 
                   [sympy.diff(z, theta1), sympy.diff(z, theta2), sympy.diff(z, theta3), sympy.diff(z, theta4)]  ])

J = sympy.simplify(J)
h = J * J.transpose()

A = sympy.Matrix(3, 3, sympy.symbols('A:3:3'))
h_det = A.det().subs(zip(list(A), list(h)))
h_inversed = (1/h_det) * h.adjugate()
J_inversed = J.transpose() * h_inversed

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/jacobian.pickle', 'rb') as inf:    
   J = pickle.loads(inf.read())

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/J_inversed.pickle', 'rb') as inf:    
   J_inversed = pickle.loads(inf.read())

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/h_inversed_cheat.pickle', 'rb') as inf:    
   h_inversed = pickle.loads(inf.read())