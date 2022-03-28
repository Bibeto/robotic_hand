# @brief
# This module is used for calculating the direct kinematics of 4 DOFs robotic arm
# The end result are the (x, y, z) with their simplified equations
import sympy 
import pickle 








NUMBER_OF_TRANSFORMATIONS = 5



print("###########START############")

# Denavit Hartenberg parameters 
(d, theta, r, alpha) = sympy.symbols("d, theta, r, alpha") 
# The 4 DOFs of the robotic arm; 4 revolutions 
(theta1, theta2, theta3, theta4)= sympy.symbols("theta1, theta2, theta3, theta4")



# Creating the default transformation matrice 
T= sympy.Matrix([   
    [sympy.cos(theta),-sympy.sin(theta)*sympy.cos(alpha),  sympy.sin(theta)*sympy.sin(alpha), r*sympy.cos(theta)], 
    [sympy.sin(theta), sympy.cos(theta)*sympy.cos(alpha), -sympy.cos(theta)*sympy.sin(alpha), r*sympy.sin(theta)], 
    [0,                sympy.sin(alpha),                   sympy.cos(alpha),                  d                 ], 
    [0,                0,                                  0,                                 1                 ]  ])


# Creating an array of transformation matrices and subs
tranformation_matrices = []

for i in range(0, NUMBER_OF_TRANSFORMATIONS): 
    tranformation_matrices.append(T.copy())


# transformation matrice 1 base_link__link_01
tranformation_matrices[0] = tranformation_matrices[0].subs(d, 160)
tranformation_matrices[0] = tranformation_matrices[0].subs(theta, 0) 
tranformation_matrices[0] = tranformation_matrices[0].subs(r, 0)
tranformation_matrices[0] = tranformation_matrices[0].subs(alpha, 0)

# transformation matrice 2 link_01__link_02 
tranformation_matrices[1] = tranformation_matrices[1].subs(d, 336)
tranformation_matrices[1] = tranformation_matrices[1].subs(theta, -theta1)  #theta1 minus 
tranformation_matrices[1] = tranformation_matrices[1].subs(r, 0)
tranformation_matrices[1] = tranformation_matrices[1].subs(alpha, sympy.pi/2) 

# transformation matrice 3  link_02__link_03
tranformation_matrices[2] = tranformation_matrices[2].subs(d, 0)
tranformation_matrices[2] = tranformation_matrices[2].subs(theta, -theta2) # theta2 minus
tranformation_matrices[2] = tranformation_matrices[2].subs(r, 500) 
tranformation_matrices[2] = tranformation_matrices[2].subs(alpha, 0) 

# transformation matrice 4 link_03__link_04  
tranformation_matrices[3] = tranformation_matrices[3].subs(d, 0)    
tranformation_matrices[3] = tranformation_matrices[3].subs(theta, -theta3) # theta3 minus
tranformation_matrices[3] = tranformation_matrices[3].subs(r, 427.71) 
tranformation_matrices[3] = tranformation_matrices[3].subs(alpha, 0)

# One translation matrix for that the joints are not aligned 
translation_matrix = sympy.Matrix([                                     
            [1,                 0,                   0,          0], 
            [0,                 1,                   0,     138.65], 
            [0,                 0,                   1,          0], 
            [0,                 0,                   0,          1]   ])

tranformation_matrices[3] = tranformation_matrices[3] * translation_matrix

# # transformation matrice 5 for the end effector
tranformation_matrices[4] = tranformation_matrices[4].subs(d, 0)    
tranformation_matrices[4] = tranformation_matrices[4].subs(theta, theta4) # theta4 
tranformation_matrices[4] = tranformation_matrices[4].subs(r, 0) 
tranformation_matrices[4] = tranformation_matrices[4].subs(alpha, 0) 
translation_matrix = sympy.Matrix([                                     
            [1,                 0,                   0,          0], 
            [0,                 1,                   0,    -180.11], 
            [0,                 0,                   1,          0], 
            [0,                 0,                   0,          1]   ])

tranformation_matrices[4] = tranformation_matrices[4] * translation_matrix


# The multiplication of the transformation matrices
final_transformation_matrice = sympy.eye(4)
for i in range(0, NUMBER_OF_TRANSFORMATIONS): 
    final_transformation_matrice= final_transformation_matrice * tranformation_matrices[i] 

x, y, z, w= final_transformation_matrice * sympy.Matrix([0, 0, 0, 1])

x = sympy.simplify(x)
y = sympy.simplify(y)
z = sympy.simplify(z)


with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/x.pickle', 'wb') as outf : 
    pickle.dump(x, outf)

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/y.pickle', 'wb') as outf : 
    pickle.dump(y, outf)

with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/z.pickle', 'wb') as outf : 
    pickle.dump(z, outf)

print("############END#############")