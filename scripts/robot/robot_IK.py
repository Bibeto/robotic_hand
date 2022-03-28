import pickle
import sympy





class Robot_IK : 
    def __init__(self, initial_postion, initial_angles):
        # initial position (x, y, z)
        self.__X0 = initial_postion
        # intial angles (theta1, theta2, theta3, theta4)
        self.__q0 = initial_angles

        with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/jacobian_inversed.pickle', 'rb') as inf:    
            self.__jacobian_inversed = pickle.loads(inf.read())

        with open('/home/bibeto/ros_ws/src/robotic_hand/matrices_saved/h_det.pickle', 'rb') as inf:    
            self.__h_det = pickle.loads(inf.read())
        
        
    def calculate_joint_rotation(self, next_position): 

        theta1, theta2, theta3, theta4 = sympy.symbols("theta1 theta2 theta3 theta4")

        # need it for verifying singularities (h_det = 0 ===> this position is a singularity) 
        # h_det = self.__h_det.copy()
        # h_det = h_det.subs(theta1, self.__q0[0])
        # h_det = h_det.subs(theta2, self.__q0[1])
        # h_det = h_det.subs(theta3, self.__q0[2])
        # h_det = h_det.subs(theta4, self.__q0[3])

        jacobian_inversed = self.__jacobian_inversed.copy()
        jacobian_inversed= jacobian_inversed.subs(theta1, self.__q0[0])
        jacobian_inversed= jacobian_inversed.subs(theta2, self.__q0[1])
        jacobian_inversed= jacobian_inversed.subs(theta3, self.__q0[2])
        jacobian_inversed= jacobian_inversed.subs(theta4, self.__q0[3])

        delta_x = next_position - self.__X0

        return ( (jacobian_inversed * delta_x) + self.__q0 ) 


    def set_q0(self, new_angular_position): 
        self.__q0 = new_angular_position

    def get_q0(self): 
        return self.__q0

    def set_X0(self, new_direct_postion): 
        self.__X0 = new_direct_postion

    def get_X0(self):
        return self.__X0