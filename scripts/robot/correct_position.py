import sympy 



def calculate_quadratic_error(actual_position, desired_position): 
    quadratic_error = sympy.sqrt(
             (actual_position[0]-desired_position[0])**2 
            +(actual_position[1]-desired_position[1])**2 
            +(actual_position[2]-desired_position[2])**2
     )
    return quadratic_error