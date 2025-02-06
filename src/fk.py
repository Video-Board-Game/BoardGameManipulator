import numpy as np

#Generated FK ported from MATLAB
def arm2fk(q1, q2, q3):
    """
    Compute the forward kinematics for the given joint angles q1, q2, and q3.
    """
    q1 = float(q1)
    q2 = float(q2)
    q3 = float(q3)
    t2 = np.cos(q1)
    t3 = np.cos(q2)
    t4 = np.cos(q3)
    t5 = np.sin(q1)
    t6 = np.sin(q2)
    t7 = np.sin(q3)
    t8 = t3 * 2.0825e-1
    t9 = t4 * 3.8825e-1
    t10 = t8 - 2.0825e-1
    t11 = t9 - 3.8825e-1

    T = np.array([
        [t2 * t7 + t3 * t4 * t5, t4 * t6, t5 * t7 - t2 * t3 * t4, 0.0],
        [-t5 * t6, t3, t2 * t6, 0.0],
        [t2 * t4 - t3 * t5 * t7, -t6 * t7, t4 * t5 + t2 * t3 * t7, 0.0],
        [t5 * (-3.825e-2) + t2 * t7 * (9.0 / 5.0e+1) - t5 * t10 + t3 * t4 * t5 * 5.6825e-1 - t3 * t5 * t11, t6 * (-2.0825e-1) + t4 * t6 * 5.6825e-1 - t6 * t11, t2 * 3.825e-2 + t2 * t10 + t5 * t7 * (9.0 / 5.0e+1) - t2 * t3 * t4 * 5.6825e-1 + t2 * t3 * t11 - 3.825e-2,1.0]
    ])

    return T.T

# Define the DH table for the three joints
# each row corresponds to a joint, in the format of: [theta, d, a, alpha]
#   theta: the angle about the previous z-axis
#   d: the distance along the previous z-axis
#   a: the distance along the x-axis to the next joint
#   alpha: the angle about the x-axis to the next joint
# in meters and radians
dh_table_const = [
    [0, -0.08375, 0, np.pi/2],
    [-np.pi/2+np.arctan(17/237.46), 0, np.hypot(237.46,17), 0],
    [np.pi/2, 0, 0.262, 0]
]




# function to turn a single row of the DH table into a transformation matrix
def dh2mat(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    
# function to compute the forward kinematics of the robot
def fk_arm1(q_arr, dh_table=dh_table_const):
    '''
    Compute the forward kinematics of the robot given the joint angles q1, q2, and q3.
    Values are in radians.
    Designed only for rotational joints.
    '''
    
    # verify the input
    if len(q_arr) != len(dh_table):
        raise ValueError("The number of joint angles must match the number of joints in the DH table.")
    
    # compute the transformation matrices for each joint
    T_matrices = []
    for i in range(len(q_arr)):
        T_matrices.append(dh2mat(q_arr[i] + dh_table[i][0], dh_table[i][1], dh_table[i][2], dh_table[i][3]))
    
    # compute the total transformation matrix
    for T_i in T_matrices:
        T = T @ T_i
        
    return T

