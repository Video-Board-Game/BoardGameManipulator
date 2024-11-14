import numpy as np

def arm2fk(q1, q2, q3):
    """
    Compute the forward kinematics for the given joint angles q1, q2, and q3.
    """
    t2 = np.cos(float(q1))
    t3 = np.cos(float(q2))
    t4 = np.cos(float(q3))
    t5 = np.sin(float(q1))
    t6 = np.sin(float(q2))
    t7 = np.sin(float(q3))
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


