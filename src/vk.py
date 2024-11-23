import numpy as np

#Generated VK ported from MATLAB
def arm2vk(q1, q2, q3):
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

    mt1 = [
        t2 * (-3.825e-2) - t2 * t10 - t5 * t7 * (9.0 / 5.0e+1) + t2 * t3 * t4 * 5.6825e-1 - t2 * t3 * t11,
        0.0,
        t5 * (-3.825e-2) + t2 * t7 * (9.0 / 5.0e+1) - t5 * t10 + t3 * t4 * t5 * 5.6825e-1 - t3 * t5 * t11,
        t5 * t6 * 2.0825e-1 - t4 * t5 * t6 * 5.6825e-1 + t5 * t6 * t11,
        -t8 + t3 * t4 * 5.6825e-1 - t3 * t11,
        t2 * t6 * (-2.0825e-1) + t2 * t4 * t6 * 5.6825e-1 - t2 * t6 * t11,
        t2 * t4 * (9.0 / 5.0e+1) - t3 * t5 * t7 * (9.0 / 5.0e+1)
    ]

    mt2 = [
        t6 * t7 * (-9.0 / 5.0e+1),
        t4 * t5 * (9.0 / 5.0e+1) + t2 * t3 * t7 * (9.0 / 5.0e+1)
    ]

    J = np.reshape(np.array(mt1 + mt2), (3, 3))
    return J.T


