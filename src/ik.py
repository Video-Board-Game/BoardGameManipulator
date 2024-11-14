import fk
import vk
import numpy as np


#Inverse Kinematics
def arm2ik(x,y,z):
    # Define the desired position of the end effector
    x_d = x
    y_d = y
    z_d = z

    # Define the initial guess for the joint angles
    # theta0_d = np.arctan2(z_d,x_d)
    # theta1_d = np.arccos(y_d/(.18+.18))
    # theta2_d = 0

    theta0_d = np.arctan2(x_d,-z_d)*1.25
    
    theta2_d = -np.arctan2(x_d,-z_d)/2

    theta1_d = np.arcsin(y_d/(.18+.18*np.cos(theta2_d))) * np.sign(theta0_d)

    # Define the number of iterations
    N = 2000

    # Define the learning rate
    alpha = 1

    # #Compute the current position of the end effector
    # x_c, y_c, z_c = fk.arm2fk(theta0_d, theta1_d, theta2_d)[0:3, 3]

    # # Compute the error
    # e = np.array([x_d - x_c, y_d - y_c, z_d - z_c])

    for i in range(N):
        # Compute the current position of the end effector
        x_c, y_c, z_c = fk.arm2fk(theta0_d, theta1_d, theta2_d)[0:3, 3]

        # Compute the error
        e = np.array([x_d - x_c, y_d - y_c, z_d - z_c])

        if(np.linalg.norm(e) < 0.0001):
            break

        # Compute the Jacobian
        J = vk.arm2vk(theta0_d, theta1_d, theta2_d)
        J = np.linalg.inv(J)
        # Update the joint angles
        theta0_d += (alpha * J[0, :].dot(e))
        theta1_d += (alpha * J[1, :].dot(e))
        theta2_d += (alpha * J[2, :].dot(e))

        

    return theta0_d, theta1_d, theta2_d, i
