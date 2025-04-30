import numpy as np


class ArmKinematics:
    def __init__(self):
        # unit in meters
        self.L0 = 0.07 
        self.L1 = np.hypot(0.22, 0.017)  # link2 length X and Y components since it's offset
        self.L2 = 0.225  
        self.L3 = np.hypot(.012235,.163)
        self.jointOffset=np.arctan(0.017/0.22)
        self.jointOffset2 = np.arctan(0.012235/0.163)  # offset for joint 2, used in inverse kinematics
        self.distPerRad = .015708 / (np.pi*2)  # 15.708 is the travel distance of the elevator in mm per rotation,
        self.gripheight=.1
        self.jointLims = [
            (-np.pi/1.5, np.pi/1.5),
            (-np.pi, 0),
            (-np.pi/1.5, np.pi/1.5)
        ]

        # theta, d, a, alpha
        self.dh_table_const = [
            # Base Link (1) to Link 2
            [self.jointOffset-np.pi/2, self.L0, self.L1, 0],
            # Link 2 to Link 3
            [np.pi/2-self.jointOffset , 0, self.L2, 0],
            # Link 3 to EE
            [self.jointOffset2, 0, self.L3, 0]
        ]

        
    def dh2mat(self, joint_val,row):
        """
        Convert Denavit-Hartenberg parameters to a transformation matrix.
        :param joint_val: The joint value for the current joint (theta).
        :param row: The Denavit-Hartenberg parameters [theta, d, a, alpha] for the current joint.
        :return: A 4x4 transformation matrix representing the transformation from the current joint to the next joint.
        """
        theta= joint_val+row[0]
        d=row[1]
        a=row[2]
        alpha=row[3]
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def fk(self,joints):
        """
        Forward kinematics for the arm, calculates the transformation matrix from base to end-effector given joint values.
        :param joints: A list or array of joint values [joint0, joint1, joint2]
        :return: A 4x4 transformation matrix representing the position and orientation of the end-effector in the base frame.
        """
        T = np.eye(4)
        for i in range(3):
            dir = 1
            T = T @ self.dh2mat(dir*joints[i],self.dh_table_const[i])
        
        T[0][3]=T[0][3] #accounting for center offset from FLU
        if len(joints) > 3:
            T[2][3] = self.L0 - joints[3] * self.distPerRad  # Adjust Z position based on joint 3 rotation
        return T
    
    def ik(self,x,y,alpha, tolerance = 0.05, debug=False):
        """Calculate the inverse kinematics for the given x, y, z position."""
       
    
        xprime = x - self.L3*np.cos(alpha) #accounting for center offset from FLU
        yprime = y - self.L3*np.sin(alpha) #accounting for center offset from FLU

        # print(x,y,alpha)
        
        #law of cosines for joint2
        joint2 = np.zeros(2)
        print(np.hypot(xprime,yprime))
        d2 = (self.L1**2+self.L2**2-np.hypot(xprime,yprime)**2)/(2*self.L1*self.L2)
        c2 = np.sqrt(1-d2**2)
        print("d2: ",d2)
        print("c2: ",c2)
        joint2[0]=np.pi/2+self.jointOffset-np.arctan2(c2,d2)
        joint2[1]=np.pi/2+self.jointOffset-np.arctan2(-c2,d2)

        # finding alpha and beta for joint1
        dbeta = (self.L1**2+np.hypot(xprime,yprime)**2-self.L2**2)/(2*self.L1*np.hypot(xprime,yprime))
        cbeta = np.sqrt(1-dbeta**2)       
        beta=np.zeros(2)
        beta[0]=np.arctan2(cbeta,dbeta)
        beta[1]=np.arctan2(-cbeta,dbeta)

        dgamma = -yprime/np.hypot(xprime,yprime)
        cgamma = np.sqrt(1-dgamma**2)
        gamma=np.zeros(2)
        gamma[0]=np.arctan2(cgamma,dgamma)
        gamma[1]=np.arctan2(-cgamma,dgamma)

        joint1 = np.zeros(4)
        joint1[0]=gamma[0]-self.jointOffset-beta[0]
        joint1[1]=gamma[0]-self.jointOffset-beta[1]
        joint1[2]=gamma[1]-self.jointOffset-beta[0]
        joint1[3]=gamma[1]-self.jointOffset-beta[1]



        # Verifying combinations to find correct combinations since with Atan2 to find potential negative angles acos wouldnt
        validAnswer= False
        validJoints = np.zeros(3)
        
        for i in range(4):
            
                for k in range(2):

                    joint3=alpha-joint1[i]-joint2[k]-self.jointOffset2
                    joints = np.array([joint1[i],joint2[k],joint3,0])
                    valid = True
                    for l in range(3):
                        if(joints[l]<self.jointLims[l][0] or joints[l]>self.jointLims[l][1]):
                            valid=False
                            break
                    
                    fkpos=self.fk(joints)
                    # print("FK: ",fkpos)
                    if debug:
                        print("Joints: ",joints)
                        print("x,y: ",x,y)
                        print("fkpos: ",fkpos[0,3],fkpos[1,3])
                        print("Error", np.hypot(fkpos[0,3]-x, fkpos[1,3]-y))
                        print((np.hypot(fkpos[0,3]-x, fkpos[1,3]-y)<tolerance))
                    if (np.hypot(fkpos[0,3]-x, fkpos[1,3]-y)<tolerance):
                        validAnswer=True
                        validJoints=joints
                        break
                    
                if validAnswer:
                    break
            
        # if not validAnswer:
            # print("No valid IK solution found")
            # print("x,y,z: ",x,y,z)
            # print("joint0: ",joint0)
            # print("joint1: ",joint1)
            # print("joint2: ",joint2)
        return validJoints if validAnswer else None


    def vk(self,joint_val):
        """
        Velocity kinematics for the arm, returns the jacobian matrix for the given joint values.
        Generated in matlab using symbolic tool box in vkGeneration.m then translated to python using CoPilot.
        :param joint_val: A list or array of joint values [joint0, joint1, joint2]
        :return: The Jacobian matrix (3x3) for the given joint values.
        """
        tht1, tht2, tht3 = joint_val
        t2 = np.cos(tht1)
        t3 = np.sin(tht1)
        t4 = np.pi / 2.0
        t5 = -t4
        t7 = t4 + tht3 - 7.146907175572792e-2
        t6 = t5 + tht2 + 7.146907175572792e-2
        t9 = np.cos(t7)
        t11 = np.sin(t7)
        t8 = np.cos(t6)
        t10 = np.sin(t6)
        t12 = (t8 * t9) / 4.0
        t13 = (t10 * t11) / 4.0
        t15 = (t2 * t8 * t11) / 4.0
        t16 = (t2 * t9 * t10) / 4.0
        t17 = (t3 * t8 * t11) / 4.0
        t18 = (t3 * t9 * t10) / 4.0
        t14 = -t13
        t19 = -t15
        t20 = -t16
        mt1 = [
            t2 / 50.0 - t3 * t8 * 2.380677458203862e-1 + t3 * t13 - (t3 * t8 * t9) / 4.0,
            t3 * (-1.0 / 50.0) - t2 * t8 * 2.380677458203862e-1 + t2 * t13 - (t2 * t8 * t9) / 4.0,
            0.0,
            t19 + t20 - t2 * t10 * 2.380677458203862e-1,
            t17 + t18 + t3 * t10 * 2.380677458203862e-1,
            t8 * 2.380677458203862e-1 + t12 + t14,
            t19 + t20,
            t17 + t18,
            t12 + t14,
        ]
        jakubian = np.reshape(mt1, (3, 3))   # Reshape to 3x3 
        return jakubian
    

    def calc_elevator_joint(self, z):
        """
        Calculate the joint value for the elevator based on the z position.
        :param z: The z position of the end-effector.
        :return: The joint value for the elevator.
        """
        return (z-self.L0) / self.distPerRad 
        

    def generate_trajectory(self, start, end, start_vel=[0,0,0], start_time=0, end_time=1):
        # TODO: MAKE THIS FUNCTION ACCOUNT FOR THE ARM'S MAX VELOCITY
        #       AND RETURN AN ADJUSTED END TIME
        # NO
        coeffs = np.zeros((3,6))
        def quintic_traj(start_angle, end_angle, start_vel, end_vel, start_acc, end_acc, start_time, end_time):
            answer_vec = np.array([start_angle, start_vel, start_acc, end_angle, end_vel, end_acc])
            # print("AnsVec", answer_vec)
            # print(start_time,end_time)
            polynomial_mat = np.array([
                [1, start_time, start_time**2, start_time**3, start_time**4, start_time**5],
                [0, 1, 2*start_time, 3*start_time**2, 4*start_time**3, 5*start_time**4],
                [0, 0, 2, 6*start_time, 12*start_time**2, 20*start_time**3],
                [1, end_time, end_time**2, end_time**3, end_time**4, end_time**5],
                [0, 1, 2*end_time, 3*end_time**2, 4*end_time**3, 5*end_time**4],
                [0, 0, 2, 6*end_time, 12*end_time**2, 20*end_time**3]
            ])
            coeffs = np.linalg.solve(polynomial_mat, answer_vec)
            return coeffs
        for i,s,e in zip([0,1,2],start,end):
            if s==e:
                coeffs[i] = np.zeros(6)
            else:
                # print(start_vel)
                # third term can be start_vel[i]
                coeffs[i] = quintic_traj(s,e,0,0,0,0,start_time,end_time)

        return coeffs
    
    def check_move_safe(self, joints):
        # Check if the move is safe
        if (self.jointLims[1][0] <= joints[1] <= self.jointLims[1][1] and self.jointLims[2][0] <= joints[2] <= self.jointLims[2][1]):
            return True
        
        fk_pos = self.fk(joints)
        x, y, z = fk_pos[0, 3], fk_pos[1, 3], fk_pos[2, 3]
        
        # TODO: add check to prevent it from going through itself
        
        # if Z is below drone props and 
        if z < 0.15:
            return True
    
        return False

if __name__ == "__main__":
    arm = ArmKinematics()
    print("Testing Arm Kinematics")
    # print("VK: ",arm.vk([0,0,0]))
    t = arm.fk([0,0,0])
    print(t)
    t = arm.ik(0.1941796, -0.0154,0,debug=True)
    print(t)

    # print(np.pi/6)
    # v=arm.vk([0,0,0])
    # print(v)
   