import numpy as np


class ArmKinematics:
    def __init__(self):
        # unit in meters
        self.L0 = 0.0275 # shift forward from center of FLU
        tuned_vertical_offset = 0.07 # tuned offset to grasp can better
        self.L1 = 0.20375 - 0.020 + tuned_vertical_offset   # vertical offset down from Pixhawk to end of base link 1
        self.L2 = np.hypot(0.23746, 0.017)  # link2 length X and Y components since it's offset
        self.L3 = 0.25  
        # L4 originally = -0.02
        # I tried to tune L4 to fix the offset, but increasing it too much caused issues within IK
        # Would appreciate someone else take a look -K
        self.L4 = -0.01  # horizontal offset of gripper center

        self.jointOffset=np.arctan(0.017/0.23746)
        self.jointLims = [
            (-np.pi/2, np.pi/2),
            (-np.pi/2, np.pi/2),
            (-np.pi/2, np.pi/2)
        ]

        # theta, d, a, alpha
        self.dh_table_const = [
            # Base Link (1) to Link 2
            [0, -self.L1, 0, np.pi/2],
            # Link 2 to Link 3
            [-np.pi/2 + self.jointOffset, 0, self.L2, 0],
            # Link 3 to EE
            [np.pi/2 - self.jointOffset, self.L4, self.L3, -np.pi/2]
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
            if i == 0:
                dir = -1#to account for joint0 being flipped since we arent using symbolic DH parameters
            T = T @ self.dh2mat(dir*joints[i],self.dh_table_const[i])
        T[0][3]=T[0][3]+self.L0 #accounting for center offset from FLU
        return T
    
    def ik(self,x,y,z):
        """Calculate the inverse kinematics for the given x, y, z position."""
        x=x-self.L0 #accounting for center offset from FLU
        joint0 = np.zeros(1)
        r=np.sqrt(x**2+y**2)

        offset = np.arcsin(self.L4/r) if r>self.L4 else 0 # to avoid singularity when r < L4
        # print("offset: ",offset)
        r = r*np.cos(offset) # adjust r to account for the offset caused by L4
        initial_theta = np.arctan2(y,x) # initial theta for joint0
        new_theta = initial_theta - offset # adjust theta to account for the offset caused by L4
        new__r =r*np.cos(offset)

        joint0[0]=-new_theta # joint0 angle, adjust for L4 offset

        # Now we can proceed with joint1 and joint2 calculations

        # Recenter to RZ plane origin at joint1 base
        zc = z+self.L1
        r2 = new__r**2+zc**2
        
        #lay of cosines for joint2
        joint2 = np.zeros(2)
        d2 = (self.L2**2+self.L3**2-r2)/(2*self.L2*self.L3)
        c2 = np.sqrt(1-d2**2)
        joint2[0]=np.pi/2+self.jointOffset-np.arctan2(c2,d2)
        joint2[1]=np.pi/2+self.jointOffset-np.arctan2(-c2,d2)

        # finding alpha and beta for joint1
        dbeta = (self.L2**2+r2-self.L3**2)/(2*self.L2*np.sqrt(r2))
        cbeta = np.sqrt(1-dbeta**2)       
        beta=np.zeros(2)
        beta[0]=np.arctan2(cbeta,dbeta)
        beta[1]=np.arctan2(-cbeta,dbeta)

        dalpha = -zc/np.sqrt(r2)
        calpha = np.sqrt(1-dalpha**2)
        alpha=np.zeros(2)
        alpha[0]=np.arctan2(calpha,dalpha)
        alpha[1]=np.arctan2(-calpha,dalpha)

        joint1 = np.zeros(4)
        joint1[0]=alpha[0]-self.jointOffset-beta[0]
        joint1[1]=alpha[0]-self.jointOffset-beta[1]
        joint1[2]=alpha[1]-self.jointOffset-beta[0]
        joint1[3]=alpha[1]-self.jointOffset-beta[1]

        # Verifying combinations to find correct combinations since with Atan2 to find potential negative angles acos wouldnt
        validAnswer= False
        validJoints = np.zeros(3)
        print(joint0)
        print(joint1)
        print(joint2)
        for i in range(4):
            for j in range(1):
                for k in range(2):
                    
                    joints = np.array([joint0[j],(j*-2+1)*joint1[i],joint2[k]])
                    valid = True
                    for l in range(3):
                        if(joints[l]<self.jointLims[l][0] or joints[l]>self.jointLims[l][1]):
                            valid=False
                            break
                    fkpos=self.fk(joints)
                    if valid and not ((fkpos[0,3]-x)**2+(fkpos[1,3]-y)**2+(fkpos[2,3]-z)**2<0.01):
                        valid = False
                    if valid:
                        validAnswer=True
                        validJoints=joints
                        break
                if validAnswer:
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
    print("FK: ",arm.fk([0,0,0]))
    print("VK: ",arm.vk([0,0,0]))
    # t = arm.fk([np.pi/6,0,0])
    # print(t)
    # t = arm.ik(t[0][3],t[1][3],t[2][3])
    # print(t)
    # print(np.pi/6)
    # v=arm.vk([0,0,0])
    # print(v)
   