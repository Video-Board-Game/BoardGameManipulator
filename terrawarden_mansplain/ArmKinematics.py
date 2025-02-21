import numpy as np


class ArmKinematics:
    def __init__(self):
        self.L1 = 0.08375
        self.L2 = np.hypot(.23746,.017)
        self.L3 = 0.3125
        self.jointOffset=np.arctan(17/237.46)
        self.jointLims = [
            (-np.pi/2, np.pi/2),
            (-np.pi/2, np.pi/2),
            (-np.pi/2, np.pi/2)
        ]


        self.dh_table_const = [
            [0, -self.L1, 0, np.pi/2],
            [-np.pi/2+self.jointOffset, 0, self.L2, 0],
            [np.pi/2-self.jointOffset, 0, self.L3, -np.pi/2]
        ]

        


    def dh2mat(self, joint_val,row):
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
        T = np.eye(4)
        for i in range(3):
            T = T @ self.dh2mat(joints[i],self.dh_table_const[i])
        return T
    
    def ik(self,x,y,z):
        

        joint0 = np.zeros(2)
        r=np.sqrt(x**2+y**2)

        
        joint0[0]=np.arctan2(y,x)
        joint0[1]=np.arctan2(-y,-x)

        zc = z+self.L1
        
        r2 = r**2+zc**2

        joint2 = np.zeros(2)
        d2 = (self.L2**2+self.L3**2-r2)/(2*self.L2*self.L3)
        c2 = np.sqrt(1-d2**2)
        joint2[0]=np.pi/2+self.jointOffset-np.arctan2(c2,d2)
        joint2[1]=np.pi/2+self.jointOffset-np.arctan2(-c2,d2)

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


        validAnswer= False
        validJoints = np.zeros(3)
        for i in range(4):
            for j in range(2):
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
        if not validAnswer:
            print("No valid IK solution found")
            print("x,y,z: ",x,y,z)
            print("joint0: ",joint0)
            print("joint1: ",joint1)
            print("joint2: ",joint2)
        return validJoints if validAnswer else None


    def vk(self,joint_val):
        l2=self.L2
        l3=self.L3
        theta1=joint_val[0]
        theta2=joint_val[1]
        theta3=joint_val[2]
        t2 = np.cos(theta1)
        t3 = np.sin(theta1)
        t4 = np.pi / 2.0
        t5 = -t4
        t7 = t4 + theta3 - 7.146907175572792e-2
        t6 = t5 + theta2 + 7.146907175572792e-2
        t9 = np.cos(t7)
        t11 = np.sin(t7)
        t8 = np.cos(t6)
        t10 = np.sin(t6)
        t12 = t8 * t11
        t13 = t9 * t10
        t14 = l3 * t8 * t9
        t15 = l3 * t10 * t11
        t16 = t2 * t8 * t9
        t17 = t3 * t8 * t9
        t18 = t2 * t10 * t11
        t19 = t3 * t10 * t11
        t20 = -t12
        t21 = -t13
        t22 = l3 * t2 * t12
        t23 = l3 * t2 * t13
        t24 = l3 * t3 * t12
        t25 = l3 * t3 * t13
        t26 = -t15
        t27 = -t16
        t28 = -t17
        t29 = l3 * t2 * t20
        t30 = l3 * t2 * t21
        t31 = l3 * t3 * t20
        t32 = l3 * t3 * t21
        t33 = t20 + t21
        t34 = t18 + t27
        t35 = t19 + t28
        
        jac = np.array([
            [-t3 * t14 + t3 * t15 - l2 * t3 * t8, t2 * t14 + t2 * t26 + l2 * t2 * t8, 0.0],
            [t3 * t12 + t3 * t13, t2 * t20 + t2 * t21, 0.0],
            [t29 + t30 - l2 * t2 * t10, t31 + t32 - l2 * t3 * t10, t14 + t26 + l2 * t8],
            [t34, t35, t33],
            [t29 + t30, t31 + t32, t14 + t26],
            [t34, t35, t33]
        ])
        
        return jac
    
    def generate_trajectory(self, start, end, duration):
        coeffs = np.zeros((3,6))
        def quintic_traj(start_angle, end_angle, start_vel, end_vel, start_acc, end_acc, start_time, end_time):
            answer_vec = np.array([start_angle, start_vel, start_acc, end_angle, end_vel, end_acc])
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
                coeffs[i] = quintic_traj(s,e,0,0,0,0,0,duration)
        return coeffs

if __name__ == "__main__":
    arm = ArmKinematics()
    t = arm.fk([0,0,0])
    print(t)
    t = arm.fk([np.pi/2,np.pi/2,np.pi/2])
    print(t)
    v=arm.vk([0,0,0])
    print(v)
   