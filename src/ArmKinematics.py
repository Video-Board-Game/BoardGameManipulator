import numpy as np


class ArmKinematics:
    def __init__(self):
        self.L1 = 0.08375
        self.L2 = np.hypot(.023746,.017)
        self.L3 = 0.262
        self.jointOffset=np.arctan(17/237.46)


        self.dh_table_const = [
            [0, -self.L1, 0, np.pi/2],
            [-np.pi/2+self.jointOffset, 0, self.L2, 0],
            [np.pi/2-self.jointOffset, 0, self.L3, 0]
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
        pass

    def vk(self,joint_val):
        pass

    