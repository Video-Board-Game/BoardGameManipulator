import numpy as np


class ArmKinematics:
    def __init__(self):
        self.L1 = 0.08375
        self.L2 = np.hypot(.023746,.017)
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
        z=-z

        joint0 = np.zeros(2)
        r=np.sqrt(x**2+y**2)

        d0 = x/r
        c1 = np.sqrt(1-d0**2)
        joint0[0]=np.arctan2(c1,d0)
        joint0[1]=np.arctan2(-c1,d0)

        zc = z-self.L1
        
        r2 = r**2+zc**2

        joint2 = np.zeros(2)
        d2 = (self.L2+self.L3**2-r2)/(2*self.L2*self.L3)
        c2 = np.sqrt(1-d2**2)
        joint2[0]=np.pi/2+self.jointOffset-np.arctan2(c2,d2)
        joint2[1]=np.pi/2+self.jointOffset-np.arctan2(-c2,d2)

        dbeta = (self.L2**2+r2-self.L3**2)/(2*self.L2*np.sqrt(r2))
        cbeta = np.sqrt(1-dbeta**2)
        
        beta=np.zeros(2)
        beta[0]=np.arctan2(cbeta,dbeta)
        beta[1]=np.arctan2(-cbeta,dbeta)

        dalpha = r/np.sqrt(r2)
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
                    joints = np.array([joint0[j],joint1[i],joint2[k]])
                    valid = True
                    for l in range(3):
                        if(joints[l]<self.jointLims[l][0] or joints[l]>self.jointLims[l][1]):
                            valid=False
                            break
                    fkpos=self.fk(joints)
                    if valid and not (fkpos[0,3]-x)**2+(fkpos[1,3]-y)**2+(fkpos[2,3]-z)**2<0.01:
                        valid = False
                    if valid:
                        validAnswer=True
                        validJoints=joints
                        break
                
        return validJoints if validAnswer else None


    def vk(self,joint_val):
        pass

    