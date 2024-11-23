import numpy as np
import fk
import matplotlib.pyplot as plt
import time

class Visualizer:
    def __init__(self, dh_table=fk.dh_table_const, base_link_offset=fk.base_joint_offset_dh):
        self.dh_table = dh_table
        self.base_link_offset = base_link_offset
        
        # store the figure and the plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
              
        # set the view angle in degrees
        self.ax.view_init(azim=120, elev=15)
                
        # set the labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # Invert the Z-self.axis just for the purposes of visualization
        # self.ax.set_zlim(self.ax.get_zlim()[::-1])
        
        # set the title
        self.ax.set_title('Robot Arm Visualization')
        
        

    # function to visualize the robot that takes in the joint angles (and possibly a DH table)
    # in this case the arm is hanging down, so I shall reverse the values 
    def visualize_arm(self, q_arr, wait_time=1):
        '''
        Visualize the robot given the joint angles as an array and a corresponding DH table.
        Values are in radians.
        '''
        
        # verify the input
        if len(q_arr) != len(self.dh_table):
            raise ValueError("The number of joint angles must match the number of joints in the DH table.")
        
        # clear the plot
        self.ax.clear()
        self.ax.set_xlim([-0.5, 0.8])
        self.ax.set_ylim([-0.8, 0.8])
        self.ax.set_zlim([-0.8, 0.5])        
        
        #TODO: refactor code an fk2 module that computes the incremental transformation matrices for each joint
        T_00 = np.eye(4) # origin
        T_11 = fk.fk_arbitrary(q_arr[0:1], self.dh_table[0:1]) # location of joint 1
        T_12 = fk.fk_arbitrary(q_arr[0:2], self.dh_table[0:2]) # location of joint 2
        T_13 = fk.fk_arbitrary(q_arr[0:3], self.dh_table[0:3]) # location of joint 3
        
        # plot the origin
        self.ax.scatter(0, 0, 0, c='k', marker='o')
        
        # plot the joints    
        self.ax.scatter(T_11[0,3], T_11[1,3], T_11[2,3], c='b', marker='o')
        self.ax.scatter(T_12[0,3], T_12[1,3], T_12[2,3], c='g', marker='o')
        self.ax.scatter(T_13[0,3], T_13[1,3], T_13[2,3], c='y', marker='o')
        
        # plot the links
        self.ax.plot([T_00[0,3], T_11[0,3]], [T_00[1,3], T_11[1,3]], [T_00[2,3], T_11[2,3]], c='b')
        self.ax.plot([T_11[0,3], T_12[0,3]], [T_11[1,3], T_12[1,3]], [T_11[2,3], T_12[2,3]], c='g')
        self.ax.plot([T_12[0,3], T_13[0,3]], [T_12[1,3], T_13[1,3]], [T_12[2,3], T_13[2,3]], c='y')
        
        # draw a rectangle on XY plane around origin to represent the base
        # the base is 0.2m wide and 0.4m long, centered at the origin
        self.ax.plot([-0.2, 0.2], [-0.1, -0.1], [0, 0], c='k')
        self.ax.plot([-0.2, 0.2], [0.1, 0.1], [0, 0], c='k')
        self.ax.plot([-0.2, -0.2], [-0.1, 0.1], [0, 0], c='k')
        self.ax.plot([0.2, 0.2], [-0.1, 0.1], [0, 0], c='k')
        
        # plot the EE location XYZ values as text
        string = 'EE: ({:.3f}, {:.3f}, {:.3f})'.format(T_13[0,3], T_13[1,3], T_13[2,3])
        self.ax.text(T_13[0,3], T_13[1,3], T_13[2,3], string, color='k')
        
        # plot the joint location in degrees as text in the legend
        j1_str = 'J1: {:4.2f}°'.format(np.degrees(q_arr[0]))
        j2_str = 'J2: {:4.2f}°'.format(np.degrees(q_arr[1]))
        j3_str = 'J3: {:4.2f}°'.format(np.degrees(q_arr[2]))
        self.ax.legend([j1_str, j2_str, j3_str], loc='upper right')
        # assign colors
        self.ax.get_legend().legend_handles[0].set_color('b')
        self.ax.get_legend().legend_handles[1].set_color('g')
        self.ax.get_legend().legend_handles[2].set_color('y')        
        

        # display the plot for a specified amount of time
        plt.pause(wait_time)
        
    def wait(self):
        plt.show()
        
    
    
visualiz = Visualizer()

start_q = [0, 0, 0]
end_q = [-np.pi/4, np.pi/3, np.pi/4]
steps = 100
total_time = 5

for i in range(steps):
    q_arr = [start_q[j] + (end_q[j] - start_q[j]) * i / steps for j in range(3)]
    
    visualiz.visualize_arm(q_arr, wait_time=total_time/steps)

visualiz.wait()