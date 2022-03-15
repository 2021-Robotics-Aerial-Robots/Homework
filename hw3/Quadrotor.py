"""
Class for plotting quadcopter
reference : PythonRobotics/ Daniel Ingram
"""

from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt

class Quadrotor():
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, size=0.30, show_animation=True):
        
        self.m = 2.0
        
        # rotation matrix
        self.R = np.eye(3);
        
        self.Ixx = 1
        self.Iyy = 1.2
        self.Izz = 1
        self.cf = 0.01
        self.L1 = size/2
        self.L2 = size/2 
        self.p1 = np.array([self.L1, 0, 0, 1]).T
        self.p2 = np.array([-self.L1, 0, 0, 1]).T
        self.p3 = np.array([0, self.L2, 0, 1]).T
        self.p4 = np.array([0, -self.L2, 0, 1]).T

        
        self.x_data = []
        self.y_data = []
        self.z_data = []
        
        self.allocation_matrix = np.array([[1 ,1, 1, 1],[self.L1, -self.L1 ,-self.L1 ,self.L1],
                                           [-self.L2 ,-self.L2 ,self.L2 ,self.L2], [self.cf, -self.cf, self.cf, -self.cf]])
        
        self.invallocation_matrix = np.linalg.inv(self.allocation_matrix)
        
        self.show_animation = show_animation

        if self.show_animation:
            plt.ion()
            fig = plt.figure()
            # for stopping simulation with the esc key.
            fig.canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            self.ax = fig.add_subplot(111, projection='3d')

        self.update_pose(x, y, z, roll, pitch, yaw)

    def update_pose(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        if self.show_animation:
            self.plot()
    def transformation_matrix(self):
        
        x = self.x
        y = self.y
        z = self.z
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch)
              * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw), z]
             ])

    def plot(self):  # pragma: no cover
        T = self.transformation_matrix()

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        plt.cla()

        self.ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0]],
                     [p1_t[1], p2_t[1], p3_t[1], p4_t[1]],
                     [p1_t[2], p2_t[2], p3_t[2], p4_t[2]], 'k.')

        self.ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
                     [p1_t[2], p2_t[2]], 'r-')
        self.ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
                     [p3_t[2], p4_t[2]], 'r-')

        self.ax.plot(self.x_data, self.y_data, self.z_data, 'b:')

        plt.xlim(-3, 6)
        plt.ylim(-3, 6)
        self.ax.set_zlim(0, 3)

        plt.pause(0.001)

  