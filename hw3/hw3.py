import time
from math import cos, sin, acos
import numpy as np
from Quadrotor import Quadrotor
from mpl_toolkits.mplot3d import Axes3D

show_animation = True

# Simulation parameters
g = 9.81

Ixx = 1
Iyy = 1
Izz = 1
J = np.eye(3)
T = 5

# Proportional coefficients
# translation
Kp_z = 10

# rotation
Kp_yaw = -30
Kp_roll = -50
Kp_pitch = -50

# Derivative coefficients
# translation

Kd_z = 6

# rotation
Kd_yaw = -10
Kd_roll = -15
Kd_pitch = -15

def quad_sim():

    # initial condition
    x_pos = 0
    y_pos = 0
    z_pos = 0
    
    x_vel = 0
    y_vel = 0
    z_vel = 0
    
    x_acc = 0
    y_acc = 0
    z_acc = 0
    
    roll = 0
    pitch = 0
    yaw = 0
    
    eul_angle = np.array([[0],[0],[0]]); 
    angular_vel = np.array([[0],[0] ,[0]])

    # desire angular velocity, we want the quadcopter to avoid rotating
    des_yaw_rate = 0
    des_roll_rate = 0
    des_pitch_rate = 0

    # time initialize
    dt = 0.1
    t = 0

    # initialize the quadcopter with ic above
    q = Quadrotor(x=x_pos, y=y_pos, z=z_pos, roll=roll,
                  pitch=pitch, yaw=yaw, size=1, show_animation=show_animation)
    
    q.R = rotation_matrix(roll, pitch, yaw)

    # counts for trajectory
    i = 0
    n_run = 3
    irun = 0
    start = time.time()
    
    # desire identity orientation
    ''' Homework3
    des_R = I_3
    
    des_R = .....
    
    '''
    while True:
        
        while t <= T:
            
        
            des_z_pos = np.array([2])
            des_z_vel = np.array([0])
            des_z_acc = np.array([0])
            
            ''' Homework3
            
            # get rel_R (rotation matrix from current orientation to desired orientation)
            
            rel_R = ....
            
            # get the rotation axis , you can see this as eigen vector (with eigen value = 1)
            
            axis = .....

            # get the rotation angle of the rotation axis
            angle_of_axis = .....
            
            # get the rotation error 
            rotation_error = .....
            
                        
            '''
            
            desire_w = np.array([des_roll_rate,des_pitch_rate,des_yaw_rate]).reshape(3,1)
            ew = angular_vel - (q.R.T @ des_R) @ desire_w
            

            '''
            ---------------Error definition------------------
            '''
            
            # ------ For P control ------
            
            # position error 
            ex_z = des_z_pos - z_pos

            # angle error
            er_roll = rotation_error[0]
            er_pitch = rotation_error[1]
            er_yaw = rotation_error[2]
    
    
            # ------ For D control ------
            
            # velocity error
            ev_z = des_z_vel - z_vel

            # angular velocity error
            ew_roll = ew[0]
            ew_pitch = ew[1]
            ew_yaw = ew[2]
            
            '''
            ---------------Controller design------------------
            '''
            
            # translation controller
            total_thrust = q.m * (g + des_z_acc + Kp_z * ex_z + Kd_z * ev_z)
            
            # rotation controller
            roll_torque = (Kp_roll * er_roll + Kd_roll * ew_roll ) * q.Ixx
            pitch_torque = (Kp_pitch * er_pitch + Kd_pitch * ew_pitch) * q.Iyy
            yaw_torque = (Kp_yaw * er_yaw + Kd_yaw * ew_yaw) * q.Izz
            total_moment = np.array([ np.array([roll_torque]), np.array([pitch_torque]), yaw_torque], dtype=object)
            
            # motor force allocation
            control_input = np.concatenate([np.array([total_thrust]),total_moment])
            motor_force = q.invallocation_matrix @ control_input
            
            '''
            ---------------------Dynamics update -----------------------
            '''
            
            # Get total_thrust and total_moment from motor 
            control_input_real = q.allocation_matrix @ motor_force
            
            total_thrust_real = control_input_real[0]
            total_moment_real = control_input_real[1:]
            
            
            # -------------- Rotation dynamics update --------------------
            # equation (2) in week4
            angular_acc = (np.linalg.inv(J)) @ (total_moment_real - vec_cross(eul_angle, np.matmul(J,eul_angle)));
            # add noise (ex: wind, external force....)
            angular_acc = add_noise(angular_acc,0.01)

            angular_vel = angular_vel + angular_acc*dt
            eul_angle = eul_angle + angular_vel*dt

            roll = eul_angle[0]   
            pitch = eul_angle[1]
            yaw = eul_angle[2]
            
            # get the rotation matrix from roll, pitch, yaw angle
            q.R = rotation_matrix(roll, pitch, yaw)

            
            # ------------ Translation dynamics update ------------------
            # equation (1) in week4
            acc = (np.matmul(q.R, np.array([0, 0, total_thrust_real.item()]).T) ) / q.m - np.array([0, 0,  g]).T
            
            # acceleration update
            x_acc = acc[0]
            y_acc = acc[1]
            z_acc = acc[2]
            # velocity update, basic physics!
            x_vel += x_acc * dt
            y_vel += y_acc * dt
            z_vel += z_acc * dt
            # position update
            x_pos += x_vel * dt
            y_pos += y_vel * dt
            z_pos += z_vel * dt
            
            # update pose for quadcopter and update the animation
            q.update_pose(x_pos, y_pos, z_pos, roll, pitch, yaw)

            t += dt

        t = 0
        i = (i + 1) % 3
        irun += 1
        
        if irun >= n_run:
            break
    end = time.time()

    print("Simulation Complete")
    print(end - start,"s")

def calculate_position(c, t):

    return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]


def calculate_velocity(c, t):

    return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]


def calculate_acceleration(c, t):

    return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]


def rotation_matrix(roll, pitch, yaw):
    """
    Calculates the ZYX rotation matrix.

    Args
        Roll: Angular position about the x-axis in radians.
        Pitch: Angular position about the y-axis in radians.
        Yaw: Angular position about the z-axis in radians.

    Returns
        3x3 rotation matrix as NumPy array
    """
    return np.array(
        [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)],
         [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
          sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
         [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw)]
         ])



def vec_cross(a,b):
    return np.array([[a[1]*b[2] - a[2]*b[2]], [-a[0]*b[2] + a[2]*b[0]], [a[0]*b[1] - a[1]*b[0]]]).reshape(3,1)


def vee_map(R):
    return np.array([R[2,1], R[0,2],R[1,0]])

def trace(R):
    return R[0,0]+R[1,1]+R[2,2]

def add_noise(x,value):
    x_noise = np.random.normal(0, value,x.shape)
    return x + x_noise

def main():

    quad_sim()


if __name__ == "__main__":
    main()
