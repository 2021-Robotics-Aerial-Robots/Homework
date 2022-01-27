trajectory.m
input(1*1):time
output(6*1):desired trajectory and desired yaw angle

controller.m
input(25*1):desired trajectory and full state feedback, x v R Omega time
output(4*1): force and moment control input

quad_dynamics.m
input(4*1):force and moment control input
output(18*1):full state feedback, x v R Omega

hat.m
input(3*1):3*1 vector
output(3*3):skew-symmetric matrix of input vector

vee.m
input(3*3):skew-symmetric matrix
output(3*1):3*1 vector

param.m
declare all parameter of the quadrotor and initial condition