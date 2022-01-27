function out = trajectory(u,P)

% input(1*1):time
% output(6*1):desired trajectory and desired yaw angle

t = u(end);
b1d = [1 0 0];
xd = [sin(t) cos(t) 1];

out = [xd b1d];
end
