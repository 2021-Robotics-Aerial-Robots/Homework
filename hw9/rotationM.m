clc
R=eye(3);
omega=[0;0.1;1];
x=omega(1);
y=omega(2);
z=omega(3);
omega_hat=[0 -z y;...
           z 0 -x;...
           -y x 0];
delta_t=0.05;       

for i=1:20  %irun for 20 times     
    R_dot=R*omega_hat;
    R=R+R_dot*delta_t;   
    [U,S,V] = svd(R); %Find the SVD of R 
    R=U*V';            %Use U and V to reconsstruct the R belong to SO(3)
end