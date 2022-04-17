close all;
clear;
clc;
syms q1 q2 q3 q4 ax ay az

% read data: measurement of the accelerometer
acc = xlsread("HW4-2.xls");

g = -9.8;
error = @(q1, q2, q3, q4, ax, ay, az) [-2*g*(q2*q4-q1*q3)-ax;
                                       -2*g*(q1*q2+q3*q4)-ay;
                                       -2*g*(0.5-q2^2-q3^2)-az];
J = @(q1, q2, q3, q4) -2*g*[-q3 q4 -q1 q2;
                             q2 q1  q4 q3;
                             0 -2*q2 -2*q3 0];

iter = 500;
[R, C] = size(acc);
quaternion = zeros(R, 4);

%% gradient descent
for k = 1:R
    
    % initial value
    q = [1; 0; 0; 0];
    t = 1;
    
    count = 0;
    result_c = [];
    
    while count < iter
        
        e = error(q(1), q(2), q(3), q(4), acc(k, 1), acc(k, 2), acc(k, 3));
        J_q = J(q(1), q(2), q(3), q(4));
        step_size = BLS(q, t, acc(k, :), error, J);

        q = q - step_size * (J_q' * e) / norm(J_q' * e);
        
        result_c(end+1) = error(q(1), q(2), q(3), q(4), acc(k, 1), acc(k, 2), acc(k, 3))' *error(q(1), q(2), q(3), q(4), acc(k, 1), acc(k, 2), acc(k, 3));
        gradient_c = J(q(1), q(2), q(3), q(4))' * error(q(1), q(2), q(3), q(4), acc(k, 1), acc(k, 2), acc(k, 3));
        
        if norm(gradient_c) < 0.1
            count = count + 1;
            break;
        end
        
        count = count + 1;
    end
    
    % check the values of c
    if k == 55      % plot for k-th data
        figure()
        plot([1:1:count], result_c, ".-", "LineWidth", 1);
        xlabel("Iteration");
        ylabel("Cost Function: c = e'*e");
    end
    
    quaternion(k, :) = q';
end

%% verify error
result_error = ones(R, 3);

for k = 1:R
    result_error(k, :)= abs(error(quaternion(k,1), quaternion(k, 2), quaternion(k, 3), quaternion(k,4), acc(k, 1), acc(k, 2), acc(k,3))');
end

% plot error
figure()
subplot(3, 1, 1)
plot([1:1:100], result_error(:, 1), "*-");
xlabel("Data Number");
ylabel("Error of ax");
subplot(3, 1, 2)
plot([1:1:100], result_error(:, 2), "*-");
xlabel("Data Number");
ylabel("Error of ay");
subplot(3, 1, 3)
plot([1:1:100], result_error(:, 3), "*-");
xlabel("Data Number");
ylabel("Error of az");

%% optput the result q to file
q1 = quaternion(:, 1);
q2 = quaternion(:, 2);
q3 = quaternion(:, 3);
q4 = quaternion(:, 4);
T = table(q1, q2, q3, q4);
writetable(T, "HW4-2_solution.xlsx", 'sheet', 1);