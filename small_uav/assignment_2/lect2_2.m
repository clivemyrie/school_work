function lect2_2

%% Problem 2 Part a
clear;clc;close all

A = [-3 2 0; -10 0 -15; 0 1 0];
B = [0;-5;0];
C = [0 0 1];
D = 0;

[Gplant_num, Gplant_denom] = ss2tf(A,B,C,D);
Gplant = tf(Gplant_num, Gplant_denom);
kp = -12;
ki = -15;
kd = -3;

Gcl = PI_rateFeedback_TF(Gplant, kp, ki, kd)

%% Problem 2 Part b
t = 0;
dt = 0.001;
tHistory = [];
xHistory = [];
yHistory = [];
uHistory = [];
x = [0;0;0];

while t<5
    theta_c = 1;
    theta = x(3);
    theta_dot = x(2);
    u = PI_rateFeedback_controller(theta_c, theta, theta_dot, t==0, dt);
    
    xdot = A*x + B*u;
    x = x + xdot*dt;
    y = C*x + D*u;
    t = t+dt;
    
    tHistory(end+1) = t;
    xHistory(:,end+1) = x';
    yHistory(:,end+1) = y;
    uHistory(end+1) = u;
    
end

plot(tHistory, yHistory, tHistory, step(Gcl, tHistory), 'r--');
xlabel('time, s'); ylabel('response, degrees');
legend('Problem 2b', 'Problem 2a'); grid on
title('Problem 2b - Step Response')

figure
plot(tHistory, uHistory);
xlabel('time, s'); ylabel('u');
title('Problem 2b - uHistory'); grid on

%% Problem 2 Part c
t = 0;
dt = 0.001;
tHistory = [];
xHistory = [];
yHistory = [];
uHistory = [];
x = [0;0;0];

while t<5
    theta_c = 6;
    theta = x(3);
    theta_dot = x(2);
    u = PI_rateFeedback_controller(theta_c, theta, theta_dot, t==0, dt);
    
    xdot = A*x + B*u;
    x = x + xdot*dt;
    y = C*x + D*u;
    t = t+dt;
    
    tHistory(end+1) = t;
    xHistory(:,end+1) = x';
    yHistory(:,end+1) = y;
    uHistory(end+1) = u;
    
end

figure
plot(tHistory, step(6*Gcl, tHistory));
xlabel('time, s'); ylabel('response, degrees');
grid on;
title('Problem 2c - Step Response')

figure
plot(tHistory, uHistory);
xlabel('time, s'); ylabel('u');
title('Problem 2c - uHistory'); grid on


end