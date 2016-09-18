function lect1
%% Problem 1 Part C
clear; clc; close all

A = [-3,2,0;
    -10,0,-15;
    0,1,0];
B = [0;-5;0];
C = [0,0,1];
D = 0;

G = ss(A,B,C,D);
G = minreal(G)

zpk_sys = zpk(G)

%% Problem 1 Part D
t = 0;
dt = 0.001;
tHistory = [];
xHistory = [];
yHistory = [];
uHistory = [];
u = 1; % Step  function
x = [0;0;0];
while t<5
    xdot = A*x + B*u;
    x = x + xdot*dt;
    y = C*x + D*u;
    t = t+dt;
    
    tHistory(end+1) = t;
    xHistory(:,end+1) = x';
    yHistory(:,end+1) = y;
    uHistory(end+1) = u;
    
end

plot(tHistory, yHistory, tHistory, step(G, tHistory), 'r--');
xlabel('time, s'); ylabel('response, degrees'); legend('y', 'step command'); grid on


%% Problem 2 Part C
s = tf('s');
kp = 6.25;
kr = 2;
num = kp;
denom = (s^2) + (kr+1)*s + kp;
G = num/denom

% Problem 2 Part D

step(G, 5); grid on

%% Problem 3 Part B
clc; close all
m = 2;
k = 12;
b = 1;

s = tf('s');
num = 1;
denom = m*(s^2) + b*s + k;
G = num/denom

figure;step(G, 25); title('Problem 3.b'); grid on
% Problem 3 Part C
t = linspace(0,25,1000);
u = 10*ones(size(t));

figure;lsim(G,u,t);
title('Problem 3.c');
ylim([0, 1.5]);
grid on

% Problem 3 Part D
figure; hold on;
kdc = 1/k;
wn = sqrt(k/m);

zeta = b/m/2/wn;
num = kdc*wn*wn;
denom = (s^2) + 2*zeta*wn*s + wn*wn;
G = num/denom;
lsim(G,u,t);

b2 = 0.7*2*wn*m;
zeta = b2/m/2/wn;
denom = (s^2) + 2*zeta*wn*s + wn*wn;
G = num/denom;
lsim(G,u,t);

b3 = 1*2*wn*m;
zeta = b3/m/2/wn;
denom = (s^2) + 2*zeta*wn*s + wn*wn;
G = num/denom;
lsim(G,u,t);

title('Problem 3.d')
legend('original', 'damping ratio = 0.7', 'damping ratio = 1')
xlim([min(t) max(t)]); ylim([0 1.5])
grid on


% Problem 3 Part E

A = [0, 1;
    -k/m,-b/m];
B = [0;1/m];
C = [1,0];
D = 0;

[num, denom] = ss2tf(A,B,C,D);

G = tf(num,denom)
G = ss(A,B,C,D)
figure;lsim(G,u,t);

title('Problem 3.e')
xlim([min(t) max(t)]); ylim([0 1.5])
grid on

%% Problem 4 Part A
phi_rad = 14*pi/180
theta_rad = -30*pi/180
psi_rad = 160*pi/180

R_ned2b = eulerToRotationMatrix(phi_rad, theta_rad, psi_rad)

% Problem 4 Part B
[phi_rad, theta_rad, psi_rad] = rotationMatrixToEuler(R_ned2b)

% Problem 4 Part C
v_rel_b = [20;-2;3]
[Va, alpha, beta] = makeVaAlphaBeta(v_rel_b)
beta_deg = rad2deg(beta)

% Problem 4 Part D
vg_ned = [-15; -12; 2];
[Vg, gamma_rad, course_rad] = makeVgGammaCourse(vg_ned);
Vg
gamma_deg = rad2deg(gamma_rad)
course_deg = rad2deg(course_rad)

% Problem 4 Part E
roll_deg = 12;
pitch_deg = 2;
yaw_deg = -140;

R_ned2b = eulerToRotationMatrix(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));

vg_ned = [-15 -12 2]';
vw_ned = [3 4 0]';

vg_b = R_ned2b*vg_ned;
vw_b = R_ned2b*vw_ned;

v_rel_b = vg_b - vw_b
[Va, alpha_rad, beta_rad] = makeVaAlphaBeta(v_rel_b);
alpha_deg = rad2deg(alpha_rad)
beta_deg = rad2deg(beta_rad)

end