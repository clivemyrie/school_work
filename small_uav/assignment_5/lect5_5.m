function lect5_5
%%
clear all;clc; close all;

load_uavsim
% Run with trimmed conditions found in Problem 2
P.Va0 = 13;


%% Problem 5, Part A
[A B] = linearize_uavsim(P);

% Extract Lat and Lon matrices
kpd = 3;ku = 4;kv = 5;kw = 6; kphi = 7; ktheta = 8; kpsi = 9; kp = 10; kq = 11; kr = 12;
kde = 1; kda = 2; kdr = 3; kdt = 4;


A_lon = A([ku kw kq ktheta kpd], [ku kw kq ktheta kpd])
B_lon = B([ku kw kq ktheta kpd], [kde kdt])


A_lat = A([kv kp kr kphi kpsi], [kv kp kr kphi kpsi])
B_lat = B([kv kp kr kphi kpsi], [kda kdr])

%% Problem 5, Part B: Spot checks
epsilon = 0.00001;
udot_over_theta = A_lon(1,4)
assert(abs(A_lon(1,4) - -P.gravity*cos(P.theta0)) < epsilon)
M_q = P.rho * P.Va0 * P.S_wing * P.c * P.c * P.C_m_q /4/P.Jy
assert(abs(A_lon(3,3) - M_q) < epsilon)
M_de = P.C_m_delta_e* P.rho * P.Va0 * P.Va0 * P.S_wing * P.c/2/P.Jy
assert(abs(B_lon(3,1) - M_de) < epsilon)

%% Problem 5, Part C: de/q Transfer Function

s=tf('s');
H = inv(s*eye(5)-A_lon)*B_lon;
H = minreal(H);
H = zpk(H);
de_over_q_tf = H(3, 1)

%% Problem 5, Part D: dr/r Transfer Function

H = inv(s*eye(5)-A_lat)*B_lat;
H = minreal(H);
H = zpk(H);
dr_over_r_tf = H(3, 2)

%% Problem 6, Part A

load_uavsim;
% Run sim

%%
figure; hold on
plot( out.time_s, out.q_dps, 'b', ... % uavsim output
out.time_s, step(0.001*de_over_q_tf, out.time_s),'r:'); % linear step
grid on; xlabel('time, s'); ylabel('pitch rate, rad/sec'); legend('Non-linear', 'Linear'); grid on
title('Problem 6, Part A')


%% Problem 6, Part B: Estimate phugoid mode from plot

%% Problem 6, Part C: Give vehicle an initial pitch
P.theta0 = 25*pi/180;
disp('Vehicle starts off climbing but starts to level off. Appears to be in a phugoid mode')

%% Problem 7, Part A: Compare linearized and non-linearized response to 0.001 deg aileron command
load_uavsim
% run sim

%%
p_over_da_tf = H(2, 1);

figure; hold on
plot( out.time_s, out.p_dps, 'b'); % uavsim output
plot(out.time_s, step(0.001*p_over_da_tf, out.time_s),'r:'); % linear step
xlabel('time, s'); ylabel('roll rate, rad/sec');
legend('Non-linear', 'Linear');
grid on

title('Problem 7, Part A')
% s_info = stepinfo(0.001*p_over_da_tf, out.time_s);
% peak_time = s_info.PeakTime;

%% Problem 7, Part B: Compare roll rate linearized and non-linearized response to 1.0 deg aileron command
load_uavsim
% run sim

%%
p_over_da_tf = H(2, 1);

figure; hold on
plot( out.time_s, out.p_dps, 'b'); % uavsim output
plot(out.time_s, step(1.0*p_over_da_tf, out.time_s),'r:'); % linear step
xlabel('time, s'); ylabel('roll rate, rad/sec');
legend('Non-linear', 'Linear');
grid on

title('Problem 7, Part B')
% s_info = stepinfo(0.001*p_over_da_tf, out.time_s);
% peak_time = s_info.PeakTime;

%% Problem 7, Part C: Observe dutch roll mode
load_uavsim
P.v0 = 5;
% run sim

%%

figure
plot(out.time_s, out.roll_deg);
xlabel('time, s'); ylabel('roll , deg');
grid on
title('Problem 7, Part C')
xlim([0,2])
figure
plot(out.time_s, out.beta_deg);
xlabel('time, s'); ylabel('side slip, deg');
grid on
xlim([0,2])
title('Problem 7, Part C')

% s_info = stepinfo(0.001*p_over_da_tf, out.time_s);
% peak_time = s_info.PeakTime;





end