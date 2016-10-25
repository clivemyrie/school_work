function lect7_4
%%
clc; close all; clear all

load_uavsim
P.Va0 = 13;

[A B] = linearize_uavsim(P);

% Extract Lat and Lon matrices
kpd = 3;ku = 4;kv = 5;kw = 6; kphi = 7; ktheta = 8; kpsi = 9; kp = 10; kq = 11; kr = 12;
kde = 1; kda = 2; kdr = 3; kdt = 4;


A_lon = A([ku kw kq ktheta kpd], [ku kw kq ktheta kpd]);
B_lon = B([ku kw kq ktheta kpd], [kde kdt]);


A_lat = A([kv kp kr kphi kpsi], [kv kp kr kphi kpsi]);
B_lat = B([kv kp kr kphi kpsi], [kda kdr]);

epsilon = 0.00001;
assert(abs(A_lon(1,4) - -P.gravity*cos(P.theta0)) < epsilon);
M_q = P.rho * P.Va0 * P.S_wing * P.c * P.c * P.C_m_q /4/P.Jy;
assert(abs(A_lon(3,3) - M_q) < epsilon);
M_de = P.C_m_delta_e* P.rho * P.Va0 * P.Va0 * P.S_wing * P.c/2/P.Jy;
assert(abs(B_lon(3,1) - M_de) < epsilon);

s=tf('s');
H = inv(s*eye(5)-A_lat)*B_lat;
H = minreal(H);
H = zpk(H);


models = compute_tf_models(P);

P = compute_autopilot_gains(models,P);
%% Problem 4, Part A (Gains)

kp = P.roll_kp
ki = P.roll_ki
kd = P.roll_kd

%% Problem 4, Part B (Step Response)
Gcl_roll_low =PI_rateFeedback_TF(models.G_da2phi,P.roll_kp,P.roll_ki,P.roll_kd);
Gcl_roll_high =PI_rateFeedback_TF( H(4,1) ,P.roll_kp,P.roll_ki,P.roll_kd);
step(Gcl_roll_low, Gcl_roll_high, 2) % 2 seconds
grid on
xlabel('time, s');ylabel('response')
legend('Lower Fidelity', 'Higher Fidelity')
title('Problem 4, Part B - Step response')


end