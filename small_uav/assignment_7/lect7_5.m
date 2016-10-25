function lect7_5
%%
clc; clear; close all

load_uavsim
P.Va0 = 13;

% Run simulation
%%
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

Gcl_roll_low = PI_rateFeedback_TF(models.G_da2phi,P.roll_kp,P.roll_ki,P.roll_kd);
Gcl_roll_high = PI_rateFeedback_TF( H(4,1) ,P.roll_kp,P.roll_ki,P.roll_kd);
figure
step(Gcl_roll_low, Gcl_roll_high, 2) % 2 seconds
grid on
xlabel('time, s');ylabel('response')
legend('Lower Fidelity', 'Higher Fidelity')
title('Problem 5, Part A - No Gusting')

figure
plot(out.time_s, out.roll_deg);
grid on; title('Problem 5, Part A - No Gusting')
xlabel('time, s');ylabel('roll, deg')

%% Run with gusting

figure
plot(out.time_s, out.roll_deg);
grid on; title('Problem 5, Part B - With Gusting')
xlabel('time, s');ylabel('roll, deg')


end