function lect7_2

clear all;clc; close all;

load_uavsim
% Run with trimmed conditions found in Problem 2
P.Va0 = 13;


% Get high fidelity TFs
[A B] = linearize_uavsim(P);

% Extract Lat and Lon matrices
kpd = 3;ku = 4;kv = 5;kw = 6; kphi = 7; ktheta = 8; kpsi = 9; kp = 10; kq = 11; kr = 12;
kde = 1; kda = 2; kdr = 3; kdt = 4;


A_lon = A([ku kw kq ktheta kpd], [ku kw kq ktheta kpd]);
B_lon = B([ku kw kq ktheta kpd], [kde kdt]);

epsilon = 0.00001;
assert(abs(A_lon(1,4) - -P.gravity*cos(P.theta0)) < epsilon);
M_q = P.rho * P.Va0 * P.S_wing * P.c * P.c * P.C_m_q /4/P.Jy;
assert(abs(A_lon(3,3) - M_q) < epsilon);
M_de = P.C_m_delta_e* P.rho * P.Va0 * P.Va0 * P.S_wing * P.c/2/P.Jy;
assert(abs(B_lon(3,1) - M_de) < epsilon);

s=tf('s');
H = inv(s*eye(5)-A_lon)*B_lon;
H = minreal(H);
H = zpk(H);


%% Problem 2, Part A - Compare TFs (poles and zeros)
models = compute_tf_models(P);

de_over_q_tf = minreal(H(3, 1))
G_de2q = zpk(models.G_de2q)


%% Problem 2, Part B - Compare step responses

figure
step(de_over_q_tf, G_de2q, 20)
xlabel('tims, s'); ylabel('q, pitch rate, deg/sec')
title('Problem 2, Part B')
grid on
legend('High Fidelity', 'Lower Fidelity')

figure
step(de_over_q_tf, G_de2q, 1)
xlabel('tims, s'); ylabel('q, pitch rate, deg/sec')
title('Problem 2, Part B')
grid on
legend('High Fidelity', 'Lower Fidelity')



end