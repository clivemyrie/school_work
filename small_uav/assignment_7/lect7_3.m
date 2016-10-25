function lect7_3

clear all;clc; close all;

load_uavsim
% Run with trimmed conditions found in Problem 2
P.Va0 = 13;


%% Get high fidelity TFs
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


%% Problem 3, Part A - Compare TFs (poles and zeros)
models = compute_tf_models(P);

da_over_p_tf = H(2, 1)
G_da2p = zpk(models.G_da2p)

%% Problem 3, Part B - Compare step responses

figure
step(da_over_p_tf, G_da2p, 10)
xlabel('tims, s'); ylabel('response')
title('Problem 3, Part B')
grid on
legend('High Fidelity', 'Lower Fidelity')

figure
step(da_over_p_tf, G_da2p, 1)
xlabel('tims, s'); ylabel('response')
title('Problem 3, Part B')
grid on
legend('High Fidelity', 'Lower Fidelity')



end