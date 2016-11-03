function lect8_1
clear;clc;close all;
load_uavsim

% Get higher fidelity TFs

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
H_lat = inv(s*eye(5)-A_lat)*B_lat;
H_lat = minreal(H_lat);
H_lat = zpk(H_lat);

H_lon= inv(s*eye(5)-A_lon)*B_lon;
H_lon= minreal(H_lon);
H_lon = zpk(H_lon);

% Lateral Channel
% Open Loop response from aileron to roll
H_lat(4,1); % Higher fidelity than models.G_da2phi
% Closed Loop response from roll command to roll
G_phic2phi = PI_rateFeedback_TF(H_lat(4,1),P.roll_kp,P.roll_ki,P.roll_kd);
% Closed Loop response from course command to course
G_chic2chi = PI_rateFeedback_TF(G_phic2phi*models.G_phi2chi, P.course_kp,P.course_ki,P.course_kd);



% Longitudinal Channel
% Open Loop response from elevator to pitch
H_lon(4,1); % Higher fidelity than models.G_de2theta
% Closed Loop response from pitch command to pitch
G_thetac2theta = PI_rateFeedback_TF(H_lon(4,1),P.pitch_kp,P.pitch_ki,P.pitch_kd);
% Closed Loop response from alt command to alt
G_altc2alt = PI_rateFeedback_TF(G_thetac2theta*models.G_theta2h, P.altitude_kp,P.altitude_ki,P.altitude_kd);
% Closed Loop response from airspeed commmand to airspeed using pitch
G_vac2va_pitch = PI_rateFeedback_TF(G_thetac2theta*models.G_theta2Va,P.airspeed_pitch_kp,P.airspeed_pitch_ki,P.airspeed_pitch_kd);
% Open Loop response from throttle to airspeed
models.G_dt2Va % Note: Can't use H(ku,kdt) because it is dominated by phugoid
% Closed Loop response from airspeed commmand to airspeed using throttle
G_vac2va_throttle = PI_rateFeedback_TF(models.G_dt2Va, P.airspeed_throttle_kp,P.airspeed_throttle_ki,P.airspeed_throttle_kd);
figure;
step(G_chic2chi)
grid on
xlabel('time, s');ylabel('step response')
title(' Step Response for chi\_cmd to chi')
sinfo = stepinfo(G_chic2chi);
chic2chi_overshoot = sinfo.Overshoot
chic2chi_risetime = sinfo.RiseTime
%Run simulation

%%

figure;hold on;
plot(out.time_s, out.course_deg, 'b.');
plot(out.time_s, out.course_cmd_deg, 'r.');
legend('course', 'course cmd')
xlabel('time, s'); ylabel('course, deg')
grid on
title('Problem 8, Part E - Course')


figure;hold on;
plot(out.time_s, out.roll_deg, 'b.');
plot(out.time_s, out.roll_cmd_deg, 'r.');
plot(out.time_s, out.da_deg, 'g.');
legend('roll', 'roll cmd', 'aileron')
xlabel('time, s'); ylabel('roll, deg')
grid on
title('Problem 8, Part E - Roll')



end