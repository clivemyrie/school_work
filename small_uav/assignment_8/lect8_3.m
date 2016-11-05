function lect8_3
clear;clc;close all;
format compact
load_uavsim

%Run simulation

%%

figure;hold on;
plot(out.time_s, out.alt_m, 'b.');
plot(out.time_s, out.alt_cmd_m, 'r.');
legend('alt', 'alt cmd')
xlabel('time, s'); ylabel('alt, m')
grid on
title('Problem 3, Part D - Altitude')

figure;hold on;
plot(out.time_s, out.pitch_deg, 'b.');
plot(out.time_s, out.pitch_cmd_deg, 'r.');
legend('pitch', 'pitch cmd')
xlabel('time, s'); ylabel('pitch, deg')
grid on
title('Problem 3, Part D - Pitch')

figure;hold on;
plot(out.time_s, out.de_deg, 'b.');
xlabel('time, s'); ylabel('elevator deflection, deg')
grid on
title('Problem 3, Part D - Elevator Deflection')

kp = P.pitch_kp
ki = P.pitch_ki
kd = P.pitch_kd
k_dc = P.K_theta_DC

ss_pitch = 20*P.K_theta_DC

end