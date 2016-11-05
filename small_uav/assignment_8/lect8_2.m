function lect8_2
clear;clc;close all;
format compact
load_uavsim

%Run simulation

%%

figure;hold on;
plot(out.time_s, out.pitch_deg, 'b.');
plot(out.time_s, out.pitch_cmd_deg, 'r.');
plot(out.time_s, out.de_deg, 'g.');
legend('pitch', 'pitch cmd', 'elevator')
xlabel('time, s'); ylabel('deg')
grid on
title('Problem 2, Part D - Pitch')

kp = P.pitch_kp
ki = P.pitch_ki
kd = P.pitch_kd
k_dc = P.K_theta_DC

ss_pitch = 20*P.K_theta_DC

end