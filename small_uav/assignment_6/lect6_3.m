function lect6_3
%% Problem 3
close all; clc; clear;


load_uavsim
% Run simulation
%%

P.altitude_kp = 0.23; % kp>0
P.altitude_ki = 0.08; % ki>0

alt_cmd = zeros(size(out.time_s));
alt_cmd(mod(out.time_s, 20)<10) = 50;
alt_cmd(mod(out.time_s, 20)>=10) = 51;

figure; hold on
plot(out.time_s, out.alt_m)
plot(out.time_s, alt_cmd)
legend('alt','alt cmd')
xlabel('time, s'); ylabel('alt, m')
title('Problem 3 (Altitude)')
grid on
figure;
plot(out.time_s, out.pitch_deg)
xlabel('time, s'); ylabel('pitch, deg')
title('Problem 3 (Pitch)')
grid on

end