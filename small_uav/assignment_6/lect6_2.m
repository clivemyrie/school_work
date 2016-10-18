function lect6_2
%% Problem 2
close all; clear; clc;

load_uavsim
% Run simulation

%%

P.pitch_kp = -0.8; % kp<0
P.pitch_ki = -0.00; % ki<=0
P.pitch_kd = -0.1; % kd<0

figure;
plot(out.time_s, out.pitch_deg)
xlabel('time, s'); ylabel('pitch, deg')
grid on
figure;
plot(out.time_s, out.de_deg)
xlabel('time, s'); ylabel('elevator deflection, deg')
grid on

end