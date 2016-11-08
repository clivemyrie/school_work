function lect9_2
%%
clc;close all; clear all;
load_uavsim
% Run simulation

%%

figure; hold on
plot(out.time_s, out.ax_accel_mps2)
plot(out.time_s, out.ay_accel_mps2)
plot(out.time_s, out.az_accel_mps2)
legend('ax accel', 'ay accel', 'az accel')
xlabel('time, s');ylabel('accelerometer measurements, m/s^2');
grid on
title('Problem 2, Part C')



end