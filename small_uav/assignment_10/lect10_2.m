function lect10_2
%%
clc; close all; clear all;
load_uavsim
% Run simulation

%%
figure; hold on; grid on
plot(out.time_s, out.p_dps, 'b.')
plot(out.time_s, out.p_gyro_dps, 'g.')
plot(out.time_s, out.p_est_dps, 'ro')
xlabel('time, s'); ylabel('roll rate, deg/s')
legend('truth', 'gyro measurement', 'estimate')
title('Problem 2, Part A')

figure; hold on; grid on
plot(out.time_s, out.q_dps, 'b.')
plot(out.time_s, out.q_gyro_dps, 'g.')
plot(out.time_s, out.q_est_dps, 'ro')
xlabel('time, s'); ylabel('pitch rate, deg/s')
legend('truth', 'gyro measurement', 'estimate')
title('Problem 2, Part B')

figure; hold on; grid on
plot(out.time_s, out.r_dps, 'b.')
plot(out.time_s, out.r_gyro_dps, 'g.')
plot(out.time_s, out.r_est_dps, 'ro')
xlabel('time, s'); ylabel('yaw rate, deg/s')
legend('truth', 'gyro measurement', 'estimate')
title('Problem 2, Part C')

figure; hold on; grid on
plot(out.time_s, out.yaw_deg, 'b.')
plot(out.time_s, out.yaw_mag_deg, 'g.')
plot(out.time_s, out.yaw_est_deg, 'ro')
xlabel('time, s'); ylabel('roll rate, deg')
legend('truth', 'magnetometer measurement', 'estimate')
title('Problem 2, Part D')


end