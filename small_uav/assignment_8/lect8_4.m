function lect8_4
clear;clc;close all;
format compact
load_uavsim

%Run simulation

%% Problem 4, Part D

figure;hold on;
plot(out.time_s, out.airspeed_mps, 'b.');
plot(out.time_s, out.airspeed_cmd_mps, 'r.');
legend('airspeed', 'airspeed cmd')
xlabel('time, s'); ylabel('speed, m/s')
grid on
title('Problem 4, Part D - Airspeed')

figure;hold on;
plot(out.time_s, out.throttle, 'b.');
xlabel('time, s'); ylabel('throttle, [0-1]')
grid on
title('Problem 4, Part D - Throttle')

kp = P.airspeed_throttle_kp
ki = P.airspeed_throttle_ki
kd = P.airspeed_throttle_kd

%% Problem 4, Part E

figure;hold on;
plot(out.time_s, out.airspeed_mps, 'b.');
plot(out.time_s, out.airspeed_cmd_mps, 'r.');
legend('airspeed', 'airspeed cmd')
xlabel('time, s'); ylabel('speed, m/s')
grid on
title('Problem 4, Part E - Airspeed')

figure;hold on;
plot(out.time_s, out.throttle, 'b.');
xlabel('time, s'); ylabel('throttle, [0-1]')
grid on
title('Problem 4, Part E - Throttle')

figure;hold on;
plot(out.time_s, out.alt_m, 'b.');
plot(out.time_s, out.alt_cmd_m, 'r.');
legend('alt', 'alt cmd')
xlabel('time, s'); ylabel('height, m')
grid on
title('Problem 4, Part E - Altitude')
end