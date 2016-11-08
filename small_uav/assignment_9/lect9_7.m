function lect9_7
%%
clc;close all; clear all;
load_uavsim
% Run simulation

%%
% Waypoints
wp_east = [0 400 500 500 0]; %m
wp_north = [100 200 300 500 600]; %m
wp_alt = [50 60 90 90 60]; %m
wp_speed = [13 13 13 16 16]; %m/s

figure; hold on
plot(wp_east,wp_north,'r.--',out.east_m,out.north_m); axis equal
legend('Waypoint', 'Truth')
xlabel('east, m');ylabel('north, m');
grid on;
title('Problem 7, Part A')

figure; hold on
plot(out.time_s, out.alt_m)
plot(out.time_s, out.alt_cmd_m)
legend('altitude', 'altitude command')
xlabel('time, s');ylabel('Altitude, m');
grid on;
title('Problem 7, Part B')

figure; hold on
plot(out.time_s, out.airspeed_mps)
plot(out.time_s, out.airspeed_cmd_mps)
legend('airspeed', 'airspeed command')
xlabel('time, s');ylabel('Airspeed, m/s');
grid on;
title('Problem 7, Part C')

figure; hold on
plot(out.time_s, out.course_deg)
plot(out.time_s, out.course_cmd_deg)
legend('course', 'course command')
xlabel('time, s');ylabel('Course, deg');
grid on;
title('Problem 7, Part D')

end