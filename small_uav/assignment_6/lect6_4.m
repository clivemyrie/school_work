function lect6_4
%% 
%clc; clear all; close all
%load_uavsim
%P.wind_e = 5;
% Run simulation

%%
close all
figure;
plot(out.time_s, out.course_deg)
grid on
xlabel('time, s'); ylabel('course, deg')

figure;
plot(out.time_s, out.yaw_deg)
grid on
xlabel('time, s'); ylabel('yaw, deg')

figure;
plot(out.time_s, out.beta_deg)
grid on
xlabel('time, s'); ylabel('sideslip, deg')

figure;
crab_angle = out.course_deg - out.yaw_deg;
plot(out.time_s, crab_angle)
grid on
xlabel('time, s'); ylabel('crab angle, deg')

figure; hold on
plot(out.east_m, out.north_m, 'b.')
grid on
xlabel('East, m'); ylabel('North, m')
idx = 300;
time_s = out.time_s(idx)
crab_angle_deg = crab_angle(idx)

% Plot position, red *
pos_ned = [out.north_m(idx), out.east_m(idx), out.alt_m(idx)];
plot(pos_ned(2), pos_ned(1), 'r*');

% Plot wind vector, green -->
wind_ned = [out.wind_north_mps;out.wind_east_mps;out.wind_down_mps];
quiver(pos_ned(2), pos_ned(1), wind_ned(2)/3, wind_ned(1)/3, 'g')

% Plot body x vector, blue -->
yaw_rad = deg2rad(out.yaw_deg(idx));
body_x = [sin(yaw_rad) cos(yaw_rad)];
quiver(pos_ned(2), pos_ned(1), body_x(1), body_x(2), 'b')
axis equal

% Plot ground speed vector, red -->

course_rad = deg2rad(out.course_deg(idx));
ground_speed = [sin(course_rad) cos(course_rad)];
quiver(pos_ned(2), pos_ned(1), ground_speed(1), ground_speed(2), 'r')
axis equal

legend('Flight Path','Vehicle Position','Wind','Body','Ground Speed')
end