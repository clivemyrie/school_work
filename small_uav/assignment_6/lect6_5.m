function lect6_5
%%
clc; close all; clear all

load_uavsim


%% Problem 5, Part B
figure; hold on
plot(out.time_s, out.wind_north_mps)
plot(out.time_s, out.wind_east_mps)
plot(out.time_s, out.wind_down_mps)
title('Problem 5, Part B')
legend('north','east','down')
grid on

xlabel('time, s'); ylabel('wind speed, m/s')

%% Problem 5, Part C
figure;
plot(out.time_s, out.roll_deg)
grid on
xlabel('time, s'); ylabel('roll, deg')
title('Problem 5, Part C (roll)')

figure;
plot(out.time_s, out.beta_deg)
grid on
xlabel('time, s'); ylabel('sideslip, deg')
title('Problem 5, Part C (sideslip)')


%% Problem 5, Part D
figure;
plot(out.time_s, out.alt_m)
grid on
xlabel('time, s'); ylabel('alt, m')
title('Problem 5, Part D (Altitude)')

figure;
plot(out.time_s, out.de_deg)
grid on
xlabel('time, s'); ylabel('elevator deflection, deg')
title('Problem 5, Part D (elevator deflection)')

end