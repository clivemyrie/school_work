function lect6_1
%%
clear all; clc; close all

%% Problem 1, Part A
s = tf('s');
zeta = .7;
w = .8;
a = 5;
g_outer_by_hand = a*w*w / (s^3+2*zeta*w*(s^2) + w*w*s +a*w*w)
g_inner = w*w / (s^2+2*zeta*w*s + w*w);

g_outer = feedback(a*g_inner*(1/s), 1)

%% Problem 1, Part B
w = 50;
zeta = 0.7;
a =w/20;

g_outer = a*w*w/(s^3+2*zeta*w*(s^2) + w*w*s +a*w*w)
g_approx = a/(s+a)

figure
step( g_inner, g_outer, g_approx, 2);
title('Problem 1, Part B'); grid on;
legend('G_ inner','G_ outer','G_ approx');

%% Problem 1, Part C
a =w/10;

g_outer = a*w*w/(s^3+2*zeta*w*(s^2) + w*w*s +a*w*w)
g_approx = a/(s+a)

figure
step( g_inner, g_outer, g_approx, 2);
title('Problem 1, Part C, a = w/10'); grid on;
legend('G_ inner','G_ outer','G_ approx');

a =w/5;

g_outer = a*w*w/(s^3+2*zeta*w*(s^2) + w*w*s +a*w*w)
g_approx = a/(s+a)

figure
step( g_inner, g_outer, g_approx, 2);
title('Problem 1, Part C, a = w/5'); grid on;
legend('G_ inner','G_ outer','G_ approx');

a =w/2;

g_outer = a*w*w/(s^3+2*zeta*w*(s^2) + w*w*s +a*w*w)
g_approx = a/(s+a)

figure
step( g_inner, g_outer, g_approx, 2);
title('Problem 1, Part C, a = w/2'); grid on;
legend('G_ inner','G_ outer','G_ approx');

end