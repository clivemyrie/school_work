function lect3_7
%%
close all;clc

q = [0.661 -0.161 -0.508 0.529]';
mag_rad = 2*acos(q(1));
mag_deg = rad2deg(mag_rad)
v = q(2:4)/sin(mag_rad/2)

end