function lect5_6
%%
clc; close all;clear all;
load_uavsim;

%%
figure; hold on
plot( out.time_s, out.q_dps, 'b', ... % uavsim output
out.time_s, step(0.001*H(kq,kde), out.time_s),'r:'); % linear step
grid on; xlabel('time, s'); ylabel('pitch rate, rad/sec'); legend(['Non-linear', 'Linear']); grid on


end