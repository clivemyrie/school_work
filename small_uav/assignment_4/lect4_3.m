function lect4_3
%%

close all

less_idx = out.time_s < 1.0;
more_idx = out.time_s > 1.0;
figure;

subplot(2,1,1);
plot(out.time_s(less_idx),out.roll_deg(less_idx));
xlabel('time, s'); ylabel('roll, deg')
title('Problem 3, Part B (t<1)')
grid on

subplot(2,1,2);
plot(out.time_s(less_idx),out.q_dps(less_idx));
xlabel('time, s'); ylabel('roll rate, deg/s')
grid on



figure;
subplot(2,1,1);
plot(out.time_s(more_idx),out.roll_deg(more_idx));
xlabel('time, s'); ylabel('roll, deg')
title('Problem 3, Part B (t>1)')
grid on

subplot(2,1,2);
plot(out.time_s(more_idx),out.q_dps(more_idx));
xlabel('time, s'); ylabel('roll rate, deg/s')
grid on



end