function lect2_1
%%
clear; clc; close all
%% Problem 1 Part a
s = tf('s');

kp = 3;
ki = 4;
kd = 5;

a = 2;
b = 1;
Gcl_num = (kp*b)*s + (ki*b);
Gcl_denom = (1+kd*b)*(s^2) + (a+kp*b)*s + (ki*b);
Gcl = Gcl_num/Gcl_denom


%% Problem 1 Part b

Gplant = b/(s+a);

Gcl = PI_rateFeedback_TF(Gplant, kp, ki, kd)

%% Problem 1 Part c
figure; hold on
step(Gcl); title('Step Response - Problem 1c')

for kp = 0:1.1:5
    for ki = 0:1.1:10
        for kd = 0:1.1:10
            Gcl_tuned = PI_rateFeedback_TF(Gplant, kp, ki, kd);
            step_info_tuned = stepinfo(Gcl_tuned);
            if (step_info_tuned.PeakTime < 1.5) && (step_info_tuned.Overshoot<10) && (step_info_tuned.Overshoot>4)
                kp_tuned = kp;
                ki_tuned = ki;
                kd_tuned = kd;
            end
        end
    end
end

Gcl_tuned = PI_rateFeedback_TF(Gplant, kp_tuned, ki_tuned, kd_tuned);
step(Gcl_tuned)
step_info_tuned = stepinfo(Gcl_tuned)
kp_tuned
ki_tuned
kd_tuned
legend('Untuned response', 'Tuned response')

end