function ec2
%%
clear; clc; close all


% total time = N*dt = 10s
N = 1000;
dt = 0.01;
measurementHz = 1.0;

theta0 = 0;
v0 = 3;
r0 = 10;
zc = 15;

x_hat0 = [0.1 3.2 9.1]';
P0 = diag([0.05^2 0.1^2 0.5^2]);
Q = diag([0.0^2 0.02^2 0.1^2]);
R = diag([0.1^2 0.01^2]);
x_hat = x_hat0;
P = P0;

x = zeros(3,N);
measurements = [0;0;0];
%x = [theta, v, r];
x(:,1) = [theta0, v0, r0];

% Generate truth
for counter = 2:N
    x_dot = [x(2, counter-1)/x(3, counter-1), 0, 3*cos(3*counter*dt)]' + Q*randn(3,1);
    x(:,counter) = x_dot*dt + x(:,counter-1);
end


for counter = 2:N
    % Propagate reference model to current time
    t = dt*counter;
    x_hat(:,counter) = x_hat(:,counter-1) + dt*f(x_hat(:, counter - 1), t);
    A = [0, 1/x_hat(3, counter), -x_hat(2, counter)/(x_hat(3, counter)^2);
        0,0,0;
        0,0,0];
    P = P + dt*(A*P + A*(P') + Q);
    P = real(.5*P + .5*(P'));
    
    % Correct estimates with measurements
    measurementReceived = abs(mod(counter*dt, measurementHz)) < 0.0001;
    if(measurementReceived)
        %disp(['Taking measurement at t = ' num2str(counter*dt)])
        C = [x_hat(3, counter)*sin(x_hat(1, counter)), 0, -cos(x_hat(1, counter));
            0, 2*x_hat(2, counter)/x_hat(3, counter), -(x_hat(2, counter)^2)/(x_hat(3, counter)^2)];
        
        L = P*(C')*inv(C*P*(C')+R);
        
        y = [zc - x(3, counter)*cos(x(1, counter));
            (x(2, counter)^2)/x(3, counter)];
        y = y + R*randn(2,1);
        
        h = [zc - x_hat(3, counter)*cos(x_hat(1, counter));
            (x_hat(2, counter)^2)/x_hat(3, counter)];
        
        measurements(:,end+1) = x(:,counter);
        x_hat(:, counter) = x_hat(:, counter) + L*(y-h);
        P = (eye(3) - L*C)*P;
        P = real(.5*P + .5*(P'));
    end
end


figure; hold on; grid on
plot(x(3,:).*sin(x(1,:)), zc - x(3,:).*cos(x(1,:)), 'b.')
plot(x_hat(3,:).*sin(x_hat(1,:)), zc - x_hat(3,:).*cos(x_hat(1,:)), 'ro')
plot(measurements(3,:).*sin(measurements(1,:)), zc - measurements(3,:).*cos(measurements(1,:)), 'gd')
plot(0, 15, 'kd')
%x_hat
%measurements
title('Extra Credit 2');
axis equal

t = linspace(0,dt*N, N);
figure;

subplot(2,3,1); hold on; grid on
plot(t, x(1,:))
plot(t, x_hat(1,:))
xlabel('T,s');ylabel('theta,s');
legend('truth', 'estimate')
title('theta')

subplot(2,3,2); hold on; grid on
plot(t, x(2,:))
plot(t, x_hat(2,:))
xlabel('T,s');ylabel('v,m/s');
legend('truth', 'estimate')
title('velocity')

subplot(2,3,3); hold on; grid on
plot(t, x(3,:))
plot(t, x_hat(3,:))
xlabel('T,s');ylabel('radius,m');
legend('truth', 'estimate')
title('radius')

subplot(2,3,4); hold on; grid on
plot(t, x(1,:) - x_hat(1,:))
xlabel('T,s');ylabel('theta,s');
title('theta error')

subplot(2,3,5); hold on; grid on
plot(t, x(2,:) - x_hat(2,:))
xlabel('T,s');ylabel('v,m/s');
title('velocity error')

subplot(2,3,6); hold on; grid on
plot(t, x(3,:) - x_hat(3,:))
xlabel('T,s');ylabel('radius,m');
title('radius error')


end

function ret = f(x_hat, t)
ret = [x_hat(2)/x_hat(3); 0; 3*cos(3*t)];
end