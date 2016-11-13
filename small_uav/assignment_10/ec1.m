function ec1
%%
clc; close all; clear

N = 100000;
x1 = 5*randn(1,N) + 30;
x2 = .3*x1 - 40;
x3 = .5*(x2-x1) +2*randn(1,N) + 20;
x4 = 4*randn(1,N) - 50;

X = [x1;x2;x3;x4];

mu = mean(X')
sigma = std(X')
sigma_squared = sigma.^2
P = cov(X')

figure; hold on; grid on; title('Extra Credit 1')
plot(x2, x4, '.')
plot(x3, x4, '.')
plot(x1, x4, '.')
plot(x2, x3, '.')
plot(x1, x3, '.')
plot(x1, x2, '.')



end