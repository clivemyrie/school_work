function lect4_2
%%
clc; clear all

gammaStar = -4.980745*pi/180;
load_uavsim

star = 0.5 * P.rho * P.Va0 * P.Va0 * P.S_wing;
a = P.C_L_0;
b = P.C_L_alpha;
c = P.C_L_delta_e;
d = -P.mass*P.gravity*cos(gammaStar);
e = P.C_D_0;
f = P.C_D_alpha;
g = P.C_D_delta_e;
h = P.mass*P.gravity*sin(gammaStar);
A = [b,c;f,g];
B = [-d/star-a;-h/star-e];
alpha_delta = inv(A) * B;
alphaStar = alpha_delta(1);
delta_eStar = alpha_delta(2);

alphaStar_deg = rad2deg(alphaStar)
delta_eStar_deg = rad2deg(delta_eStar)

P.theta0=gammaStar+alphaStar; P.delta_e0=delta_eStar;
P.u0=P.Va0*cos(alphaStar); P.w0=P.Va0*sin(alphaStar);
end