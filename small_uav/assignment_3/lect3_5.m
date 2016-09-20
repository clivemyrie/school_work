function lect3_5
%%
close all; clc
% FL, FR, BL, BR, Avionics
x = [0.13;0.13;-0.13;-0.13;0.0];
y = [-0.22;0.22;-0.22;0.22;0];
z = [0;0;0;0;0];
m = [0.2;0.2;0.2;0.2;0.8];

%% Problem 5, Part A
Jx =  sum((y.^2+z.^2).*m);
Jy =  sum((x.^2+z.^2).*m);
Jz =  sum((x.^2+y.^2).*m);

Jxy = sum((x.*y).*m);
Jyz = sum((y.*z).*m);
Jxz = sum((x.*z).*m);

J = [Jx,-Jxy,-Jxz;
    -Jxy,Jy,-Jyz;
    -Jxz, -Jyz, Jz]

%% Problem 5, Part B
x = [x;0.05];
y = [y;0];
z = [z;0.1];
m = [m;0.3];

Jx =  sum((y.^2+z.^2).*m);
Jy =  sum((x.^2+z.^2).*m);
Jz =  sum((x.^2+y.^2).*m);

Jxy = sum((x.*y).*m);
Jyz = sum((y.*z).*m);
Jxz = sum((x.*z).*m);

J = [Jx,-Jxy,-Jxz;
    -Jxy,Jy,-Jyz;
    -Jxz, -Jyz, Jz]
end