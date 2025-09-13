clear;
clc;

%Initialization params
kx = 3;
ky = 3;
K = [kx,0;
    0,ky];

x0 = 2;
y0 = 2;
X0 = [x0;y0];

xd = 4;
yd = 7;
Xd = [xd;yd];

xd_dot = 0;
yd_dot = 0;
Xd_dot = [xd_dot; yd_dot];

obj = [3.8,6.5];
thr = 0.1;