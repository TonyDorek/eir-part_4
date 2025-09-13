clear;
clc;

%Initialization params
kx = 0.6;
ky = 0.6;
K = [kx,0;
    0,ky];

x0 = 0;
y0 = 0;
X0 = [x0;y0];

xd = 102;
yd = 56;
Xd = [xd;yd];

xd_dot = 0;
yd_dot = 0;
Xd_dot = [xd_dot; yd_dot];

obj = [60,35];
thr = 5;