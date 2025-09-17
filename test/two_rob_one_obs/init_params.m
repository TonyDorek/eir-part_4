clear;
clc;

%Initialization params
kx1 = 0.6;
ky1 = 0.6;
K1 = [kx1,0;
    0,ky1];

kx2 = 0.6;
ky2 = 0.6;
K2 = [kx2,0;
    0,ky2];


x01 = 0;
y01 = 0;
X01 = [x01;y01];
x02 = 102;
y02 = 56;
X02 = [x02;y02];

x1d = 102;
y1d = 56;
X1d = [x1d;y1d];
x2d = 2;
y2d = 2;
X2d = [x2d;y2d];

x1d_dot = 0;
y1d_dot = 0;
X1d_dot = [x1d_dot; y1d_dot];
x2d_dot = 0;
y2d_dot = 0;
X2d_dot = [x2d_dot; y2d_dot];

obj = [60,30];
thr = 5;
% mem = 0;