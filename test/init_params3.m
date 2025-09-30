clear;
clc;

kx1 = 0.6;
ky1 = 0.6;
K1 = [kx1,0;
    0,ky1];

kx2 = 0.6;
ky2 = 0.6;
K2 = [kx2,0;
    0,ky2];

kx3 = 0.6;
ky3 = 0.6;
K3 = [kx3,0;
    0,ky3];


x01 = 0;
y01 = 0;
X01 = [x01;y01];
x02 = 102;
y02 = 56;
X02 = [x02;y02];
x03 = 50;
y03 = -20;
X03 = [x03; y03];    

x1d = 102;
y1d = 56;
X1d = [x1d;y1d];
x2d = 2;
y2d = 2;
X2d = [x2d;y2d];
x3d = 80;
y3d = 10;
X3d = [x3d; y3d];      

x1d_dot = 0;
y1d_dot = 0;
X1d_dot = [x1d_dot; y1d_dot];
x2d_dot = 0;
y2d_dot = 0;
X2d_dot = [x2d_dot; y2d_dot];
x3d_dot = 0;
y3d_dot = 0;
X3d_dot = [x3d_dot; y3d_dot];


obj1 = [60, 30];    
obj2 = [20, 40];    
thr = 5;            


