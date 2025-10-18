clear;
clc;

n_robots = 5;           
thr = 2;               

kx = 0.6 * ones(1, n_robots);
ky = 0.6 * ones(1, n_robots);

for i = 1:n_robots
    K{i} = [kx(i), 0;
             0, ky(i)];
end

for i = 1:n_robots
    eval(sprintf('K%d = K{%d};', i, i));
end

X0 = [  0, 102,  50, 160, 200;   
        0,  56, -20,  40, 100];  

Xd = [102,   2,  40, 180, 220;    
       56,   2,  60,  80, 120];   


Xd_dot = zeros(2, n_robots);  

for i = 1:n_robots
    eval(sprintf('X%dd = Xd(:,%d);', i, i));       % crea X1d, X2d, X3d, X4d, X5d
    eval(sprintf('X%dd_dot = Xd_dot(:,%d);', i, i)); % crea anche X1d_dot, ecc.
    eval(sprintf('X%02d = X0(:,%d);', i, i));       % opzionale: anche X01, X02, ecc.
end


obj1 = [60, 45];  
obj2 = [30, 25];