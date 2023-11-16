clear; clc;
load('ident_data1');

time = err_data.time;
X = err_data.signals.values;
U = in_data.signals.values;

Vx = 80;

Ts = 0.05;
lf = 1.311; % distance between the centre of gravity and the front axle
lr = 1.311;  % distance between the centre of gravity and the rear axle
m = 1599.98;   % mass
Jz = 2393.665;  % moment of inertia around z axis
%Cy = 72705; %N/rad, front wheel cornering stiffness
Cy = 27229.08;

sim('compare_sim');