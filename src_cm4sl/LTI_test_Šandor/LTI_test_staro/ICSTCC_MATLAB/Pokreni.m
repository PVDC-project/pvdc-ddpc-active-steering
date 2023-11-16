v0 = 80/3.6;
Ts = 50e-3;
md = getCurvature(v0,0:0.1:15);


vehicle = LoadVehicleParameters();
% ovo koristi simulaicijski model
lf = vehicle.lf;
lr = vehicle.lr;
Iz = vehicle.Jz;
m = vehicle.m;
Cf = vehicle.Cf;
Cr = vehicle.Cr;
d = vehicle.d;
tau = 0.02
v0 = 22
md = getCurvature(v0,0:0.1:15)
Csf = 0
Ts  = 0.1
N = 10
dlc = 1
out_dlc = sim('LPV_MPC_1.slx');
dlc = 0
out_ss = sim('LPV_MPC_1.slx');


