function [Ad, Bd, Bd1]= getMatrices(vehicle, Vx)


lf = vehicle.lf;
lr = vehicle.lr;
Jz = vehicle.Jz;
m = vehicle.m;
Cf = vehicle.Cf;
Cr = vehicle.Cr;
Csf = vehicle.Csf;
d = vehicle.d;

Ts = 0.05;
A = [
    0, 1, 0, 0;
    0, -(2*Cf+2*Cr)/m/Vx, (2*Cf+2*Cr)/m, -(2*Cf*lf-2*Cr*lr)/m/Vx;
    0, 0, 0, 1;
    0, -(2*Cf*lf-2*Cr*lr)/Jz/Vx, (2*Cf*lf-2*Cr*lr)/Jz, -(2*Cf*lf^2+2*Cr*lr^2)/Jz/Vx];
B = [0, 2*Cf/m, 0, 2*Cf*lf/Jz; 0, 0 , 0, 2*d*Csf/Jz]';
B1 = [0, -(2*Cf*lf-2*Cr*lr)/m/Vx-Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Jz/Vx]';

%%
a = B(2,1);
b = B1(2);
Kff1 = -b/a;

c = B(4,1);
d = B(4,2);
e = B1(4);
Mzff  = -(c*(-b/a)+e)/d;

%%
C = eye(4);
G = ss(A,[B B1],C,0);
sysd = c2d(G, Ts);
Ad = sysd.A;
Bd = sysd.B(:,1:2);
Bd1 = sysd.B(:,3);
