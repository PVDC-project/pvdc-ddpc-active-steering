function [A,B]= ErrorStateMatrices(VEHICLE,Vx)
% from Rajamani, eq. 2.45 (p. 35)
lf = VEHICLE.LF;
lr = VEHICLE.LR;
Jz = VEHICLE.INERTIA_Z;
m = VEHICLE.MASS;
Cf = VEHICLE.CORNERING_STIFF;
Cr = Cf;

A = [0, 1, 0, 0;
     0, -(2*Cf+2*Cr)/m/Vx, (2*Cf+2*Cr)/m, -(2*Cf*lf-2*Cr*lr)/m/Vx;
     0, 0, 0, 1;
     0, -(2*Cf*lf-2*Cr*lr)/Jz/Vx, (2*Cf*lf-2*Cr*lr)/Jz, -(2*Cf*lf^2+2*Cr*lr^2)/Jz/Vx];
Bswa = [0, 2*Cf/m, 0, 2*Cf*lf/Jz]';
Bcurv = Vx * [0, -(2*Cf*lf-2*Cr*lr)/m/Vx - Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Jz/Vx]';
B = [Bswa Bcurv];
% C = eye(4);

% G = ss(A,B,C,0);
% sysd = c2d(G, Ts);
% Ad = sysd.A;
% Bd = sysd.B;
end
