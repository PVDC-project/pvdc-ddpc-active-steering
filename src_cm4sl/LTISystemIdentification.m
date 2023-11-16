clear;clc;

load trajectory.mat yd ud;
% yd - lateral error, yaw error (e1, e2)
% ud - steering wheel angle, road curvature (kappa)
    
Nlti = 100;  % use the same trajectory length for DDPC and system identification
Xfull = [yd(1,2:Nlti); (yd(1,2:Nlti)-yd(1,1:Nlti-1))/Ts;  yd(2,2:Nlti); (yd(2,2:Nlti)-yd(2,1:Nlti-1))/Ts];
U = ud(:,2:Nlti-1);
Y = yd(:,2:Nlti-1);
X = Xfull(:,1:end-1);
Xnext = Xfull(:,2:end);

Nx = 4;  % assumed system dimension
W = [Xnext; Y];
V = [X; U];
VVt = V*V';
WVt = W*V';
M = WVt * pinv(VVt); % identified matrix [A B; C 0]
Alti = M(1:Nx,1:Nx);
Blti = M(1:Nx,Nx+1:end);
Bdlti = Blti(:,1);  % steering wheel angle
Bd1lti = Blti(:,2);  % road curvature
Clti = M(Nx+1:end,1:Nx);

% compare to the physical model, some conversions needed
vxRef = 80/3.6;
Ts = 0.05;
[Ad, Bd, Bd1] = GetMatrices(vxRef, Ts);