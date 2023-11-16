function [dx, y] = VehicleModelLTV(t,x,u_ext,m,Jz,lf,lr,Cy,varargin)

    Vx = u_ext(3);
    u = u_ext(1:2)';
    Cf = Cy;
    Cr = Cy;

    A = [
        0, 1, 0, 0;
        0, -(2*Cf+2*Cr)/m/Vx, (2*Cf+2*Cr)/m, -(2*Cf*lf-2*Cr*lr)/m/Vx;
        0, 0, 0, 1;
        0, -(2*Cf*lf-2*Cr*lr)/Jz/Vx, (2*Cf*lf-2*Cr*lr)/Jz, -(2*Cf*lf^2+2*Cr*lr^2)/Jz/Vx];
    B = [0, 2*Cf/m, 0, 2*Cf*lf/Jz]';
    B1 = [0, -(2*Cf*lf-2*Cr*lr)/m/Vx-Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Jz/Vx]';
    
    dx = A*x + [B/13,B1]*u;
    y = x;
end

