function [Ad, Bd, Bd1]= GetMatrices(Vx, Ts)

    VehicleParameters; % run the script

    lf = VEHICLE.LF;
    lr = VEHICLE.LR;
    Jz = VEHICLE.INERTIA_Z;
    m = VEHICLE.MASS;
%     Cf = VEHICLE.CORNERING_STIFF;
%     Cr = VEHICLE.CORNERING_STIFF;
    Cf = 72705/2;   % default Carmaker tires: 117950
    Cr = 72705/2;   % default Carmaker tires: 143700    
    
    % e1, e1dot, e2, e2dot
    A = [0, 1, 0, 0;
         0, -(2*Cf+2*Cr)/m/Vx, (2*Cf+2*Cr)/m, -(2*Cf*lf-2*Cr*lr)/m/Vx;
         0, 0, 0, 1;
         0, -(2*Cf*lf-2*Cr*lr)/Jz/Vx, (2*Cf*lf-2*Cr*lr)/Jz, -(2*Cf*lf^2+2*Cr*lr^2)/Jz/Vx];
    B = [0, 2*Cf/m, 0, 2*Cf*lf/Jz]';  % steering angle
    B1 = [0, -(2*Cf*lf-2*Cr*lr)/m/Vx-Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Jz/Vx]';  % yaw rate reference

    C = eye(4);
    G = ss(A,[B B1],C,0);
    sysd = c2d(G, Ts);
    Ad = sysd.A;
    Bd = sysd.B(:,1);  % steering angle
    Bd1 = sysd.B(:,2);  % desired yaw rate
end
