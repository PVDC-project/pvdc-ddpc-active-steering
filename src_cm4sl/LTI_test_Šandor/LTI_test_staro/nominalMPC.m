function u_opt = nominalMPC(t, x0, preview, N, Vx)

persistent Ad Bd Bd1 controller

%UNTITLED4 Summary of this function goes here%   Detailed explanation goes here
if Vx < 5
    Vx = 5;
end

vehicle = LoadVehicleParameters(0);
[Ad, Bd, Bd1]= getMatrices(vehicle, Vx);
Bd=Bd/13;

%% Constraints
max_steering = 70; % degrees

%% Objective
Q = diag([50 0 150 0]);
R = 2;
P = Q;

%%
yalmip('clear')
%clear all

% Model data
nx = 4; % Number of states
nu = 1; % Number of inputs
Bd = Bd(:,1); % samo skretanje
% 
% u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
% constraints = [];
% objective = 0;
% x = x0;
% for k = 1:N
%     x = Ad*x + Bd*u{k} + Bd1*Vx*preview(k);
%     objective = objective + x'*Q*x + u{k}'*R*u{k};
%     constraints = [constraints, -[max_steering*pi/180] <= u{k}<= [max_steering*pi/180]];
% end
% x = Ad*x + Bd*u{k} + Bd1*Vx*preview(k);
% objective = objective + x'*P*x;
% sol = optimize(constraints,objective);
% u_opt = value(u{1});


if t == 0
    nu = 1;
    nx = 4;
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    x00 = sdpvar(4,1);
    preview0  = sdpvar(10,1);
    constraints = [];
    objective = 0;
    x = x00;
    for k = 1:N
        x = Ad*x + Bd*u{k} + Bd1*Vx*preview0(k);
        objective = objective + x'*Q*x + u{k}'*R*u{k};
        constraints = [constraints, -[max_steering*pi/180] <= u{k}<= [max_steering*pi/180]];
    end
    objective = objective + x'*Q*x;
    ops = sdpsettings('verbose',0);
    controller = optimizer(constraints,objective,ops,[x00;preview0],u{1});  
end
u_opt = controller{[x0;preview']};
%x1 = Ad*x0 + Bd*u_opt + Bd1*Vx*preview(1);

end