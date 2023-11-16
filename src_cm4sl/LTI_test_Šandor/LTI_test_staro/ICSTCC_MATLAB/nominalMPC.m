function u_opt = nominalMPC(x0, preview, N, Vx)

persistent Vx_old Ad Bd Bd1
%UNTITLED4 Summary of this function goes here%   Detailed explanation goes here
if Vx < 5
    Vx = 5;
end

vehicle = LoadVehicleParameters();
[Ad, Bd, Bd1]= getMatrices(vehicle, Vx)


%% Constraints
max_slip = 0.05;
max_steering = 7.5; % degrees

%% Objective
Q = eye(4);
R = 1e-9;
P = Q;

%%
yalmip('clear')
%clear all

% Model data
nx = 4; % Number of states
nu = 1; % Number of inputs
Bd = Bd(:,1); % samo skretanje

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
constraints = [];
objective = 0;
x = x0;
for k = 1:N
    x = Ad*x + Bd*u{k} + Bd1*Vx*preview(k);
    objective = objective + x'*Q*x + u{k}'*R*u{k};
    constraints = [constraints, -[max_steering*pi/180; max_slip] <= u{k}<= [max_steering*pi/180; max_slip]];
end
x = Ad*x + Bd*u{k} + Bd1*Vx*preview(k);
objective = objective + x'*P*x;
sol = optimize(constraints,objective);
u_opt = value(u{1});
x1 = Ad*x0 + Bd*u_opt + Bd1*Vx*preview(1);

end