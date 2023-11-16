function controller_output = LTV_MPC(controller_input, x0)
global Q R
VehicleParameters; % load vehicle parameters

t = controller_input(1);  % simulation time
vx = controller_input(2);  % m/s
Ts = controller_input(3);  % s
L = controller_input(end-2);  % length of the prediction horizon
n = controller_input(end-1);  % assumed state dimension
collecting_data = controller_input(end);
curvature_preview = controller_input(4+(4)*n:end-4);
nx = 4;  % number of states

persistent LTVcontroller

vxRef = 80/3.6;
if(t == 0)    
    yalmip('clear')
    
    [Ad, Bd, Bd1] = GetMatrices(vxRef, Ts); % find matrices
    Bd = Bd;  % steering wheel angle as the input
    Cd = [1 0 0 0;
          0 0 1 0];
    %Cd = eye(4);
       
    % Objective
    % Q, R set globally
    P =  diag([Q(1,1) 1e-6 Q(2,2) 1e-6]);    
    
    % Constraints
    max_steering = 70/VEHICLE.STEERING_RATIO; % degrees
    
    % Model data
    nu = 2; % Number of inputs inside controller
    
    u = sdpvar(repmat(nu,1,L),ones(1,L));
    curvature_preview_sdpvar = sdpvar(1,L);
    x0var = sdpvar(nx,1);
    
    constraints = [];
    objective = 0;
    x = x0var;
    for k = 1:L
        objective = objective + (Cd*x)'*Q*(Cd*x) + u{k}'*R*VEHICLE.STEERING_RATIO^2*u{k};
        x = Ad*x + Bd*u{k}(1) + Bd1*(u{k}(2));
        constraints = [constraints, -max_steering*pi/180 <= u{k}(1)<= max_steering*pi/180];
        constraints = [constraints, u{k}(2) == curvature_preview_sdpvar(k)];
    end
    objective = objective + x'*P*x;
  
    ops = sdpsettings('verbose',0,'solver','mosek');
    parameters_in = {x0var, curvature_preview_sdpvar};
    solutions_out = u;
    
    LTVcontroller = optimizer(constraints, objective, ops, parameters_in, solutions_out);
end

% activate steering ?
steering_on = false;
if t > 15 && ~collecting_data; steering_on = true; end
if ~steering_on; controller_output = zeros(4,1); return; end

inputs = {x0', vxRef*curvature_preview};

tic;
[solutions, errorcode, ~, ~, ~, diagnostics] = LTVcontroller(inputs);
yalmip_time = toc;

if errorcode ~= 0
    disp(diagnostics)
end
uopt = solutions{1}(1)*VEHICLE.STEERING_RATIO;  % optimal inputs

controller_output = [uopt; yalmip_time; diagnostics.solvertime; errorcode];

end