function controller_output = LTI_MPC(controller_input, x0)
VehicleParameters; % load vehicle parameters
nx = 4;  % number of states
nu = 2;
ny = 2;

t = controller_input(1);  % simulation time
vx = controller_input(2);  % m/s
L = controller_input(end-2);  % length of the prediction horizon
n = controller_input(end-1);  % assumed state dimension
curvature_preview = controller_input(4+(ny+nu)*n:end-4);
collecting_data = controller_input(end);

%     n_past = reshape(controller_input(4:4+(ny+nu)*n-1),ny+nu,n);
%     y0 = [n_past(1:ny,1);n_past(1:ny,2)]; 

persistent LTIcontroller
vxRef = 80/3.6;

if(t == 0)
    yalmip('clear')

    load lti_model Alti Blti Clti;
    Ad = Alti;
    Bd = Blti(:,1)/VEHICLE.STEERING_RATIO;
    Bd1 = Blti(:,2)/vxRef;
    Cd = Clti;

    % Constraints
    max_steering = 70/VEHICLE.STEERING_RATIO; % degrees

    % Objective
    Q = diag([50 150]);
    R = diag([0.5 0]);
    P =  diag([Q(1,1) 1e-6 Q(2,2) 1e-6]);    

    % Model data
    nu = 2;  % Number of inputs inside controller

    u = sdpvar(repmat(nu,1,L),ones(1,L));
    curvature_preview_sdpvar = sdpvar(1,L);
    x0var = sdpvar(nx,1);

    constraints = [];
    objective = 0;
    x = x0var;
    for k = 1:L
        objective = objective + (Cd*x)'*Q*(Cd*x) + u{k}'*R*VEHICLE.STEERING_RATIO^2*u{k};
        x = Ad*x + Bd*u{k}(1) + Bd1*u{k}(2);
        constraints = [constraints, -max_steering*pi/180 <= u{k}(1)<= max_steering*pi/180];
        constraints = [constraints, u{k}(2) == curvature_preview_sdpvar(k)];
    end
    objective = objective + x'*P*x;

    ops = sdpsettings('verbose',0, 'solver','mosek');
    parameters_in = {x0var, curvature_preview_sdpvar};
    solutions_out = u;

    LTIcontroller = optimizer(constraints, objective, ops, parameters_in, solutions_out);   
end

% activate steering ?
steering_on = false;
if t > 15 && ~collecting_data; steering_on = true; end
if ~steering_on; controller_output = zeros(4,1); return; end

%     inputs = {y0, curvature_preview};
inputs = {x0', vxRef*curvature_preview};

tic;
[solutions, errorcode, ~, ~, ~, diagnostics] = LTIcontroller(inputs);
yalmip_time = toc;

if errorcode ~= 0
    disp(diagnostics)
end
uopt = solutions{1}(1)*VEHICLE.STEERING_RATIO;  % optimal steering wheel angle

controller_output = [uopt; yalmip_time; diagnostics.solvertime; errorcode];

end