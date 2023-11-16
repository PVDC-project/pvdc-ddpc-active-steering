function controller_output = DDPC(controller_input)
t = controller_input(1);  % simulation time
vx = controller_input(2);  % m/s
Ts = controller_input(3);  % s
N = controller_input(end-3);  % number of samples in the test trajectory
L = controller_input(end-2);  % length of the prediction horizon
n = controller_input(end-1);  % assumed state dimension
collecting_data = controller_input(end);

persistent DDPController steering_on

nu = 2;  % number of inputs, steering wheel angle and curvature
ny = 2;  % number of outputs

n_past = reshape(controller_input(4:4+(ny+nu)*n-1),ny+nu,n);
y0 = n_past(1:ny,:);
u0 = n_past(ny+1:end,:);

curvature_preview = controller_input(4+(ny+nu)*n:end-4);
% yaw_rate_ref = vx * curvature_preview;

if (t == 0)
    yalmip('clear'); steering_on = false;
    
    % controller variables
    u = sdpvar(nu,L+n,'full');  % complete input sequence
    y = sdpvar(ny,L+n,'full');  % complete output sequence
    u0 = sdpvar(nu,n,'full');  % past n inputs (measurements)
    y0 = sdpvar(ny,n,'full');  % past n outputs (measurements)
    alpha = sdpvar(N-(L+n)+1,1);  % vector of size (N-L+1), H*alpha=[u;y]
    slack = sdpvar(ny*n,1);  % slack variable for constraint relaxation
    Hopt = sdpvar((nu+ny)*(L+n),N-(L+n)+1,'full');
    
    % objective function
    %Q = diag([5;10]);  % output weights
    %R = diag([0.05;0]);  % input weights
    
    Q = diag([50;150]);  % output weights
    R = diag([2;0]);  % input weights
    
    lambda_alpha = 5e-3;  % regularization term on alpha
    lambda_slack = 1e7;  % regularization term on the slack variable
    objective = 0;
    for k = n+1:n+L  % running cost, y'Qy + u'Ru, first nx elements are for the initial condition
        objective = objective + y(:,k)'*Q*y(:,k) + u(:,k)'*R*u(:,k);
    end
    objective = objective + lambda_alpha * norm(alpha)^2;  % regularization cost
    objective = objective + lambda_slack * norm(slack)^2;
    objective = objective + y(:,k)'*Q*y(:,k);
    % constraints
    u_max = deg2rad(70);  u_min = -u_max;  % steering wheel angle limits
    constraints = [];
    constraints = [constraints, Hopt*alpha == [u(:);y(:)]];  % the model of the data-driven MPC
%     constraints = [constraints, Hopt*alpha == [u(:);y(:)]+[zeros((n+L)*nu,1);slack;zeros(ny*L,1)]];  % "relaxed" model of the data-driven MPC
    constraints = [constraints, u(:,1:n)==u0(:,1:n), y(:,1:n)==y0(:,1:n)];  % initial state constraint (through n last measurements)
    constraints = [constraints, u_min <= u(1,n+1:L+n) <= u_max];  % input box constraints
    
    % define the controller
    parameters_in = {u0,y0,Hopt,u(2,n+1:L+n)};  % reference, current "state", PE trajectory, curvature preview
    solutions_out = {u,y,alpha};  % optimal input, output and "model"
    ops = sdpsettings('verbose',0,... % print output? (0-2)
                      'solver','mosek'); %,...
                      %'osqp.eps_abs',1e-6,...
                      %'osqp.eps_rel',1e-6);
    DDPController = optimizer(constraints,objective,ops,parameters_in,solutions_out);
    
    controller_output = zeros(4,1);
    return;
end

% DDPController = load('DDPC_saved.mat').DDPC;

% if t > 15 && ~steering_on && ~collecting_data; steering_on = true; end
% if ~steering_on; controller_output = zeros(4,1); return; end

% trajectories = load('trajectories').trajectories;
% vxs = [trajectories{2:end,1}];
% vx_diffs = abs(vxs - vx);
% idx_vx_closest = find(vx_diffs == min(vx_diffs), 1);
% ud = trajectories{idx_vx_closest+1,end-1}';
% start_idx = 1; %round(0.5*length(ud));
% ud = ud(:,start_idx:start_idx+N-1);
% yd = [trajectories{idx_vx_closest+1,2:5}]';
% yd = yd(:,start_idx:start_idx+N-1);
% 
% % create the Hankel matrices of order L+n, check for persistency of excitation
% Hu = HankelMatrix(ud,L+n);
% if (rank(Hu) ~= nu*(L+n))
%     warning('Input is not PE'); 
% end
% Hy = HankelMatrix(yd,L+n);
% H = [Hu;Hy];

trajectory = load('trajectory');
ud = trajectory.ud(:,1:N); 
yd = trajectory.yd(:,1:N);
Hu = HankelMatrix(ud,L+n);
Hy = HankelMatrix(yd,L+n);
H = [Hu;Hy];
% H = load('trajectory').H;

inputs = {u0,y0,H,curvature_preview};

tic;
[solutions, errorcode, ~, ~, ~, diagnostics] = DDPController(inputs);
% [solutions, errorcode, diagnostics] = DDPCf(inputs);
% diagnostics.solvertime = diagnostics.solvetime;
yalmip_time = toc;

% [yalmip_time diagnostics.solvertime]
if errorcode ~= 0
    disp(diagnostics)
%     warning(['Error code ',num2str(errorcode),': ',errortext{1}]);
end
U = solutions{1};  % optimal inputs
uopt = U(1,n+1);

controller_output = [uopt; yalmip_time; diagnostics.solvertime; errorcode];
end