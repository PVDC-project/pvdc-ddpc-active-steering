%% Simulation setup
clear;clc;close all;
cd 'C:\Projects_Josip\pvdc-ddpc-active-steering\src_cm4sl\'

% MPC cost function weights
global Q R
Q = diag([50 150]);  % error weights
R = diag([.5 0]);  % control input weights

Ts = 0.05;  % [s] sampling time
T_test = 5;  % [s] duration of the test trajectory
N = T_test/Ts;  % number of samples in the test trajectory
L = 12;  % length of the prediction horizon
n = 4;  % assumed system dimension, for recording past measurements

mpc_variant = 0;  % data-driven = 0; model-based = 1; identification-based = 2
ref_id = 0;  % DLC = 0; MLC = 1
DLCoffset = 250;  % [m] glob al X at which the maneuver starts

nu = 2;  % number of inputs
ny = 2;  % number of outputs
collect_data = 1;  % collect data for the Hankel matrix before the maneuver?
plot_test_trajectory = 0;  % plot the collected data

%% Collect data if required
if collect_data
    collecting_data = 1;
    disp('Collecting data...')
    sim('generic.mdl')
    
    t_sim = sigsOut{1}.Values.Time;
    error_state = GetSimData(sigsOut,'error_state');
    steering_angle = GetSimData(sigsOut,'steering_angle');
    curvature = GetSimData(sigsOut,'curvature');
    
    t_start = 15; t_end = 30;
    idx = t_sim > t_start & t_sim < t_end;
    t_idx = t_sim(idx); t_idx = t_idx - t_idx(1);
    error_state_pe = error_state(:,idx);
    steering_angle_pe = steering_angle(:,idx);
    curvature_pe = curvature(:,idx);
    
    if plot_test_trajectory
        figure;yyaxis left;plot(t_idx,error_state_pe(1,:)');title('Test error states');xlabel('Time [s]');ylabel('$e_1\ $[m]')
        yyaxis right;plot(t_idx,180/pi*error_state_pe(2,:)');ylabel('$e_2\ [^\circ]$');
        figure;yyaxis left;plot(t_idx,180/pi*steering_angle_pe);ylabel('Steering angle $[^\circ]$');xlabel('Time [s]')
        yyaxis right;plot(t_idx,curvature_pe);ylabel('Curvature [1/m]');xlabel('Time [s]');title('Test control inputs')
        figure;plot(GetSimData(sigsOut,'X_global')',GetSimData(sigsOut,'Y_global')','o');
        xlabel('X [m]');ylabel('Y [m]');title('Test path')
    end
    
    yd = error_state_pe;
    ud = [steering_angle_pe; curvature_pe];
    
    Hu = HankelMatrix(ud,L+n);
    if (rank(Hu) ~= nu*(L+n))
        warning('Input is not PE');
    end
    Hy = HankelMatrix(yd,L+n);
    H = [Hu;Hy];
    
    save trajectory.mat H yd ud
    
    disp('Data collection done.')
else
    disp('Skipping data collection.')
end

%% Black box LTI model identification
if (mpc_variant == 2)
    load trajectory.mat yd ud;
    
    Nlti = N;  % use the same trajectory length for identification
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
    Clti = M(Nx+1:end,1:Nx);
    
    save lti_model.mat Alti Blti Clti
end

%% Run the simulation
cmguicmd('StopSim');  % stop Carmaker☺
collecting_data = 0;
disp('Starting the simulation...')
sim('generic.mdl')
disp('Simulation done.')

%% Plotting
t_sim = sigsOut{1}.Values.Time;

% error state
error_state = GetSimData(sigsOut,'error_state');
figure; plot(t_sim,error_state); xlabel('Time [s]'); ylabel('Error states'); legend({'e_1','e_2'})

% diagnostics
yalmip_time = GetSimData(sigsOut,'yalmip_time');
solver_time = GetSimData(sigsOut,'solver_time');
status = GetSimData(sigsOut,'status');
figure;
subplot(3,1,1); plot(t_sim,yalmip_time); ylabel('YALMIP time [s]')
subplot(3,1,2); plot(t_sim,solver_time); ylabel('Solver time [s]')
subplot(3,1,3); plot(t_sim,status); ylabel('YALMIP status'); xlabel('Time [s]')

% global XY position
X_global = GetSimData(sigsOut,'X_global');
Y_global = GetSimData(sigsOut,'Y_global');
figure; plot(X_global,Y_global); xlabel('X [m]'); ylabel('Y [m]');

% Y and yaw tracking, steering wheel angle
Y_ref = GetSimData(sigsOut,'Y_ref');
Y_global = GetSimData(sigsOut,'Y_global');
yaw_ref = GetSimData(sigsOut,'yaw_ref');
yaw = GetSimData(sigsOut,'yaw');
steering_angle = GetSimData(sigsOut,'steering_angle');

figure
subplot(3,1,1)
plot(t_sim,Y_global); hold on; plot(t_sim,Y_ref,'--'); ylabel('Y [m]')
subplot(3,1,2)
plot(t_sim,yaw); hold on; plot(t_sim,yaw_ref); ylabel('yaw [deg]')
subplot(3,1,3)
plot(t_sim,rad2deg(steering_angle)); hold on; yline(-70,'--'); yline(70,'--'); ylabel('SWA [deg]')
xlabel('Time [s]')

% test trajectory
% figure;yyaxis left;plot([0:Ts:15-2*Ts],error_state_pe(1,:)');xlabel('$t$ [s]');ylabel('$e_1$ [m]')
% yyaxis right;plot([0:Ts:15-2*Ts],180/pi*error_state_pe(2,:)');ylabel('$e_2\ [^\circ]$');
% x0=500;
% y0=500;
% width=550;
% height=200;
% set(gcf,'position',[x0,y0,width,height])

%% References
% Berberich et al.: Data-Driven Model Predictive Control with Stability and Robustness Guarantees
% https://sci-hub.st/10.1109/tac.2020.3000182
% Berberich et al.: Data-Driven Tracking MPC for Changing Setpoints
% https://sci-hub.st/10.1016/j.ifacol.2020.12.389
% Raff et al: Nonlinear MPC of a Four Tank System
% https://sci-hub.st/10.1109/cca.2006.285898
% Coulson et al.: Data-Enabled Predictive Control: In the Shallows of the DeePC
% https://sci-hub.st/10.23919/ecc.2019.8795639
% Data-enabled predictive control for quadcopters
% https://onlinelibrary.wiley.com/doi/epdf/10.1002/rnc.5686
% Linear Tracking MPC for Nonlinear Systems—Part II: The Data-Driven Case
% https://ieeexplore.ieee.org/document/9756053