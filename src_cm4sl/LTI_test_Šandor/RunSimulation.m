%% Simulation setup
clear;close all;clc;
mpc_variant = 2;  % 0-DDPC, 1-MPC, 2-IPC
global lambda_alpha lambda_slack
lambda_alpha = 0.2;
lambda_slack = 1e6;

vx = 80/3.6;
Tsim = 30;
Ts = 0.05;  % [s] controller sampling time
Ts2 = 1e-3;  % [s] simulation sampling time
T_test = 5;  % [s] duration of the test trajectory
N = T_test/Ts;  % number of samples in the test trajectory
L = 15;  % length of the prediction horizon
DLCoffset = round(vx*3.6/60*250,-1);  % [m] global X at which the maneuver starts
n = 4;  % assumed system dimension, for recording past measurements

nu = 2;  % number of inputs
ny = 2;  % number of outputs
collect_data = 1;  % collect data for the Hankel matrix before the maneuver

VehicleParameters  % load vehicle parameters
m = VEHICLE.MASS;
Iz = VEHICLE.INERTIA_Z;
lf = VEHICLE.LF;
lr = VEHICLE.LR;
Cf = VEHICLE.CORNERING_STIFF;
Cr = Cf;
[A,B] = ErrorStateMatrices(VEHICLE,vx);
C = [1 0 0 0;
     0 0 1 0];

Tsim = 30;

%% Collect data if required
if collect_data
    collecting_data = 1;
    logs = sim('LTI_test');
    sigsOut = logs.logsout;

    t_sim = sigsOut{1}.Values.Time;
    error_state = GetSimData(sigsOut,'error_state');
    steering_angle = GetSimData(sigsOut,'steering_angle');
    curvature = GetSimData(sigsOut,'curvature');

    t_start = 15; t_end = 30;
    idx = t_sim > t_start & t_sim < t_end;
    error_state_pe = error_state(:,idx);
    steering_angle_pe = steering_angle(:,idx);
    curvature_pe = curvature(:,idx);

    figure;yyaxis left;plot(error_state_pe(1,:)');title('Test error states');xlabel('k');ylabel('$e_1$')
    yyaxis right;plot(error_state_pe(2,:)');ylabel('$e_2$');
    figure;yyaxis left;plot(steering_angle_pe);ylabel('Steering angle');xlabel('k')
    yyaxis right;plot(curvature_pe);ylabel('Curvature');xlabel('k');title('Test control inputs')
    figure;plot(GetSimData(sigsOut,'X_global')',GetSimData(sigsOut,'Y_global')','o');
    xlabel('X [m]');ylabel('Y [m]');title('Test path')

    yd = error_state_pe;
    ud = [steering_angle_pe; curvature_pe];

    Hu = HankelMatrix(ud,L+n);
    if (rank(Hu) ~= nu*(L+n))
        warning('Input is not PE'); 
    end
    Hy = HankelMatrix(yd,L+n);
    H = [Hu;Hy];
    
    save trajectory.mat H yd ud
end

%% Black box LTI model identification
if (mpc_variant == 2)
    
    
    load trajectory.mat yd ud;
    %%
    Nlti = N;  % use the same trajectory length for identification
    Xfull = [yd(1,2:Nlti); (yd(1,2:Nlti)-yd(1,1:Nlti-1))/Ts;  yd(2,2:Nlti); (yd(2,2:Nlti)-yd(2,1:Nlti-1))/Ts];
    %%
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
    %M = (VVt'\WVt')' - to ne pomaže ništa!
%    M2 = (V'\W')' - ni to isto!
 
    %%
    
    Alti = M(1:Nx,1:Nx);
    Blti = M(1:Nx,Nx+1:end);
    Clti = M(Nx+1:end,1:Nx);
    
    save lti_model.mat Alti Blti Clti
    
  
    %{
    load trajectory.mat yd ud;
    Nlti = N;  % use the same trajectory length for identification
    Nx = 4;  % assumed system dimension

   % Extract input and output data for system identification
    U = ud(:, 2:Nlti-1)';
    Y = yd(:, 2:Nlti-1)';
 %   Create iddata objects for input and output data
    input_data = iddata(Y, U, Ts);
  %  Set the options for system identification
    opts = ssestOptions('Focus', 'simulation', 'EnforceStability', 1);
    %Perform system identification to get the black-box state-space model
    ss_model = ssest(input_data, Nx, 'Ts', Ts, opts);

%    Display the identified state-space model
    disp(ss_model);

    Alti = ss_model.A;
    Blti = ss_model.B;
    Clti = ss_model.C;

       
    save lti_model.mat Alti Blti Clti    
    
    disp('System model identified.')
    %}
end

%% Run the simulation
collecting_data = 0;
logs = sim('LTI_test');
%%
sigsOut = logs.logsout;

% %% Plotting
t_sim = sigsOut{find(strcmp(sigsOut.getElementNames,'steering_angle'))}.Values.Time;  % controller time vector
t_sim2 = sigsOut{find(strcmp(sigsOut.getElementNames,'yaw'))}.Values.Time;  % simulation time vector
% 
% % error state
% error_state = GetSimData(sigsOut,'error_state');
% figure; plot(t_sim,error_state([1 2],:)); xlabel('Time [s]'); ylabel('Error states'); legend({'e_1','e_2'})
% 
% % diagnostics
% yalmip_time = GetSimData(sigsOut,'yalmip_time');
% solver_time = GetSimData(sigsOut,'solver_time');
% status = GetSimData(sigsOut,'status');
% figure;
% subplot(3,1,1); plot(t_sim,yalmip_time); ylabel('YALMIP time [s]')
% subplot(3,1,2); plot(t_sim,solver_time); ylabel('Solver time [s]')
% subplot(3,1,3); plot(t_sim,status); ylabel('YALMIP status'); xlabel('Time [s]')
% 
% % Y and yaw tracking, steering wheel angle
%%
Y_ref = GetSimData(sigsOut,'Y_ref');  % DLC
Y_global = GetSimData(sigsOut,'Y_global');
yaw_ref = GetSimData(sigsOut,'yaw_ref');
yaw = GetSimData(sigsOut,'yaw');
steering_angle = GetSimData(sigsOut,'steering_angle');
% Y_des = GetSimData(sigsOut,'Y_des');  % Rajamani error2global

figure
subplot(3,1,1)
plot(t_sim2,Y_global); hold on; plot(t_sim,Y_ref,'--'); ylabel('Y [m]')
subplot(3,1,2)
plot(t_sim2,yaw); hold on; plot(t_sim,yaw_ref); ylabel('yaw [deg]')
subplot(3,1,3)
plot(t_sim,rad2deg(steering_angle)); hold on; yline(-70,'--'); yline(70,'--'); ylabel('SWA [deg]')
xlabel('Time [s]')
%%
% 
% % global XY position
% % X_global = GetSimData(sigsOut,'X_global');
% % Y_global = GetSimData(sigsOut,'Y_global');
% % figure; plot(X_global,Y_global); xlabel('X [m]'); ylabel('Y [m]'); axis equal