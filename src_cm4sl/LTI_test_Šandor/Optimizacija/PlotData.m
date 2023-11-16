clear;clc;close all;

data60 = load('ddpc_nominal_60.mat');
data80 = load('ddpc_nominal_80.mat');
data100 = load('ddpc_nominal_100.mat');

%% test trajectory (80 km/h)
close all;
Ts = data80.Ts;
figure;yyaxis left;plot([0:Ts:15-2*Ts],data80.error_state_pe(1,:)');xlabel('$t$ [s]');ylabel('$e_1$ [m]')
yyaxis right;plot([0:Ts:15-2*Ts],180/pi*data80.error_state_pe(2,:)');ylabel('$e_2\ [^\circ]$');
x0=500;
y0=500;
width=550;
height=200;
set(gcf,'position',[x0,y0,width,height])

%% 80 km/h
close all;
t_sim = data80.t_sim;
t_start = 15;
t_end = 35;
idx60 = t_sim > t_start & t_sim < t_end;
t_sim = t_sim(idx60);

% error state
error_state80 = data80.error_state(:,idx60);
figure; plot(t_sim,error_state80); xlabel('$Time$ [s]'); ylabel('Error states'); legend({'e_1','e_2'})

% diagnostics
yalmip_time = data80.yalmip_time(idx60);
solver_time = data80.solver_time(idx60);
status = data80.status(idx60);
figure;
subplot(3,1,1); plot(t_sim,yalmip_time); ylabel('YALMIP time [s]')
subplot(3,1,2); plot(t_sim,solver_time); ylabel('Solver time [s]')
subplot(3,1,3); plot(t_sim,status); ylabel('YALMIP status'); xlabel('Time [s]')

% Y and yaw tracking, steering wheel angle
Y_ref = data80.Y_ref(idx60);
Y_global = data80.Y_global(idx60);
yaw_ref = data80.yaw_ref(idx60);
yaw = data80.yaw(idx60);
steering_angle = data80.steering_angle(idx60);

figure
subplot(3,1,1)
plot(t_sim,Y_global); hold on; plot(t_sim,Y_ref,'--'); ylabel('$Y$ [m]')
ylim([min(min(Y_global),min(Y_ref))-0.2, max(max(Y_global),max(Y_ref))+0.2])
subplot(3,1,2)
plot(t_sim,yaw); hold on; plot(t_sim,yaw_ref); ylabel('$\psi$ [$^\circ$]')
ylim([min(min(yaw),min(yaw_ref))-0.05, max(max(yaw),max(yaw_ref))+0.05])
subplot(3,1,3)
plot(t_sim,rad2deg(steering_angle)); hold on; yline(-45,'--'); yline(45,'--'); ylabel('$\delta_{SW}$ [$^\circ$]')
xlabel('$t$ [s]')

%% 60 km/h
close all;
t_sim = data60.t_sim;
t_start = 15;
t_end = 45;
idx60 = t_sim > t_start & t_sim < t_end;
t_sim = t_sim(idx60);

% error state
error_state80 = data60.error_state(:,idx60);
figure; plot(t_sim,error_state80); xlabel('Time [s]'); ylabel('Error states'); legend({'e_1','e_2'})

% diagnostics
yalmip_time = data60.yalmip_time(idx60);
solver_time = data60.solver_time(idx60);
status = data60.status(idx60);
figure;
subplot(3,1,1); plot(t_sim,yalmip_time); ylabel('YALMIP time [s]')
subplot(3,1,2); plot(t_sim,solver_time); ylabel('Solver time [s]')
subplot(3,1,3); plot(t_sim,status); ylabel('YALMIP status'); xlabel('Time [s]')

% Y and yaw tracking, steering wheel angle
Y_ref = data60.Y_ref(idx60);
Y_global = data60.Y_global(idx60);
yaw_ref = data60.yaw_ref(idx60);
yaw = data60.yaw(idx60);
steering_angle = data60.steering_angle(idx60);

figure
subplot(3,1,1)
plot(t_sim,Y_global); hold on; plot(t_sim,Y_ref,'--'); ylabel('Y [m]')
ylim([min(min(Y_global),min(Y_ref))-0.2, max(max(Y_global),max(Y_ref))+0.2])
subplot(3,1,2)
plot(t_sim,yaw); hold on; plot(t_sim,yaw_ref); ylabel('$\psi$ [$^\circ$]')
ylim([min(min(yaw),min(yaw_ref))-0.05, max(max(yaw),max(yaw_ref))+0.05])
subplot(3,1,3)
plot(t_sim,rad2deg(steering_angle)); hold on; yline(-45,'--'); yline(45,'--'); ylabel('$\delta_{SW}$ [$^\circ$]')
xlabel('$t$ [s]')

%% 100 km/h
close all;
t_sim = data100.t_sim;
t_start = 15;
t_end = 35;
idx60 = t_sim > t_start & t_sim < t_end;
t_sim = t_sim(idx60);

% error state
error_state80 = data100.error_state(:,idx60);
figure; plot(t_sim,error_state80); xlabel('Time [s]'); ylabel('Error states'); legend({'e_1','e_2'})

% diagnostics
yalmip_time = data100.yalmip_time(idx60);
solver_time = data100.solver_time(idx60);
status = data100.status(idx60);
figure;
subplot(3,1,1); plot(t_sim,yalmip_time); ylabel('YALMIP time [s]')
subplot(3,1,2); plot(t_sim,solver_time); ylabel('Solver time [s]')
subplot(3,1,3); plot(t_sim,status); ylabel('YALMIP status'); xlabel('Time [s]')

% Y and yaw tracking, steering wheel angle
Y_ref = data100.Y_ref(idx60);
Y_global = data100.Y_global(idx60);
yaw_ref = data100.yaw_ref(idx60);
yaw = data100.yaw(idx60);
steering_angle = data100.steering_angle(idx60);

figure
subplot(3,1,1)
plot(t_sim,Y_global); hold on; plot(t_sim,Y_ref,'--'); ylabel('Y [m]')
ylim([min(min(Y_global),min(Y_ref))-0.2, max(max(Y_global),max(Y_ref))+0.2])
subplot(3,1,2)
plot(t_sim,yaw); hold on; plot(t_sim,yaw_ref); ylabel('$\psi$ [$^\circ$]')
ylim([min(min(yaw),min(yaw_ref))-0.05, max(max(yaw),max(yaw_ref))+0.05])
subplot(3,1,3)
plot(t_sim,rad2deg(steering_angle)); hold on; yline(-45,'--'); yline(45,'--'); ylabel('$\delta_{SW}$ [$^\circ$]')
xlabel('$t$ [s]')

%% combined plot
close all;
% X limits for plotting
X_start = 200;
X_end = 500;
idx60 = data60.X_global > X_start & data60.X_global < X_end;
X_global60 = data60.X_global(idx60);
idx80 = data80.X_global > X_start & data80.X_global < X_end;
X_global80 = data80.X_global(idx80);
idx100 = data100.X_global > X_start & data100.X_global < X_end;
X_global100 = data100.X_global(idx100);

figure
subplot(3,1,1)
plot(X_global60,data60.Y_global(idx60)); hold on; ylabel('$Y$ [m]')
plot(X_global80,data80.Y_global(idx80)); plot(X_global100,data100.Y_global(idx100));
plot(X_global60,data60.Y_ref(idx60),'k--','LineWidth',0.5);
legend({'60 km/h','80 km/h','100 km/h','reference'})
ylim([-4 4])
% ylim([min(min(Y_global(idx)),min(Y_ref(idx)))-0.2, max(max(Y_global),max(Y_ref))+0.2])

subplot(3,1,2)
plot(X_global60,180/pi*data60.yaw(idx60)); hold on; ylabel('$\psi$ [$^\circ$]')
plot(X_global80,180/pi*data80.yaw(idx80)); plot(X_global100,180/pi*data100.yaw(idx100));
plot(X_global60,180/pi*data60.yaw_ref(idx60),'k--','LineWidth',0.5);
legend({'60 km/h','80 km/h','100 km/h','reference'})
% ylim([min(min(yaw),min(yaw_ref))-0.05, max(max(yaw),max(yaw_ref))+0.05])

subplot(3,1,3)
plot(X_global60,rad2deg(data60.steering_angle(idx60))); hold on; ylabel('$\delta_{SW}$ [$^\circ$]')
plot(X_global80,rad2deg(data80.steering_angle(idx80))); plot(X_global100,rad2deg(data100.steering_angle(idx100)));
yline(-70,'--'); yline(70,'--','HandleVisibility','off');
legend({'60 km/h','80 km/h','100 km/h','limits'})
xlabel('$X$ [m]')