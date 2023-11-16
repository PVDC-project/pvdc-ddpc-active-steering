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
% Y and yaw tracking, steering wheel angle
Y_ref = GetSimData(sigsOut,'Y_ref');  % DLC
Y_global = GetSimData(sigsOut,'Y_global');
X_global = GetSimData(sigsOut,'X_global');
yaw_ref = GetSimData(sigsOut,'yaw_ref');
yaw = GetSimData(sigsOut,'yaw');
steering_angle = GetSimData(sigsOut,'steering_angle');
% Y_des = GetSimData(sigsOut,'Y_des');  % Rajamani error2global

% figure
%%
figure
subplot(3,1,1)
plot(t_sim2,Y_global); hold on; plot(t_sim,Y_ref,'--'); ylabel('Y [m]')
subplot(3,1,2)
%plot(t_sim,yaw); hold on; plot(t_sim,yaw_ref); ylabel('yaw [deg]')
subplot(3,1,3)
%plot(t_sim,rad2deg(steering_angle)); hold on; yline(-90,'--'); yline(90,'--'); ylabel('SWA [deg]')
xlabel('Time [s]')