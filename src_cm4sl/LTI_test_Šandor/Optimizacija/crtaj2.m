
%%
clear

vx = 80/3.6;  % m/s
RunSimulation
%DLCoffset = round(vx*3.6/60*250,-1);  % [m] global X at which the maneuver starts
%sim('LTI_test2.slx')
t_sim = sigsOut{find(strcmp(sigsOut.getElementNames,'steering_angle'))}.Values.Time;  % controller time vector
save ddpc_nominal_80.mat

%%
save ddpc_nominal_60.mat

%
clear
vx = 80/3.6;  % m/s
RunSimulation
%DLCoffset = round(vx*3.6/60*250,-1);  % [m] global X at which the maneuver starts
%sim('LTI_test2.slx')
t_sim = sigsOut{find(strcmp(sigsOut.getElementNames,'steering_angle'))}.Values.Time;  % controller time vector
save ddpc_nominal_80_2.mat

%
clear
vx = 100/3.6;  % m/s
RunSimulation
%DLCoffset = round(vx*3.6/60*250,-1);  % [m] global X at which the maneuver starts
%sim('LTI_test2.slx')
t_sim = sigsOut{find(strcmp(sigsOut.getElementNames,'steering_angle'))}.Values.Time;  % controller time vector

save ddpc_nominal_100_2.mat
%%

%%
close all
%%
load ddpc_nominal_60
%%
load ddpc_nominal_60.mat%ddpc_nominal_60_2
%%
%close all
%figure
subplot(3,1,1)
hold on
%plot(250+logs.X.Data-vx*15, logs.Y_ref.Data,'--', 'HandleVisibility','off');
ylabel('Y [m]')
plot(250+logs.X.Data-vx*15, logs.Y.Data)
xlim([200 500])
xlabel('X [m]')
subplot(3,1,2)
hold on
%plot(250+logs.X.Data-vx*15, 180/pi*logs.Yaw_ref.Data,'--','HandleVisibility','off')
plot(250+logs.X.Data-vx*15, 180/pi*logs.yaw.Data)
ylabel('yaw [deg]')
xlabel('X [m]')
xlim([200 500])
subplot(3,1,3)
plot(250+logs.X.Data-vx*15, logs.steering_angle.Data*180/pi)
hold on; yline(-75,'--'); yline(75,'--'); ylabel('SWA [deg]')
xlabel('X [m]')
xlim([200 500])

%%

subplot(3,1,1)
xlabel('$X$ [m]', 'interpreter','latex')
ylabel('$Y$ [m]', 'interpreter','latex')
legend('Nominal MPC', 'DDPC')
subplot(3,1,2);
legend('Nominal MPC', 'DDPC')
xlabel('$X$ [m]', 'interpreter','latex')
ylabel('$\Psi [^\circ{}]$','interpreter','latex')
subplot(3,1,3);

xlabel('$X$ [m]', 'interpreter','latex')
ylabel('$\delta_{SW} [^\circ{}]$','interpreter','latex')