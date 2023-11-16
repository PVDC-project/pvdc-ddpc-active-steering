function J = cost(lambda_alpha0,lambda_slack0)


load steeering80.mat
global Jbest lambda_alpha_best lambda_slack_best

if isempty(Jbest)
    Jbest = Inf;
end
vx = 80/3.6;  % m/s
global lambda_alpha
global lambda_slack
lambda_alpha= lambda_alpha0;  % regularization term on alpha
lambda_slack=lambda_slack0;  % regularization term on the slack variable

%lambda_alpha = lambda_alpha0;
%lambda_slack = lambda_slack0;
%
RunSimulationDDPC;
steering_angle = GetSimData(sigsOut,'steering_angle');

%%
close all
figure
plot(steering_angle)
hold on
plot(steering_angle_nominal)
Y_global = GetSimData(sigsOut,'Y_global');
plot(Y_global)

%%
J = (steering_angle - steering_angle_nominal)*(steering_angle - steering_angle_nominal)';

%pause
end