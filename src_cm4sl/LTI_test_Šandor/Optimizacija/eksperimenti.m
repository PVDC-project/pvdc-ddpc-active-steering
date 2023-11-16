clear;clc;close all;

vx = 60/3.6;  % m/s

RunSimulation
save('nominal_60')
%pause
%%
clear;clc;close all;
vx = 80/3.6;  % m/s
RunSimulation
save('nominal_80')
%%
pause
%%
clear;clc;close all;
vx = 100/3.6;  % m/s
RunSimulation
save('nominal_100')

%%
 figure 
 hold on
 subplot(3,1,1); hold on
 subplot(3,1,2); hold on
 subplot(3,1,3); hold on
 
 clear; load('ddpc_nominal_60.mat')
 crtaj
 clear; load('nominal_60.mat')
 crtaj
 %%
 
 figure 
 hold on
 subplot(3,1,1); hold on
 subplot(3,1,2); hold on
 subplot(3,1,3); hold on
 clear; load('ddpc_nominal_80.mat')
 crtaj
 clear; load('nominal_80.mat')
 crtaj
 
