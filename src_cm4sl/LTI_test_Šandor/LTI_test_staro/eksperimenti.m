clear;clc;close all;

vx = 60/3.6;  % m/s

RunSimulation
% save('nominal_60')
crtaj
%%
clear;clc;close all;
vx = 80/3.6;  % m/s
RunSimulation
crtaj
%%

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
 clear; load('ddpc_nominal_60.mat')
 crtaj
 clear; load('nominal_60.mat')
 crtaj
 %%
 
 
 crtaj
 clear; load('ddpc_nominal_100.mat')
 crtaj
 
