clear; clc; 

load('ident_data1.mat');
nSample = length(in_data.signals.values);
X1 = err_data.signals.values;
U1 = [in_data.signals.values, 80/3.6*ones(nSample,1)];
load('ident_data2.mat');
nSample = length(in_data.signals.values);
X2 = err_data.signals.values;
U2 = [in_data.signals.values, 80/3.6*ones(nSample,1)];
load('ident_data3.mat');
nSample = length(in_data.signals.values);
X3 = err_data.signals.values;
U3 = [in_data.signals.values, 80/3.6*ones(nSample,1)];

%%
Tsample = 50e-3;
vehicle = LoadVehicleParameters();

% set estimation data
d = TrainData(X1, U1, X2, U2, X3, U3, Tsample);

% define identification object
FileName      = 'VehicleModelLTV';       % File describing the model structure.
Order         = [4 3 4];           % Model orders [ny nu nx].
Parameters    = vehicle.ReturnParams;   % Initial parameters.
InitialStates = [X1(1,:)', X2(1,:)', X3(1,:)'];            % Initial states.
Ts            = 0;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'Vehicle Dynamics');

% define estimation info
set(nlgr, 'InputName', {'Steering angle', 'Yaw rate desired','vx'},...
          'InputUnit', {'rad', 'rad/s','m/s'}, ...
          'OutputName', {'ey', 'dEy', 'eyaw', 'dEyaw'}, ...
          'OutputUnit', {'m', 'm/s', 'rad', 'rad/s'}, ...
          'TimeUnit', 's');
      
nlgr = setinit(nlgr, 'Name', {'ey', 'dEy', 'eyaw', 'dEyaw'});
nlgr = setinit(nlgr, 'Unit', {'m', 'm/s', 'rad', 'rad/s'});
nlgr = setpar(nlgr, 'Name', {'m' 'Jz' 'lf' 'lr' 'Cy'});
nlgr = setpar(nlgr, 'Unit', {'kg' 'kgm^2' 'm' 'm' 'N/rad'});

% fix some parameters
nlgr = setpar(nlgr, 'Fixed', {true, true, true, true, false});

% define parameter bounds
nlgr.Parameters(5).Minimum = 1e2;
nlgr.Parameters(5).Maximum = 1e6;

% set the absolute and relative error tolerances to small values 
nlgr.SimulationOptions.AbsTol = 1e-6;
nlgr.SimulationOptions.RelTol = 1e-5;

opt = nlgreyestOptions('Display', 'on', 'SearchMethod','auto');
%%
nlgr = nlgreyest(d, nlgr, opt);

compare(nlgr,d);

function data = TrainData(X1, U1, X2, U2, X3, U3, Tsample)

    data = iddata({X1,X2,X3}, {U1,U2,U3}, {Tsample,Tsample,Tsample}, 'Name', 'Vehicle Dynamics');
    data.InputName = {'Steering angle', 'Yaw rate desired','vx'};
    data.InputUnit = {'rad', 'rad/s','m/s'};
    data.OutputName = {'ey', 'dEy', 'eyaw', 'dEyaw'};
    data.OutputUnit = {'m', 'm/s', 'rad', 'rad/s'};
    data.Tstart = {0, 0, 0};
    data.TimeUnit = 's';
    
end