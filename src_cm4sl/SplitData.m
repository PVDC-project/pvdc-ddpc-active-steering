clear;clc;close all;
load simout.mat
t_data = simout.time;
vx_data = simout.signals.values(:,1);
vy_data = simout.signals.values(:,2);
yaw_rate_data = simout.signals.values(:,3);
swa_data = simout.signals.values(:,4);
vxs = [60 70 80] / 3.6;  % m/s
t_start = [15 35 55];
t_end = [30 50 70];
% vx_allowed_offset = 5 / 3.6;  % m/s
labels = {'vx (m/s)','y (m)','vy (m/s)','yaw (rad)','yaw rate (rad/s)','steering wheel angle (rad)','time (s)'};
trajectories = cell(length(vxs)+1,length(labels));
trajectories(1,:) = labels;
for i=2:length(vxs)+1
    vx = vxs(i-1);
%     idx = find(vx_data > (vx-vx_allowed_offset) & vx_data < (vx+vx_allowed_offset));
    idx = find(t_data > t_start(i-1) & t_data < t_end(i-1));
    if all(diff(idx)==1)
        trajectories{i,1} = vx;
        trajectories{i,2} = cumtrapz(t_data(idx), vy_data(idx));
        trajectories{i,3} = vy_data(idx);
        trajectories{i,4} = cumtrapz(t_data(idx), yaw_rate_data(idx));
        trajectories{i,5} = yaw_rate_data(idx);
        trajectories{i,6} = swa_data(idx);
        trajectories{i,7} = t_data(idx);
    end
end
for i=2:length(vxs)+1
    figure
    t_test = trajectories{i,end};
    ud = trajectories{i,end-1};
    yd = [trajectories{i,2:5}]';
    subplot(3,1,1); plot(t_test, yd(1,:)); ylabel('y [m]'); title(['$v_x$ = ',num2str(vxs(i-1)*3.6),' km/h'])
    subplot(3,1,2); plot(t_test, 180/pi * yd(3,:)); ylabel('$\theta$ [$^\circ$]')
    subplot(3,1,3); plot(t_test, 180/pi * ud); 
    ylabel('$\delta_{sw}$ [$^\circ$]'); xlabel('Time [s]')
end
save trajectories.mat trajectories


    

