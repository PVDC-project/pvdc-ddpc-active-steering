classdef Vehicle
    % VEHICLE
    %   Class which contains all relevant vehicle model parameters.
    
    properties
        m    % mass
        Jz   % moment of inertia around z axis
        lf   % distance between the centre of gravity and the front axle
        lr   % distance between the centre of gravity and the rear axle
        w    % half of the vehicles width
        Cf   % front wheel cornering stiffness
        Cr   % rear wheel cornering stiffness
        Csf   % longitudinal stiffness
        d    % slip to torque 
    end
    
    methods
        function obj = Vehicle(m, Jz, lf, lr, Cf, Cr,Csf, d)
           % initialize vehicle object
           obj.m = m;
           obj.Jz = Jz;
           obj.lf = lf;
           obj.lr = lr;
           obj.Cf = Cf;
           obj.Cr = Cr;
           obj.Csf = Csf;
           obj.d = d;
        end
        
    end
end

