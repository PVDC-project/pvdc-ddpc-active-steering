classdef Vehicle
    % VEHICLE
    %   Class which contains all relevant vehicle model parameters.
    
    properties
        m    % mass
        Jz   % moment of inertia around z axis
        lf   % distance between the centre of gravity and the front axle
        lr   % distance between the centre of gravity and the rear axle
        Cy   % cornering stiffness
    end
    
    methods
        function obj = Vehicle(m, Jz, lf, lr, Cy)
           % initialize vehicle object
           obj.m = m;
           obj.Jz = Jz;
           obj.lf = lf;
           obj.lr = lr;
           obj.Cy = Cy;
        end
        
        function paramList = ReturnParams(obj)
           paramList = {obj.m; 
                        obj.Jz; 
                        obj.lf;
                        obj.lr;
                        obj.Cy};
        end
        
    end
end

