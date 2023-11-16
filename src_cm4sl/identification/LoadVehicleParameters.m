function vehicle = LoadVehicleParameters()

    % big vehicle
    lf = 1.311; % distance between the centre of gravity and the front axle
    lr = 1.311;  % distance between the centre of gravity and the rear axle
    m = 1599.98;   % mass
    Jz = 2393.665;  % moment of inertia around z axis
    Cf = 72705; %N/rad, front wheel cornering stiffness
    Cr = 72705; %N/rad, rear wheel
    vehicle = Vehicle(m, Jz, lf, lr, Cf); % create vehicle object
end

