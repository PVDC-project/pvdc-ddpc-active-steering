close all;clc;

Ts = 0.05;
X_ref = 60/3.6*0:Ts:50;  % time=0:Ts:Tsim
N = 20;

for i=1:length(X_ref)
    X_along_horizon = X_ref(i:i+N);
    
    shape = 2.4;
    dx1 = 25;
    dx2 = 21.95;
    dy1 = 4.05;
    dy2 = 5.7;
    Xs1 = 27.19;
    Xs2 = 56.45;

    DLCoffset = 0;

    z1 = shape/dx1*(X_along_horizon - DLCoffset - Xs1) - shape/2;
    z2 = shape/dx2*(X_along_horizon - DLCoffset - Xs2) - shape/2;

    YawRef_along_horizon = atan( dy1*(1./cosh(z1)).^2*(1.2/dx1) - dy2*(1./cosh(z2)).^2*(1.2/dx2));
    Y_along_horizon = dy1/2*(1+tanh(z1)) - dy2/2*(1+tanh(z2));

    % PM
    K_along_horizon_PM = -((dy1.*shape.^2.*tanh(shape./2 - (shape.*(X_along_horizon - DLCoffset - Xs1))./dx1).*(tanh(shape./2 - (shape.*(X_along_horizon - DLCoffset - Xs1))./dx1).^2 - 1))./dx1.^2 - (dy2.*shape.^2.*tanh(shape./2 - (shape.*(X_along_horizon - DLCoffset - Xs2))./dx2).*(tanh(shape./2 - (shape.*(X_along_horizon - DLCoffset - Xs2))./dx2).^2 - 1))./dx2.^2)./(((dy1.*shape.*(tanh(shape./2 - (shape.*(X_along_horizon - DLCoffset - Xs1))./dx1).^2 - 1))./(2.*dx1) - (dy2.*shape.*(tanh(shape./2 - (shape.*(X_along_horizon - DLCoffset - Xs2))./dx2).^2 - 1))./(2.*dx2)).^2 + 1).^(3./2);

    DX = diff(X_along_horizon) / Ts; 
    DX = [DX, DX(end)];
    DY = diff(Y_along_horizon) / Ts;
    D2Y = diff(DY) / Ts; 
    DY = [DY, DY(end)];
    D2Y = [D2Y, D2Y(end), D2Y(end)];
    K_along_horizon = DX.*D2Y./(DX.^2+DY.^2).^(3/2);
    
    
    
    if (~mod(i,10))
        plot(i:i+N,K_along_horizon,'b'); hold on; plot(i:i+N,K_along_horizon_PM,'r')
        legend({'formula','PM'})
    end
    
    pause(0.01)
    % figure;plot(K_along_horizon-K_along_horizon_PM)
    % plot(K_along_horizon-K_along_horizon_PM)
end

