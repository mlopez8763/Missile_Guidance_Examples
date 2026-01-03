clc; close all; clear all; format long; format compact;

%% Linearized two-dimensional missile-target engagement simulation
% -------------------------------------------------------------------------
% PARAMETERS:
%   Vc = closing velocity
%   nT = target acceleration
%   Y  = missile j-hat distance to origin (LOS)
%   Vm = missile velocity
%   HEDEG = heading error in degree
%   Tf = total flight time
%   N  = effective navigation ratio   
%   dY = missile velocity (linearized)
%   LAM= LOS angle
%   nc = commanded acceleration (for missile)
% -------------------------------------------------------------------------

% Given situational variables
Vc = 4000;
nT = 0;
Y  = 0;
Vm = 3000;
HEDEG = -20;
Tf = 10;
N  = 4; 
dY = -Vm * deg2rad(HEDEG);% initial sideways velocity due to heading error
T  = 0;
dT = 0.01;

% Helper simulation variables
S  = 0;
n  = 0;

% Simulation
while T <= Tf-1e-5 % prevent catastrophic cancellation 
    % Preserve current time step states
    Y_OLD = Y;
    dY_OLD = dY;
    STEP = 1;
    FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            Y = Y + dT*dY; % forward Euler
            dY= dY + dT*ddY;
            T = T + dT;
        end
        TGO = Tf - T + 0.00001;
        dLAM= (dY*TGO + Y)/(Vc*TGO*TGO);
        nc  = N*Vc*dLAM; % proportional navigation law
        ddY = nT-nc;
        FLAG= 1;
    end
    FLAG = 0;
    % 2nd order RK
    Y = 0.5*(Y + Y_OLD + dT*dY);
    dY= 0.5*(dY+ dY_OLD + dT*ddY);
    S = S + dT;
    if S >= 0.0999
        S = 0;
        n = n+1;
        array_T(n) = T;
        array_Y(n) = Y;
        array_dY(n)= dY;
        array_nc_g(n) = nc/32.2; % convert commanded acceleration to G's
    end
end
% Display results
figure(1);
plot(array_T,array_nc_g); grid on;
xlabel('Time [s]');
ylabel('Missile Acceleration [G]');
