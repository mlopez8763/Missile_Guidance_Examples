clc; close all; clear all; format long; format compact;

% Adjoint construction for a single lag pronav homing loop.
nT = 3*32.2;    % target maneuver 
N  = 4;         % effective navigation ratio
TAU = 1;        % time constant
Tf  = 10;       % flight time
Vm  = 3000;     % missile velocity
HE  = deg2rad(-20); % heading error
T   = 0;        % start flight time

% Simulation variables
n  = 0;             % iteration counter
S  = 0;             % sampling counter 
TP = T + 0.00001;   % simulation start time
X1 = 0;
X2 = 0;
X3 = 1;
X4 = 0;
h  = 0.01;          % step size
% -------------------------------------------------------------------------
% NOTE:
%   The X_ variables correspond to the signals in Fig. 3.12. In the
%   following paragraph, it is explained that:
%   X1: the miss distance sensitivity, multiplying by nT to get miss
%   X2: related to the miss sensitivity due to an initial heading error,
%       multiply by -Vm*HE to get miss
%   X3: the derivative 
% -------------------------------------------------------------------------
while TP <= (Tf - 1e-5)
    % preserve current states
    X1_OLD = X1;
    X2_OLD = X2;
    X3_OLD = X3;
    X4_OLD = X4;
    STEP = 1;
    FLAG = 0;
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            % forward Euler from derivatives
            X1 = X1 + h*dX1;
            X2 = X2 + h*dX2;
            X3 = X3 + h*dX3;
            X4 = X4 + h*dX4;
            TP = TP + h; % increase time
        end
        % compute derivatives for prediction
        TGO = TP + 0.00001;
        Y1  = (X4 - X2)/TAU;
        dX1 = X2;
        dX2 = X3;
        dX3 = Y1*N/TGO;
        dX4 = -Y1;
        FLAG= 1;
    end
    FLAG = 0; % reset for next iteration
    % finish 2nd order Runge-Kutta method
    X1 = 0.5*(X1 + X1_OLD + h*dX1);
    X2 = 0.5*(X2 + X2_OLD + h*dX2);
    X3 = 0.5*(X3 + X3_OLD + h*dX3);
    X4 = 0.5*(X4 + X4_OLD + h*dX4);
    S = S + h;
    % sample data from simulation
    if S >= 0.0999
        S = 0;
        n = n + 1;
        array_TP(n) = TP;
        array_target_maneuver_miss(n) = X1*nT;
        array_target_HE_miss(n) = -Vm*HE*X2;
    end
end
% plot results
figure(1);
plot(array_TP,array_target_maneuver_miss); grid on;
xlabel('Flight Time (Sec)')
ylabel('Target Maneuver Miss (Ft)')
figure(2);
plot(array_TP,array_target_HE_miss); grid on;
xlabel('Flight Time (Sec)')
ylabel('Heading Error Miss (Ft)')