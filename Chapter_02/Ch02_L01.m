clc; close all; clear all; format long; format compact;

%% Intro to Proportional Navigation
% Two-dimensional missile-target engagement simulation where the inputs are
% the initial location of the missile & target, speeds, flight time, and
% effective navigation ratio.

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
%                   Given parameters (pp. 18-19)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
% Situational variables
HEDEG   = -20;   % heading error [deg]
N       = 4;     % effective navigation ratio
HE      = deg2rad(HEDEG); % heading error [rad]

% Missile Parameters
RMx     = 0;     % x position
RMy     = 10000; % y position
VM      = 3000;  % velocity of missile

% Target Parameters
RTx     = 40000; % x position
RTy     = 10000; % y position
nT      = 0;     % target's acceleration (evasive maneuver)
BETA    = 0;     % target's flight-path angle
VT      = 1000;  % velocity of target
VTx     = -VT*cos(BETA); 
VTy     = VT*sin(BETA);

% Relative dynamics
RTMx = RTx - RMx;
RTMy = RTy - RMy;
RTM  = sqrt(RTMx^2 + RTMy^2);
LAM  = atan2(RTMy,RTMx);            % LOS angle
LEAD = asin(VT*sin(BETA+LAM)/VM);   % missile lead angle 
THETA= LAM+LEAD;                    % missile flight-path angle
VMx  = VM*cos(THETA+HE); % missile x velocity
VMy  = VM*sin(THETA+HE); % missile y velocity
VTMx = VTx - VMx;        % RELATIVE x vel.
VTMy = VTy - VMy;        % RELATIVE y vel.
Vc   = -(RTMx*VTMx + RTMy*VTMy)/RTM; % closing velocity

% Simulation 
T = 0; % initial time
S = 0; % down sample (to not store excess data)
n = 0; % counter variable for iteration

% Begin simulation while closing velocity is non-zero
while Vc >= 0
    if RTM < 1000
        dT = 2e-4; % <-- if relative distance is small, refine step size
    else
        dT = 1e-2;
    end
    % Retain values at current time step, x_n
    BETA_OLD = BETA;
    RTx_OLD  = RTx;
    RTy_OLD  = RTy;
    RMx_OLD  = RMx;
    RMy_OLD  = RMy;
    VMx_OLD  = VMx;
    VMy_OLD  = VMy;
    STEP = 1;
    FLAG = 0;
    while STEP <= 1 
        % PASS 2: After calculating relative dynamics, determine
        % provisional states x_p
        if FLAG == 1
            STEP = 2;
            % Forward Euler for predictor updates: x_p = x_n + dT*f(x_n,t)
            BETA = BETA + dT*dBETA;
            RTx  = RTx + dT*VTx;
            RTy  = RTy + dT*VTy;
            RMx  = RMx + dT*VMx;
            RMy  = RMy + dT*VMy;
            VMx  = VMx + dT*AMx;
            VMy  = VMy + dT*AMy;
            T = T + dT; % advance to next time step
        end
        % PASS 1: Calculate current missile-target states, x_n
        % PASS 3: Calculate predictor states, x_p, again for T+dT
        RTMx = RTx - RMx;
        RTMy = RTy - RMy;
        RTM  = sqrt(RTMx^2 + RTMy^2);
        VTMx = VTx - VMx;
        VTMy = VTy - VMy;
        Vc   = -(RTMx*VTMx + RTMy*VTMy)/RTM;
        LAM  = atan2(RTMy,RTMx);
        dLAM = (RTMx*VTMy - RTMy*VTMx)/(RTM^2);
        nc   = N*Vc*dLAM;
        AMx  = -nc*sin(LAM);
        AMy  = nc*cos(LAM);
        VTx  = -VT*cos(BETA);
        VTy  = VT*sin(BETA);
        dBETA= nT/VT;
        FLAG = 1;
    end
    FLAG = 0; % reset flag to retain values during next iteration
    %----------------------------------------------------------------------
    % RK2 (Heun) CORRECTOR:
    %
    % At this point:
    %   - *_OLD variables hold the start-of-step state x_n
    %   - Current state variables hold the predicted state x_p
    %   - Current derivative variables correspond to slopes at x_p (end 
    %     slopes)
    %
    % The Heun update is:
    %   x_{n+1} = x_n + (dT/2) * [ f(x_n) + f(x_p) ]
    %
    % This code uses an algebraically equivalent form:
    %   x_{n+1} = 0.5*( x_n + x_p + dT * f(x_p) )
    %
    % Because x_p already equals x_n + dT*f(x_n), this produces the average
    % -slope RK2.
    %----------------------------------------------------------------------
    BETA = 0.5*(BETA_OLD + BETA + dT*dBETA);
    RTx  = 0.5*(RTx_OLD + RTx + dT*VTx);
    RTy  = 0.5*(RTy_OLD + RTy + dT*VTy);
    RMx  = 0.5*(RMx_OLD + RMx + dT*VMx);
    RMy  = 0.5*(RMy_OLD + RMy + dT*VMy);
    VMx  = 0.5*(VMx_OLD + VMx + dT*AMx);
    VMy  = 0.5*(VMy_OLD + VMy + dT*AMy);
    S = S+dT;
    if S >= 0.09999     % logging data every 0.1 second (downsampling)
        S = 0;
        n = n+1;
        array_T(n) = T;
        array_RTx(n) = RTx;
        array_RTy(n) = RTy;
        array_RMx(n) = RMx;
        array_RMy(n) = RMy;
        array_nc_g(n)= nc/32.2; % convert commanded acceleration to g's
        array_RTM(n) = RTM;
    end
end
% Simulation results and plotting
RTM
figure(1);
plot(array_RTx, array_RTy, array_RMx, array_RMy); grid on;
title('Two-dimensional tactical missile-target engagement simulation');
xlabel('Downrange (Ft) ');
ylabel('Altitude (Ft)');
figure(2);
plot(array_T,array_nc_g); grid on;
title('Two-dimensional tactical missile-target engagement simulation');
xlabel('Time (sec)');
ylabel('Acceleration of missle (G)');