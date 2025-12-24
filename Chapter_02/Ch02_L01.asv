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
RMy     = 1e5;   % y position
VM      = 3000;  % velocity of missile

% Target Parameters
RTx     = 4e5;   % x position
RTy     = 1e5;   % y position
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

while Vc >= 0
    if RTM < 1000
        dT = 2e-4;
    else
        dT = 1e-2;
    end
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
        if FLAG == 1
            STEP = 2;
            BETA = BETA + dT*dBETA;
            RTx  = RTx + dt*VTx;
            RTy  = RTy + dt*VTy;
            RMx  = RMx + dT*VMx;
            RMy  = RMy + dT*VMy;
            VMx  = VMx + dT*AMx;
            VMy  = VMy + dT*AMy;
            T = T + dT;
        end
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
    FLAG = 0;
    BETA = 0.5*(BETA_OLD + BETA + dT*dBETA);
    RTx  = 