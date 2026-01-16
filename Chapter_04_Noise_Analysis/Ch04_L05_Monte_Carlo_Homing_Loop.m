clc; close all; clear all; format long; format compact;

% Monte Carlo simulation of homing loop with random target maneuver

% Preallocate variables
Z  = zeros(1,1000);
TF = zeros(1,10);
I  = zeros(1,100);
array_TF = zeros(1,10);
array_SIGMA = zeros(1,10);
array_XMEAN = zeros(1,10);
count = 0;      % iteration counter
Vc = 4000;
NT = 3*32.2;    % 3G target maneuver
N  = 3;         % effective ProNav ratio
TAU= 1;         % time constant
RUN= 50;        % number of Monte Carlo simulations
% Begin simulation for ten different flight times
for TF = 1:10
    Z1 = 0;
    for i = 1:RUN
        SUM = rand(1);
        TSTART = TF*SUM;
        PZ = rand(1);
        PZ = PZ - 0.5;
        if PZ > 0
            COEF = 1;
        else 
            COEF = -1;
        end
        Y = 0;
        dY= 0;
        T = 0;
        h = 0.01;
        S = 0;
        NC= 0;  % missile commanded acceleration
        NL= 0;  % missile achieved acceleration
        % Begin Monte Carlo simulations per each flight time
        while T <= (TF - 1e-5)
            if T < TSTART
                NT = 0;
            else
                NT = COEF*3*32.2;
            end
            Y_OLD = Y;
            dY_OLD= dY;
            NL_OLD= NL;
            STEP = 1;
            FLAG = 0;
            while STEP <= 1
                if FLAG == 1
                    % Forward Euler
                    Y = Y + h*dY;
                    dY= dY + h*ddY;
                    NL= NL + h*dNL;
                    T = T + h;
                    STEP = 2;
                end
                % Relative dynamics (see equations in textbook)
                TGO = TF - T + 0.00001;
                RTM = Vc*TGO;
                dLAM= (RTM*dY + Y*Vc)/(RTM^2);
                NC  = N*Vc*dLAM;
                dNL = (NC - NL)/TAU;
                ddY = NT-NL;
                FLAG= 1;
            end
            FLAG = 0;
            % Finish RK-2 method
            Y = 0.5*(Y_OLD + Y + h*dY);
            dY= 0.5*(dY_OLD + dY + h*ddY);
            NL= 0.5*(NL_OLD + NL + h*dNL);
            S = S + h; % down-sample
        end
        Z(i) = Y;
        Z1 = Z(i) + Z1;
        XMEAN = Z1/i;
    end
    SIGMA = 0;
    Z1 = 0;
    for i = 1:RUN
        Z1 = (Z(i) - XMEAN)^2 + Z1;
        if i == 1
            SIGMA = 0;
        else
            SIGMA = sqrt(Z1/(i-1));
        end
    end
    count = count + 1;
    array_TF(count) = TF;
    array_SIGMA(count) = SIGMA;
    array_XMEAN(count) = XMEAN;
end
% Plot results
figure(1);
plot(array_TF, array_SIGMA, 'r+'); grid on;
title('Shaping filter Monte Carlo results')
xlabel('Time')
ylabel('Standard Deviation / Mean')
axis([00,10,00,30])