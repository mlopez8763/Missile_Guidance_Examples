clc; close all; clear all; format long; format compact;

TAU = 0.2;          % correlation time constant
PHI = 1.0;          % white noise
count = 0;
T = 0;
h = 0.01;           % integration step size
SIG = sqrt(PHI/h);  % standard deviation of pseudo-white noise
Y = 0;  
while T <= 5.0
    X = SIG*randn;
    Y_OLD = Y;
    STEP = 1;
    FLAG = 0;
    while STEP <= 1
        % 2nd pass using the predicted derivatives
        if FLAG == 1            
            STEP = 2;
            Y = Y + h*dY;       % forward Euler
            T = T + h;
        end
        % Predict derivatives
        dY = (X - Y)/TAU;
        FLAG = 1;
    end
    FLAG = 0;
    Y = (Y_OLD + Y + h*dY)/2;   % RK-2 method
    SIG_plus = sqrt(PHI*(1-exp(-2*T/TAU))/(2*TAU));
    SIG_minus= -SIG_plus;
    count = count + 1;
    % Store simulation data
    array_T(count) = T;
    array_Y(count) = Y;
    array_SIG_plus(count) = SIG_plus;
    array_SIG_minus(count)= SIG_minus;
end
% Display simulation data
figure(1);
plot(array_T, array_Y, array_T, array_SIG_plus, array_T, array_SIG_minus);
grid on;
title('Simulation of low-pass filter driven by white noise')
xlabel('Time (S)')
ylabel('Y')