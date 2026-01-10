clc; close all; clear all; format long; format compact;

% MATLAB script computes the sampled standard deviation without using any
% of the MATLAB's internal math functions. More specifically, this is a
% Monte Carlo simulation to estimate standard  deviation.

% Preallocate variables 
Z = zeros(1, 100);
count = 0;  % iteration counter variable
Z1 = 0;     % mean of sampel set "Z"

% Generate Gaussian-distributed samples
for i = 1:100
    SUM = 0;
    for j = 1:12
        RAND = rand(1);
        SUM = SUM + RAND;
    end
    X = SUM - 6;    % zero-mean Gaussian adjustmenet
    Z(i) = X;
    Z1 = Z(i) + Z1;
    XMEAN = Z1/i;
end
SIGMA = 0;
Z1 = 0;     % now square deviations
for i = 1:100
    Z1 = (Z(i)-XMEAN)^2 + Z1;
    if i == 1
        SIGMA = 0;
    else
        SIGMA = sqrt(Z1/(i-1)); % sample standard dev.
    end
    count = count + 1;
    array_i(count) = i;
    array_SIGMA(count) = SIGMA;

end
figure(1);
plot(array_i, array_SIGMA); grid on;
title('Sampled Standard Deviation');
xlabel('Number of Samples');
ylabel('Calculated Standard Deviation');