clc; close all; clear all; format long; format compact;
% Program used to  generate probability density function
% Preallocation
H = zeros(1,10000);
X = zeros(1,10000);
count = 0;

XMAX = 6;
XMIN = -6;
RANGE= XMAX-XMIN;
TMP = 1./sqrt(6.28);
N = 100;
BIN = 50;
for i = 1:N
    SUM = 0;
    for j = 1:12
        RAND = rand(1);
        SUM  = SUM + RAND;
    end
    X(i) = SUM - 6;
end

for i = 1:BIN
    H(i) = 0;
end

for i = 1:N
    K = fix(((X(i)-XMIN)/RANGE)*BIN) + 0.99;
    if K < 1
        K = 1;
    end
    if K > BIN
        K = BIN;
    end
    K = round(K);
    H(K) = H(K) + 1;
end

for k = 1:BIN
    PDF = (H(k)/N)*BIN/RANGE;
    AB = XMIN + k*RANGE/BIN;
    TH = TMP*exp(-AB*AB/2);
    count = count + 1;
    array_AB(count) = AB;
    array_PDF(count)= PDF;
    array_TH(count) = TH;
end

figure(1);
plot(array_AB,array_PDF,array_AB,array_TH); grid on;
title('Sample Gaussian distribution');
xlabel('X');
ylabel('Probability Density Function');