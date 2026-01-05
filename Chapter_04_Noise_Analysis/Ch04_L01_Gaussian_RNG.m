clc; close all; clear all; format long; format compact;
% Gaussian random number generator (w/o using 'randn()' function) 
count = 0;
N = 100;
for i = 1:N
    SUM = 0;
    for j = 1:12
        RAND = rand(1);
        SUM = SUM + RAND;
    end
    X = SUM - 6;
    count = count + 1;
    array_i(count) = i;
    array_X(count) = X;
end
figure(1);
plot(array_i, array_X); grid on;
title('One hundred random numbers with Gaussian distribution');
xlabel('Number');
ylabel('Value');