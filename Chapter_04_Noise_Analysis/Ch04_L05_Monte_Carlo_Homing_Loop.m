clc; close all; clear all; format long; format compact;

% Monte Carlo simulation of homing loop with random target maneuver

% Preallocate variables
Z  = zeros(1,1000);
TF = zeros(1,10);
I  = zeros(1,100);
array_TF = zeros(1,10);
array_SIGMA = zeros(1,10);
array_XMEAN = zeros(1,10);
count = 0; % iteration counter
