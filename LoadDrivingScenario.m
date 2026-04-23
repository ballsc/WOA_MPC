% 25 mph
%initialVehicleVelocity = 8.9;
% 35 mph
initialVehicleVelocity = 15.6;

simStepSize = 0.05;

load("busDefinitions.mat");

% Load weights
% load("best_whale_lookAhead10.mat")
% Kp = Leader_pos(1); Ki = Leader_pos(2); Qy = Leader_pos(3); 
% Qyaw = Leader_pos(4); Rang = Leader_pos(5); Sang = Leader_pos(6);
% 
% clear Leader_pos Leader_score