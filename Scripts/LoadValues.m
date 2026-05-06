% Load weights
% mat file with values you want to load, 
% uncomment if running after full run where Leader_pos is already defined
load("best_whale_lookAhead3_oval.mat")

% Assign positions of Leader_pos to variables to use in simulink
Kp = Leader_pos(1); Ki = Leader_pos(2); Qy = Leader_pos(3); 
Qyaw = Leader_pos(4); Rang = Leader_pos(5); Sang = Leader_pos(6);

clear Leader_pos Leader_score