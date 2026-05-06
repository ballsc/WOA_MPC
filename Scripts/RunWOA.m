clear

% declare variables for use in WOA simulink
Max_iteration = 100; % number of iterations
SearchAgents_no = 8; % number of whales
lb = [0 0 0 0 1 1]; % lower bounds for each variable
ub = [4 3 5 5 15 15]; % upper bounds for each variable
var_list = ["Kp", "Ki", "Qy", "Qyaw", "Rang", "Sang"]; % variable names to change in simulink
dim = size(var_list, 2);
par_flag = 1; % 1 to make whales run in parallel, 0 else

% load in necessary variables for simulink model
run("LoadDrivingScenario")

model = 'DrivingScenario'; % name of simulink model to be modified
load_system(model)

% Start parallel pool
if par_flag
    pool = gcp('nocreate');
    if isempty(pool)
        parpool('local');
    end
end

% Run WOA with the defined model on the defined variables
[Leader_score,Leader_pos,Convergence_curve] = WOA_simulink(SearchAgents_no, ...
                                                Max_iteration, lb, ub, dim, ...
                                                model, var_list, 1, par_flag);

% Results
disp(Leader_score)
disp(Leader_pos)