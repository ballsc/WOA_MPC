clear

% declare variables for use in WOA simulink
Max_iteration = 100;
SearchAgents_no = 8; % size of max parpool
lb = [0 0 0 0 1 1];
ub = [4 3 5 5 15 15];
var_list = ["Kp", "Ki", "Qy", "Qyaw", "Rang", "Sang"];
dim = size(var_list, 2);
par_flag = 1;

% load in necessary variables for simulink model
run("LoadDrivingScenario")

model = 'DrivingScenario';
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