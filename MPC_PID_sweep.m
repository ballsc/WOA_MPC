clear

% declare variables for use in WOA simulink
Max_iteration = 100;
SearchAgents_no = 10;
lb = [0 0 0 0 1 1];
ub = [2 1 2 1 3 3];
var_list = ["Kp", "Ki", "Qy", "Qey", "Rang", "Sang"];
dim = size(var_list);

% load in necessary variables for simulink model
run("LoadDrivingScenario")

model = 'DrivingScenario';
load_system(model)

% Run WOA with the defined model on the defined variables
[Leader_score,Leader_pos,Convergence_curve] = WOA_simulink(SearchAgents_no, ...
                                                Max_iteration, lb, ub, dim, ...
                                                model, var_list, 1);

% Results
disp(Leader_score)
plot(size(Convergence_curve), Convergence_curve)