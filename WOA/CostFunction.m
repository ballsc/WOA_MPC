% Cost function for WOA using simulink

% This cost function uses the provided model and variables, 
% Sets the variables in the simulink model,
% Then evaluates the system with the parameters and returns the result

function Cost = CostFunction(model, var_list, x)

    in = Simulink.SimulationInput(model);

    for i = 1:size(var_list, 2)
        in = in.setVariable(var_list(i), x(i));
    end

    out = sim(in);

    % Current cost is based on survival time in the simulation.
    % Change if you have a specific error to minimize/ value to maximize
    Cost = max(out.logsout{1}.Values.Time);
    % Cost = out.logsout.find("Variable").Values.Data

end