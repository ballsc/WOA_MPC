%__________________________________________
% model = simulink model
% var_list = list of simulink variables
% dim = number of your variables
% Max_iteration = maximum number of generations
% SearchAgents_no = number of search agents
% lb=[lb1,lb2,...,lbn] where lbn is the lower bound of variable n
% ub=[ub1,ub2,...,ubn] where ubn is the upper bound of variable n
% If all the variables have equal lower bound you can just
% define lb and ub as two single number numbers


% The Whale Optimization Algorithm
function [Leader_score, Leader_pos, Convergence_curve] = WOA_simulink(SearchAgents_no, Max_iter, lb, ub, dim, model, var_list, value, par_flag)

% Initialize position vector and score for the leader
Leader_pos = zeros(1, dim);
Leader_score = -inf; %-inf for maximization, inf for minimization

% Initialize the positions of search agents
Positions = initialization(SearchAgents_no, dim, ub, lb);

Convergence_curve = zeros(1, Max_iter);

t = 0; % Loop counter

simIn(SearchAgents_no,1) = Simulink.SimulationInput(model);
All_fitness = -inf(1, SearchAgents_no);

% Main loop
while t < Max_iter
    
    % Runs whales in parallel
    if par_flag
        for i = 1:SearchAgents_no
            
            % Return back the search agents that go beyond the boundaries of the search space
            Flag4ub = Positions(i,:) > ub;
            Flag4lb = Positions(i,:) < lb;
            Positions(i,:) = (Positions(i,:).*(~(Flag4ub + Flag4lb))) + ub.*Flag4ub + lb.*Flag4lb;
    
        end
    
        % Build SimIn Array for current generation
        for i = 1:SearchAgents_no
            simIn(i) = Simulink.SimulationInput(model);
    
            % Set variables in simIn from each whale
            for j = 1:dim
                simIn(i) = simIn(i).setVariable(var_list(j), Positions(i, j));
            end
    
            % Set simulation parameters
            simIn(i) = simIn(i).setModelParameter("SimulationMode","accelerator");
        end
    
        % Simulate all whales in parallel
        simOut = parsim(simIn, "ShowProgress", "off", ...
                        "TransferBaseWorkspaceVariables", "on", ...
                        "AttachedFiles", "OvalTrack.mat");
    
        % Evaluate fitness from each output
        for i = 1:SearchAgents_no
            if isempty(simOut(i).ErrorMessage)
                % Cost is based on total time in simulation
                All_fitness(i) = max(simOut(i).logsout{1}.Values.Time)
            else
                % Penalize failed runs
                All_fitness(i) = -inf;
            end
    
            if All_fitness(i) > Leader_score
                Leader_score = All_fitness(i);
                Leader_pos = Positions(i,:);
            end
        end

    % Runs whales one at a time
    else
        for i=1:size(Positions,1)
        
            % Return back the search agents that go beyond the boundaries of the search space
            Flag4ub=Positions(i,:)>ub;
            Flag4lb=Positions(i,:)<lb;
            Positions(i,:)=(Positions(i,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;
            
            in = Simulink.SimulationInput(model);
    
            for j = 1:size(var_list, 2)
                in = in.setVariable(var_list(j), x(j));
            end
    
            out = sim(in);
        
            % Current cost is based on survival time in the simulation.
            % Change if you have a specific error to minimize/ value to maximize
            fitness = max(out.logsout{1}.Values.Time);
            % Cost = out.logsout.find("Variable").Values.Data
            
            % Update the leader
            if fitness<Leader_score % Change this to > for maximization problem
                Leader_score=fitness; % Update alpha
                Leader_pos=Positions(i,:);
            end
        
        end
    end
    
    a = 2 - t*((2)/Max_iter); % a decreases linearly fron 2 to 0 in Eq. (2.3)
    
    % a2 linearly dicreases from -1 to -2 to calculate t in Eq. (3.12)
    a2 = -1 + t*((-1)/Max_iter);
    
    % Update the Position of search agents 
    for i = 1:size(Positions,1)
        r1 = rand(); % r1 is a random number in [0,1]
        r2 = rand(); % r2 is a random number in [0,1]
        
        A = 2*a*r1 - a;  % Eq. (2.3) in the paper
        C = 2*r2;      % Eq. (2.4) in the paper
        
        
        b = 1;               %  parameters in Eq. (2.5)
        l = (a2 - 1)*rand + 1;   %  parameters in Eq. (2.5)
        
        p = rand();        % p in Eq. (2.6)
        
        for j = 1:size(Positions,2)
            
            if p < 0.5   
                if abs(A) >= 1
                    rand_leader_index = floor(SearchAgents_no*rand()+1);
                    X_rand = Positions(rand_leader_index, :);
                    D_X_rand = abs(C*X_rand(j) - Positions(i,j)); % Eq. (2.7)
                    Positions(i,j) = X_rand(j) - A*D_X_rand;      % Eq. (2.8)
                    
                elseif abs(A)<1
                    D_Leader = abs(C*Leader_pos(j) - Positions(i,j)); % Eq. (2.1)
                    Positions(i,j) = Leader_pos(j) - A*D_Leader;      % Eq. (2.2)
                end
                
            elseif p>=0.5
              
                distance2Leader = abs(Leader_pos(j) - Positions(i,j));
                % Eq. (2.5)
                Positions(i,j) = distance2Leader*exp(b.*l).*cos(l.*2*pi) + Leader_pos(j);
                
            end
            
        end
    end
    
    t=t+1;
    Convergence_curve(t) = Leader_score;
    
    if t>2
        line([t-1 t], [Convergence_curve(t-1) Convergence_curve(t)], 'Color', 'b')
        xlabel('Iteration');
        ylabel('Best score obtained so far');        
        drawnow
    end
 
    if value == 1
        hold on
        scatter(t*ones(1, SearchAgents_no), All_fitness, '.', 'k')
    end

    disp(Positions)
            
end
