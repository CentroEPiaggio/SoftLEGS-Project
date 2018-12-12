%% Run the OPT procedure from the Robotran project 
function runOpt(Robotran_SLeg_Path) 
    cd(strcat(Robotran_SLeg_Path,'workR'));
    % run
    optimal_Kel_walk_experiments_Multi_Sims;
end