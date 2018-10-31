% This script allows you to get all the overall data Whole_xxx and
% succ_cases from recovered executions of the optimization process

addpath('Functions\');

resDirName = 'G_Walk_Soft_MultiParam_Bounded\';
foldCont = dir(resDirName);
index = 1;
i_succ = 0;
successful_cases = [];
mat_ris = [];
start_idx = 3; % Skip the standard folder (.\) and (..\)

for idx=start_idx:(size(foldCont,1)-2)
    % Move into the results folder
    cd(resDirName);
    
    if (foldCont(idx).isdir) 
        simName = foldCont(idx).name;
        cd(simName);
        load(simName(11:end));
        cd('..')
    end
    % Speed and torque limits
    max_speed=4;
    min_speed=-4;
    max_torque=6;
    min_torque=-6;
    
    % number of time intervals
    N = 30;
    N_trans = N-2;  
    
    % Unknown!
    T_s = [ inf T/N inf ];
    gamma_x = 3/T_s(1);   % natural frequency
    gamma_y = 3/T_s(2);
    gamma_alpha = 3/T_s(3);
        
    % store the results
    mat_ris{index,1}=index;
    mat_ris{index,2}=res_K;
    mat_ris{index,3}=status;
    mat_ris{index,4}=full(val);
    mat_ris{index,5}=[walk_dist , h_foot, T];
    mat_ris{index,6}=res_q;
    mat_ris{index,7}=res_qd;
    mat_ris{index,8}=res_qdd;
    mat_ris{index,9}=res_ts;
    mat_ris{index,10}=res_theta;
    mat_ris{index,11}=res_thetad;
    mat_ris{index,12}=tau;
    mat_ris{index,13}=res_Fc; 
    mat_ris{index,14}= [gamma_x gamma_y gamma_alpha]; 
%     index=index+1;
    
    % Unknown!
    time = vertcat(T/N*[0:N]);
    inv_err_pos_x = inv_err0(1,:);
    inv_err_pos_y = inv_err0(2,:);
    inv_err_pos_gamma = inv_err0(5,:);
    
    if strcmp(status,'Solve_Succeeded') || strcmp(status,'Solved_To_Acceptable_Level')
            
        % check on drift
        if max(abs(inv_err_pos_x)) < 5e-3 && max(abs(inv_err_pos_y)) < 5e-3 && max(abs(inv_err_pos_gamma)) < 1.e-2
            i_succ = i_succ+1;            
            successful_case_vec = [index];
            successful_case_vec = [successful_case_vec; full(val)];
            successful_case_vec = [successful_case_vec; walk_dist/T];
            successful_case_vec = [successful_case_vec; h_foot];
            successful_case_vec = [successful_case_vec; walk_dist];
            
            successful_cases = [successful_cases successful_case_vec];

%             forces_post_processing
        end % check on drift

    end % if solve succeeded
        
    cd('..');
    
end

 % Save retrieved data
name_tot = [resDirName,'succ_cases_',num2str((idx-2)),'_RETR'];
save(name_tot,'successful_cases')

name_whole=[resDirName,'Whole_',num2str(1),'_',num2str((idx-2)),'_RETR'];
save(name_whole,'mat_ris','min_speed','max_speed','min_torque','max_torque');
% Clear variables
rmpath('Functions\');
clearvars -except successful_cases mat_ris
