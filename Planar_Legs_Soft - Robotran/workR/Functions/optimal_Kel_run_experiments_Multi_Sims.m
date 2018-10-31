% This file executes multiple simulations about the walking task for a
% Rigid Robot. The optimal problem return position speed acceleration of
% each single joints fulfilling several constraints on speed, torque limit
% and force. It generates two folders: one for the walk task and the other
% for the run task. Moreover it is also possible to modify speed and torque
% limit setting the respective parameters.

clc
clearvars

addpath('Casadi_Funs','Functions')
import casadi.*

% Run and Walk tasks, create necessary structures and call the optimization
% problem

% In order to split the whole amount of simulations or to recover the 
% simulation set after an unexpected interrupt here a structure is created
% SIMMAT_WALK and RUN
VAL=10;

for VAL=60

ExpVal=0.42;

% name1 =[ 'G_Run_Soft_K_',num2str(VAL),'_experiments_EV_',num2str(ExpVal*100),'New_Set/']; 
% name2 = 'Subfolder_sim_run_';
name1 =[ 'G_Run_Soft_K_',num2str(VAL),'_experiments_EV_',num2str(ExpVal*100),'New_Set_TESTRICHI/']; 
name2 = 'Subfolder_sim_run_';

% This code calculates the step lenght and time w.r.t. the data and trend
% referred in the paper "A simple model....: Step Length relationship"
% leg_length

l=0.3; %leg_length
g=9.81;
 
% Walk_Speed=[0.04 0.06 0.08 0.1 0.12 0.14];
Walk_Speed=[0.31 0.41 0.44 0.48 0.49 0.5];% 0.06 0.08 0.1 0.12 0.14];

% Walk_Speed_2=[0.02:0.0025:0.5];
% Foot_H_2=[0.01:0.0025:0.03];%0.015;

Walk_Speed_norm=Walk_Speed./sqrt(g*l); % WalkSpeed_norm is ni on the paper (Gasparri) 
% Step_L_norm=(Walk_Speed.^0.42);

alpha_run = 1.068;

Step_L_norm=alpha_run*(Walk_Speed.^ExpVal); % ExpVal is beta on the paper (Gasparri et al) equation (1)
Step_L=Step_L_norm*l;
 
T=Step_L./Walk_Speed;
 
% Foot_H=[0.015 0.02];
% Foot_H=[0.01 0.015 0.02];
Foot_H=[0.02];%0.015;

 
SIMMAT_ST=[ T' Step_L'];
SIMMAT=[];
if length(Foot_H)<=1
   SIMMAT=[ SIMMAT_ST ones(length(SIMMAT_ST),1)*Foot_H]; 
else
    for i=1:length(Foot_H)
    SIMMATm=[ SIMMAT_ST ones(length(SIMMAT_ST),1)*Foot_H(i)];
    SIMMAT=[SIMMAT;SIMMATm];
    end
end

save('SIMMAT_RUN','SIMMAT')
size(SIMMAT)

% if exist(name1) 
% else mkdir(name1)
% end
mkdir(name1)

flag_gamma = 1;
    
% mat_ris is a structure which summarize all the results
mat_ris=cell(size(1,1),14); 

% Index used to correctly concatenate the results
Mindex=1; 
successful_comb = {};
successful_cases = [];
i_succ = 0;

% Set of simulations to test
start_sim=1; 
end_sim=max(size(SIMMAT));
 
% Check Dimensions 
if start_sim<1
    start_sim=1;
end
if start_sim>size(SIMMAT,1)
    start_sim=1;
    end_sim=size(SIMMAT,1);
end
if end_sim>size(SIMMAT,1)
    end_sim=size(SIMMAT,1);
end

% Now execute the optimizations
for index=start_sim:end_sim 
    
% Speed and torque limits
    max_speed=4;
    min_speed=-4;
    max_torque=6;
    min_torque=-6;      
 
%     max_speed=inf;
%     min_speed=-inf;
%     max_torque=inf;
%     min_torque=-inf;  
%     
    K_el_min = VAL;
    K_el_max = VAL;
    K_el_guess = VAL;
    
% Set optimization parameters 
% max iterations
    maxiter=3000;
% tolerance
    tol=1.e-6;
% This flag enables the application of dual infeasibility tolerance    
    flag_dual=0;
    dual_tol=1.e-6; %default = 1
    
% This flag enables the constraints on the CoP    
    flag_COP=1;

% Further optimization parameters    
%     par = NM{index,5};
%     if par(3) < T && par(2)>h_foot
        
        T=SIMMAT(index,1); %par(3); %
        walk_dist=SIMMAT(index,2); %par(1); %
        h_foot=SIMMAT(index,3); %par(2); %

        % number of time intervals
        N = 30;
        
        T_s = T/(5*N);

%         T_s = [ inf T/N inf ];
%         cases = combvec(T_s,T_s,T_s);

        n_step_h =  9;
        fc_per_tol = 2.5; % N  % 2.5
        tau_per_tol = 0.25; % Nm  % 0.25
        
        % set trial coefficients for Baumgarte's stabilization
        zita = 1;        % damping ratio
        
        gamma = 3/T_s;
        
%         gamma_x = 3/T_s(1);   % natural frequency
%         gamma_y = 3/T_s(2);
%         gamma_alpha = 3/T_s(3);

%         gamma_x = 3/cases(1,i_comb);   % natural frequency
%         gamma_y = 3/cases(2,i_comb);
%         gamma_alpha = 3/cases(3,i_comb);

        % solve the ocp
        damping = 0.5;%0.3;
        damping_L = 0.01;
        N_trans = N/3;
%         N_trans = 9*N/10;
%         N_trans = N-2;
        
        a_r = 0.06;  % TODO: conservative epsilon
        a_f = 0.06;
        
        mu = 0.7;   % friction coefficient
        
        optimal_Kel_run_NLproblem_MG_experiments                      
%         optimal_Kel_walk_NLproblem_MG_exp_new_foot_shape

        status=solver.stats.return_status;
        
        for ind_gamma = 1:3
            if not(strcmp(status,'Solve_Succeeded') )        
                gamma = gamma - gamma/6;
                
%                 gamma = gamma_y;

                optimal_Kel_run_NLproblem_MG_experiments

                status=solver.stats.return_status;
            end
        end
        
        % run
        gamma_x = gamma;
        gamma_y = gamma;
        gamma_alpha = 0;

        save_res
        if strcmp(status,'Solve_Succeeded') || strcmp(status,'Solved_To_Acceptable_Level')
            
            % check on drift
            if max(abs(inv_err_pos_x)) < 5e-3 && max(abs(inv_err_pos_y)) < 5e-3 && max(abs(inv_err_pos_gamma)) < 1.e-2
                i_succ = i_succ+1;            
    %             successful_comb{i_succ,1} = [gamma_x, gamma_y, gamma_alpha];
                successful_case_vec = [index];
                successful_case_vec = [successful_case_vec; full(val_opt)];
                successful_case_vec = [successful_case_vec; walk_dist/T];
                successful_case_vec = [successful_case_vec; h_foot];
                
                successful_cases = [successful_cases successful_case_vec];

                forces_post_processing_run
            end % check on drift

        end % if solve succeeded
        
%     end %  if T and h_foot
% 
plot_step_shape;
saveas(gcf,[name_sim_folder,'/StepShape.fig'])
saveas(gcf,[name_sim_folder,'/StepShape.png'])

def=res_theta-res_q(4:end,:);
figure('Name','Def')
plot(rad2deg(def)','LineWidth',2)
saveas(gcf,[name_sim_folder,'/def_deg.fig'])
figure('Name','TH')
plot(rad2deg(res_theta)','LineWidth',2);

saveas(gcf,[name_sim_folder,'/th_deg.fig'])
figure('Name','QL')
plot(rad2deg(res_q(4:end,:))','LineWidth',2);
saveas(gcf,[name_sim_folder,'/ql_deg.fig'])

close all
end % for index

name_tot = [name1,'succ_cases'];
save(name_tot,'successful_cases')

name_whole=[name1,'Whole_',num2str(start_sim),'_',num2str(end_sim)];
save(name_whole,'mat_ris','min_speed','max_speed','min_torque','max_torque');

end
