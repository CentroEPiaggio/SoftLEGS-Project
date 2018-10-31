% This file executes multiple simulations about the walking task for a
% Rigid Robot. The optimal problem return position speed acceleration of
% each single joints fulfilling several constraints on speed, torque limit
% and force. It generates two folders: one for the walk task and the other
% for the run task. Moreover it is also possible to modify speed and torque
% limit setting the respective parameters.

clc
clearvars
global fldName walkS stepH stepL beta useBeta

addpath('Functions','Casadi_Funs')
import casadi.*

% Run and Walk tasks, create necessary structures and call the optimization
% problem

% In order to split the whole amount of simulations or to recover the 
% simulation set after an unexpected interrupt here a structure is created
% SIMMAT_WALK and RUN
VAL=10;

for VAL=60

% name1 =['G_Walk_Soft_Multiparam\']; 
name1 = fldName;
name2 = 'Subfolder_sim_walk_';

% This code calculates the step lenght and time w.r.t. the data and trend
% referred in the paper "A simple model....: Step Length relationship"
% leg_length

l=0.3; %leg_length
g=9.81;

%% EDITED By RICCARDO MENGACCI TO INSERT STEP LENGHT AS PARAMETERS

Walk_Speed = walkS;
Walk_Length = stepL;
Foot_H = stepH;
BETA = beta;

SIMMAT=[];
if (useBeta)
    % To use new SIMMAT (beta-shaped region) 
    disp([10,'USING BETA-SHAPED REGION FOR Speed/StepL!',10]);
    for i=1:length(Walk_Speed)
        for j=1:length(BETA)
            for k=1:length(Foot_H)
                Walk_Length = l*(Walk_Speed(i)^BETA(j));
                T = Walk_Length/Walk_Speed(i);
                SIMMAT = [SIMMAT; ...
                          T, Walk_Length, Foot_H(k)];
            end
        end
    end
else
    % To use original SIMMAT (square region cutted or not)
    disp([10,'USING SQUARE REGION FOR Speed/StepL!',10]);
    for i=1:length(Walk_Speed)
        for j=1:length(Walk_Length)
            for k=1:length(Foot_H)
                T = Walk_Length(j)/Walk_Speed(i);
                SIMMAT = [SIMMAT; ...
                          T, Walk_Length(j), Foot_H(k)];
            end
        end
    end
end
% Remove the data out of the admittable region beta in [0.15:0.56] and then plot the results
% COMMENT if you want to use original SIMMAT points
removeData

save('SIMMAT_WALK','SIMMAT')
size(SIMMAT)

% Initialize starting and ending index for the optimizations
start_sim=1; 
end_sim=max(size(SIMMAT));
    
% Check if a folder with the same name already exists and in the case
% ask for restart a previous collection or start a new one
if exist(name1) 
    start_sim = check2restart(name1);
    if (start_sim == -1)            % Do not save on the same folder!
        return
    end
else
    mkdir(name1);
end
    
% mat_ris is a structure which summarize all the results (filled in save_res)
mat_ris=cell(size(1,1),14); 

% Index used to correctly concatenate the results
Mindex=1; 
successful_comb = {};
successful_cases = [];
i_succ = 0;
flag_gamma = 1;

% Now execute the optimizations
for index=start_sim:end_sim 
    
% Speed and torque limits
    max_speed=4;
    min_speed=-4;
    max_torque=6;
    min_torque=-6;
    
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

        T_s = [ inf T/N inf ];
%         cases = combvec(T_s,T_s,T_s);

        n_step_h = 9;
        fc_per_tol = 2.5; % N
        tau_per_tol = 0.25; % Nm
        
        % set trial coefficients for Baumgarte's stabilization
        zita = 1;        % damping ratio
        
        gamma_x = 3/T_s(1);   % natural frequency
        gamma_y = 3/T_s(2);
        gamma_alpha = 3/T_s(3);

%         gamma_x = 3/cases(1,i_comb);   % natural frequency
%         gamma_y = 3/cases(2,i_comb);
%         gamma_alpha = 3/cases(3,i_comb);

        % solve the ocp
        damping = 0.5;%0.3;
        damping_L = 0.01;
%         N_trans = 9*N/10;
        N_trans = N-2;
        
        a_r = 0.06;  % TODO: conservative epsilon
        a_f = 0.06;
        
        mu = 0.7;   % friction coefficient
                      
        optimal_Kel_walk_NLproblem_MG_exp_new_foot_shape

        status=solver.stats.return_status;
        
        for ind_gamma = 1:3
            if not(strcmp(status,'Solve_Succeeded') )        
                gamma_y = gamma_y - gamma_y/6;
                
                gamma = gamma_y;

                optimal_Kel_walk_NLproblem_MG_exp_new_foot_shape

                status=solver.stats.return_status;
            end
        end

        save_res
        
%         save_tau_taug
%         input(' ')
        save_taug_taumc
        
        if strcmp(status,'Solve_Succeeded') || strcmp(status,'Solved_To_Acceptable_Level')
            
            % check on drift
            if max(abs(inv_err_pos_x)) < 5e-3 && max(abs(inv_err_pos_y)) < 5e-3 && max(abs(inv_err_pos_gamma)) < 1.e-2
                i_succ = i_succ+1;            
    %             successful_comb{i_succ,1} = [gamma_x, gamma_y, gamma_alpha];
                successful_case_vec = [index];
                successful_case_vec = [successful_case_vec; full(val_opt)];
                successful_case_vec = [successful_case_vec; walk_dist/T];
                successful_case_vec = [successful_case_vec; h_foot];
%                 successful_case_vec = [successful_case_vec; walk_dist];
                
                successful_cases = [successful_cases successful_case_vec];

                forces_post_processing
            end % check on drift

        end % if solve succeeded
        
%     end %  if T and h_foot
% 
plot_step_shape;
saveas(gcf,[name_sim_folder,'/StepShape.fig'])


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

name_tot = [name1,'succ_cases_',num2str(end_sim)];
save(name_tot,'successful_cases')

name_whole=[name1,'Whole_',num2str(start_sim),'_',num2str(end_sim)];
save(name_whole,'mat_ris','min_speed','max_speed','min_torque','max_torque');

return
end
