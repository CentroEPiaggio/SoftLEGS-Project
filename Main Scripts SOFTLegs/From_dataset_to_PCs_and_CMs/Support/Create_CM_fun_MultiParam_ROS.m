clc

global N_PCs polyDegree

STR = {'Traj_','Pose_'};
PC_Str = {'First_','Second_','Third_','Fourth_','Fifth_','Sixth_'};
ALL_FUN_NAME = {};

fileID = fopen(['../../ROS/synergy_map_fun.h'],'w');

% Function header
fprintf(fileID,'/* ------------------------------------------------------------------------- \n');
fprintf(fileID,['\tHEADER FUNCTION FOR MAPs OF THE SOFTLegs PROJECT \n']);
fprintf(fileID,'\tMap function obtained with MultiPolyRegress function');
fprintf(fileID,'\n');
fprintf(fileID,['\tGenerated automatically the ',datestr(datetime)],'\n');
fprintf(fileID,'\n------------------------------------------------------------------------- */\n');
fprintf(fileID,['#include<iterator> \n']);
% name of the function
fprintf(fileID,['Eigen::MatrixXd From_Syn_To_Traj(double speed,double foot_h, double foot_l,Eigen::MatrixXd PCs_Synergy, Eigen::MatrixXd Mus_Synergy){ \n']);
fprintf(fileID,'\n');

% variables
fprintf(fileID,'// Size variables');
fprintf(fileID,'\n');
fprintf(fileID,['int r_PCs = ',num2str(size(set_SVD_TH{1}, 1)),';\n']);
fprintf(fileID,['int c_PCs = ',num2str(size(set_SVD_TH{1}, 2)),';\n']);
fprintf(fileID,['int r_Mus = ',num2str(size(set_SVD_TH{4}, 1)),';\n']);
fprintf(fileID,['int c_Mus = ',num2str(size(set_SVD_TH{4}, 2)),';\n']);
fprintf(fileID,'\n');

% Strings to get PCs and mus
fprintf(fileID,'// Get principal components and mus from matrices ');
fprintf(fileID,'\n');
% define dimensions of matrices
for i_syn=1:N_PCs
    fprintf(fileID, ['Eigen::MatrixXd ', PC_Str{i_syn},'PC(r_PCs,1); \n']);
    fprintf(fileID, ['Eigen::MatrixXd ', PC_Str{i_syn},'mu(r_Mus,1); \n']);
end
% PCs
fprintf(fileID,'// PCs');
fprintf(fileID,'\n');
fprintf(fileID, 'for(int i=0;i<r_PCs;i++){ \n');
for i_syn=1:N_PCs
    fprintf(fileID, ['\t', PC_Str{i_syn},'PC(i,1) = PCs_Synergy(i,',num2str(i_syn),'); \n']);    
end
fprintf(fileID,'}\n');
% mus
fprintf(fileID,'// Mus');
fprintf(fileID,'\n');
fprintf(fileID, 'for(int i=0;i<r_Mus;i++){ \n');
for i_syn=1:N_PCs
    fprintf(fileID, ['\t', PC_Str{i_syn},'mu(i,1) = Mus_Synergy(i,',num2str(i_syn),'); \n']);    
end
fprintf(fileID,'}\n');

% Useful symbols and strings
prodString = '*';
sumString ='+';  
STR_tNEW = ' ';
STR_pNEW = ' ';
STR_tO = ' ';
STR_pO = ' ';

for i_syn=1:N_PCs
        % Load data
        load(['Original_N_',num2str(i_syn),'_TP'])
        
        % Pose and Trajectory polynomials
        coef_pose = fitresult_Pose.Coefficients;        % Vector of coefficents values
        poly_pose = fitresult_Pose.Legend;              % Vector of string of polynomial values
        power_pose = fitresult_Pose.PowerMatrix;        % Matrix of pow values
        coef_traj = fitresult_Trajectory.Coefficients;  % Vector of coefficents values
        poly_traj = fitresult_Trajectory.Legend;        % Vector of string of polynomial values
        power_traj = fitresult_Trajectory.PowerMatrix;  % Matrix of pow values
         
        % Number of coefficents (it depends on the polynomial order)
        N_coef_traj = length(fitresult_Trajectory.Coefficients);
        N_coef_pose = length(fitresult_Pose.Coefficients); % should be equal to N_coef_traj
               
        % Compose the strings for the trajectory
        for k=1:N_coef_traj
            % speed
            if (power_traj(k,1) ~= 0) 
                pow_s = power_traj(k,1);
                t1_pow = ['pow(speed,',num2str(pow_s),')'] ;
            else
                t1_pow = '1';
            end
            % foot_h
            if (power_traj(k,2) ~= 0) 
                pow_fh = power_traj(k,2);
                t2_pow = ['pow(foot_h,',num2str(pow_fh),')'] ;
            else
                t2_pow = '1';
            end
            % foot_l
            if (power_traj(k,3) ~= 0) 
                pow_fl = power_traj(k,3);
                t3_pow = ['pow(foot_l,',num2str(pow_fl),')'] ;
            else
                t3_pow = '1';
            end      
            STR_tNEW = [STR_tNEW sumString num2str(coef_traj(k)) prodString t1_pow prodString t2_pow prodString t3_pow];
        end
        clear pow_s pow_fh pow_fl
        
         % Compose the strings for the pose
        for k=1:N_coef_traj
            % speed
            if (power_pose(k,1) ~= 0) 
                pow_s = power_pose(k,1);
                p1_pow = ['pow(speed,',num2str(pow_s),')'] ;
            else
                p1_pow = '1';
            end
            % foot_h
            if (power_pose(k,2) ~= 0) 
                pow_fh = power_pose(k,2);
                p2_pow = ['pow(foot_h,',num2str(pow_fh),')'] ;
            else
                p2_pow = '1';
            end
            % foot_l
            if (power_pose(k,3) ~= 0) 
                pow_fl = power_pose(k,3);
                p3_pow = ['pow(foot_l,',num2str(pow_fl),')'] ;
            else
                p3_pow = '1';
            end      
            STR_pNEW = [STR_pNEW sumString num2str(coef_pose(k)) prodString p1_pow prodString p2_pow prodString p3_pow];
        end
        clear pow_s pow_fh pow_fl
        
        % Obtain polynomial order
        params = size(fitresult_Pose.PowerMatrix,2);
        polyOrder = polyDegree;%round( log(N_coef_traj)/log(params) );
        
        % Trajectory
        fprintf(fileID,strcat('// Order of the polynomial used is:',' ',num2str(polyOrder)));
        fprintf(fileID,'\n');
        fprintf(fileID,strcat('// Principal components n�',num2str(i_syn)));
        fprintf(fileID,'\n');
        traj_PC_str = [STR{1},num2str(i_syn),'_gain'];
        fprintf(fileID,['double ', traj_PC_str]);
        fprintf(fileID,'=');
        fprintf(fileID,strcat(STR_tNEW,';'));
        fprintf(fileID,'\n');
        % Pose
        pose_PC_str = [STR{2},num2str(i_syn),'_gain'];
        fprintf(fileID,['double ',pose_PC_str]);
        fprintf(fileID,'=');
        fprintf(fileID,strcat(STR_pNEW,';'));
        fprintf(fileID,'\n\n');
                
        % Composw the output line, summation of the terms
        % Trajectory
        t_str = strcat(' ',PC_Str{i_syn},'PC');                             % as --> First_PC*Traj_1_gain <---- etc...
        STR_tO = [STR_tO sumString eval('strcat(t_str,prodString,traj_PC_str)')];
        % Pose
        p_str = strcat(' ',PC_Str{i_syn},'mu ');                            % as --> First_mu*Pose_1_gain <---- etc...
        STR_pO = [STR_pO sumString eval('strcat(pose_PC_str,prodString,p_str)'),'(i,1)'];
        
        % Clean the strings
        STR_tNEW = '';
        STR_pNEW = '';        
end

% Output
fprintf(fileID,'// Result of the map');
fprintf(fileID,'\n'); 
fprintf(fileID,'Eigen::MatrixXd out_Traj = '); 
fprintf(fileID,[STR_tO,';']); 
fprintf(fileID,'\n');
fprintf(fileID,'Eigen::MatrixXd New_out_Traj(60,6); \n');
fprintf(fileID,'for(int i=0;i<6;i++){ \n');
fprintf(fileID,'\t for(int j=0;j<60;j++) \n');
fprintf(fileID,'\t \t New_out_Traj(j,i) = out_Traj(i*60+j)');
fprintf(fileID,strcat(STR_pO,';')); 
fprintf(fileID,'\n'); 
fprintf(fileID,'}\n'); 
fprintf(fileID,'return New_out_Traj;')
fprintf(fileID,'\n}');

% ----------------------------------------------------------------------- %
% I should add the fake function in order to do not have errors
fprintf(fileID,'\n'); 
fprintf(fileID,'\n'); 
fprintf(fileID,['Eigen::MatrixXd From_Syn_To_Traj_Interpolated_Sampled_TEST(double speed,double foot_h,Eigen::MatrixXd First_Synergy,Eigen::MatrixXd Second_Synergy,Eigen::MatrixXd First_mu,Eigen::MatrixXd Second_mu, int num, double curr_Ts, double des_Ts){}']);
fprintf(fileID,'\n'); 
fprintf(fileID,'\n'); 
fprintf(fileID,['Eigen::MatrixXd From_Syn_To_Traj_Interpolated_Sampled(double speed,double foot_h,Eigen::MatrixXd First_Synergy,Eigen::MatrixXd Second_Synergy,Eigen::MatrixXd First_mu,Eigen::MatrixXd Second_mu, int num, double curr_Ts, double des_Ts){}']);
fprintf(fileID,'\n'); 
fprintf(fileID,'\n'); 
% ----------------------------------------------------------------------- %

% End of function
fclose(fileID);

% Copy file in the current directory
% copyfile(['../',STR_INIT,'From_CM_To_Traj_TP.m'], pwd)