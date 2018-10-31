
clc

global N_PCs polyDegree

STR = {'Traj_','Pose_'};
PC_Str = {'First_','Second_','Third_','Fourth_','Fifth_','Sixth_'};
ALL_FUN_NAME = {};

fileID = fopen(['../',STR_INIT,'From_CM_To_Traj_TP.m'],'w');

% Function header
fprintf(fileID,['function out = ',STR_INIT,'From_CM_To_Traj_TP(x1,x2,x3,PCs,mus)']);
fprintf(fileID,'\n');
fprintf(fileID,'%% ------------------------------------------------------------------------- \n');
fprintf(fileID,'%% Map function obtained with MultiPolyRegress function');
fprintf(fileID,'\n');
fprintf(fileID,['%% Generated automatically the ',datestr(datetime)],'\n');
fprintf(fileID,'\n');
fprintf(fileID,'\n');

% Strings to get PCs and mus
fprintf(fileID,'%% Get principal components and mus');
fprintf(fileID,'\n');
fprintf(fileID,'%% PCs');
fprintf(fileID,'\n');
for i_syn=1:N_PCs
    fprintf(fileID, [PC_Str{i_syn},'PC = PCs(:,',num2str(i_syn),');']);
    fprintf(fileID,'\n');
end
fprintf(fileID,'%% mus');
fprintf(fileID,'\n');
for i_syn=1:N_PCs
    fprintf(fileID, [PC_Str{i_syn},'mu = mus(:,',num2str(i_syn),');']);
    fprintf(fileID,'\n');
end
fprintf(fileID,'\n');

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
        coef_traj = fitresult_Trajectory.Coefficients;  % Vector of coefficents values
        poly_traj = fitresult_Trajectory.Legend;        % Vector of string of polynomial values
        
        % Number of coefficents (it depends on the polynomial order)
        N_coef_traj = length(fitresult_Trajectory.Coefficients);
        N_coef_pose = length(fitresult_Pose.Coefficients); % should be equal to N_coef_traj
               
        % Compose the strings
        for k=1:N_coef_traj
            STR_tNEW = [STR_tNEW sumString eval('strcat(num2str(coef_traj(k,:)),prodString,poly_traj(k,:))')];
            STR_pNEW = [STR_pNEW sumString eval('strcat(num2str(coef_pose(k,:)),prodString,poly_pose(k,:))')];
        end        
         
        % Obtain polynomial order
        params = size(fitresult_Pose.PowerMatrix,2);
        polyOrder = polyDegree;%round( log(N_coef_traj)/log(params) );
        
        % Trajectory
        fprintf(fileID,strcat('%% Order of the polynomial used is:',' ',num2str(polyOrder)));
        fprintf(fileID,'\n');
        fprintf(fileID,strcat('%% Principal components n°',num2str(i_syn)));
        fprintf(fileID,'\n');
        traj_PC_str = strcat(STR{1},num2str(i_syn),'_gain');
        fprintf(fileID,traj_PC_str);
        fprintf(fileID,'=');
        fprintf(fileID,strcat(STR_tNEW,';'));
        fprintf(fileID,'\n');
        % Pose
        pose_PC_str = strcat(STR{2},num2str(i_syn),'_gain');
        fprintf(fileID,pose_PC_str);
        fprintf(fileID,'=');
        fprintf(fileID,strcat(STR_pNEW,';'));
        fprintf(fileID,'\n\n');
                
        % Composw the output line, summation of the terms
        % Trajectory
        t_str = strcat(' ',PC_Str{i_syn},'PC');                             % as --> First_PC*Traj_1_gain <---- etc...
        STR_tO = [STR_tO sumString eval('strcat(t_str,prodString,traj_PC_str)')];
        % Pose
        p_str = strcat(' ',PC_Str{i_syn},'mu ');                            % as --> First_mu*Pose_1_gain <---- etc...
        STR_pO = [STR_pO sumString eval('strcat(p_str,prodString,pose_PC_str)')];
        
        % Clean the strings
        STR_tNEW = '';
        STR_pNEW = '';        
end

% Output
fprintf(fileID,'%% Result of the map');
fprintf(fileID,'\n'); 
fprintf(fileID,'out = transpose(reshape('); 
fprintf(fileID,strcat(STR_tO,',60,6)) ...')); 
fprintf(fileID,'\n \t'); 
fprintf(fileID,strcat(STR_pO,';')); 
fprintf(fileID,'\n'); 
fprintf(fileID,'\nend'); 

% End of function
fclose(fileID);

% Copy file in the current directory
% copyfile(['../',STR_INIT,'From_CM_To_Traj_TP.m'], pwd)