clc

fileID = fopen(['../../ROS/synergy_values.h'],'w');

% Function header
fprintf(fileID,'/* ------------------------------------------------------------------------- \n');
fprintf(fileID,['\tHEADER FUNCTION FOR ENCODE THE DATASET OF THE SOFTLegs PROJECT \n']);
fprintf(fileID,'\tMap function obtained with MultiPolyRegress function');
fprintf(fileID,'\n');
fprintf(fileID,['\tGenerated automatically the ',datestr(datetime)],'\n');
fprintf(fileID,'\n------------------------------------------------------------------------- */\n');

% variables
fprintf(fileID,'// Size variables');
fprintf(fileID,'\n');
fprintf(fileID,['int r_PCs = ',num2str(size(set_SVD_TH{1}, 1)),';\n']);
fprintf(fileID,['int c_PCs = ',num2str(size(set_SVD_TH{1}, 2)),';\n']);
fprintf(fileID,['int r_Mus = ',num2str(size(set_SVD_TH{4}, 1)),';\n']);
fprintf(fileID,['int c_Mus = ',num2str(size(set_SVD_TH{4}, 2)),';\n']);
fprintf(fileID,'\n');

% matrices definitions
fprintf(fileID,'// Differently from what Gian Maria did before, I store all the PCs \n');
fprintf(fileID,'Eigen::MatrixXd PCs(r_PCs,c_PCs); \n');
fprintf(fileID,'Eigen::MatrixXd Mus(r_Mus,c_Mus); \n');

% function body
fprintf(fileID,['int initialize_synergies(){ \n']);
fprintf(fileID,'\n');

% PCs
PCs_m = set_SVD_TH{1};
for i=1:size(set_SVD_TH{1}, 1)
    for j=1:size(set_SVD_TH{1}, 2)
        fprintf(fileID, ['PCs(',num2str(i-1),',',num2str(j-1),') =',num2str(PCs_m(i,j)),'\n']); 
    end   
end

fprintf(fileID,'\n');

% Mus
Mus_m = set_SVD_TH{4};
for i=1:size(set_SVD_TH{4}, 1)
    for j=1:size(set_SVD_TH{4}, 2)
        fprintf(fileID, ['Mus(',num2str(i-1),',',num2str(j-1),') =',num2str(Mus_m(i,j)),'\n']); 
    end   
end


fprintf(fileID,'std::cout<<"Synergies loaded"<<std::endl;\n');
fprintf(fileID,'return 1; \n');
fprintf(fileID,'\n}');

% End of function
fclose(fileID);

% Copy file in the current directory
% copyfile(['../',STR_INIT,'From_CM_To_Traj_TP.m'], pwd)