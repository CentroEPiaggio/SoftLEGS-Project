% In this file we create the set of principal components (PCs) of the optimal dataset
% obtained by NO. Then we identify a set o mapping functions (CMs) which relate the 
% weight of the components to the stride parameters.

warning off
close all
clearvars -except flag_save 
clc
addpath('Support')
Speed_cut = [];
H_Step_cut = [];
L_Step_cut = [];
MAT_SIMS_cut = {};

tic

flag_display_figures='on';

if exist('flag_save')
else data_set=1; %old data set else new data set to validate
    
for data_set=1:1
    
    run('Support/Process_Signals_Pre_PCA');

    load([str_walk])
    Q_or_TH='TH';

    % Funtion useful to shape the region, e.g. by the beta shape, before
    % perform the SVD analysis and maps generation
    % UNCOMMENT if you want to shape the region
%     MAT_SIMS_cut = shapeRegion(MAT_SIMS);
%     MAT_SIMS = MAT_SIMS_cut; 

    % Function used to perform maps with the step height parameter fixed
    % UNCOMMENT if you want to fix the parameter
    MAT_SIMS_cut = fixedStepH(MAT_SIMS);
    MAT_SIMS = MAT_SIMS_cut; 
    
    str_walk=[STR_INIT,'MAT_SIMS_cutted_walk'];
    save(str_walk,'MAT_SIMS');
    
    %identify PCs
    [U_all_TH,D_all_TH,V_all_TH,U_mu_all_TH,D_mu_all_TH,V_mu_all_TH]=SVD_Analysis(MAT_SIMS,flag_display_figures,NumSamples,Q_or_TH,STR_INIT);

    % save results
    set_SVD_TH={U_all_TH,D_all_TH,V_all_TH,U_mu_all_TH,D_mu_all_TH,V_mu_all_TH};

%     save('set_SVD_TH','set_SVD_TH','successful_cases','Q_or_TH')
%     save('.\Support\set_SVD_TH','set_SVD_TH','successful_cases','Q_or_TH')
    save('..\Final_Maps\set_SVD_TH','set_SVD_TH','successful_cases','Q_or_TH')

    flag_save=1;

        if data_set==1
            % here we create the Component Mapping Functions CMs
        %     run('Support/Create_CM_fun');
            %% EDITED By RICCARDO MENGACCI TO ADD THE THIRD PARAMETER FOR FITTING
            run('Support/Create_CM_fun_MultiParam');
        end

    end
end
%%
%Finally we validate the CM with a new dataset
% run('Support/CM_Validation'); 
run('Support/CM_Validation_MultiParam'); 
toc