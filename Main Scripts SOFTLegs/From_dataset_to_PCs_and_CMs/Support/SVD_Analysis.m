function [U_all,D_all,V_all,U_mu_all,D_mu_all,V_mu_all]=SVD_Analysis(MAT_SIMS,flag_display_figures,NumSamples,Q_or_TH,STR_INIT)
% Params

if not((strcmp(Q_or_TH,'Q') || strcmp(Q_or_TH,'TH')))
    disp('Q_or_TH not correct');
    return
end    
    
% NumSamples = 60;
FirstSimulationstoCut = 1;
FinalSimulationstoCut = max(size(MAT_SIMS));
NumSignals = 6;

Num_Sims=max(size(MAT_SIMS));

% note that raws represent the continuous variable (e.g. time)

joint_data = {'hl','kl','al','hr','kr','ar'};

Xhl = [];
Xkl = [];
Xal = [];
Xhr = [];
Xkr = [];
Xar = [];


vect_sims=1:max(size(MAT_SIMS));
flag_save=0;

% Q_or_TH indicates on which signals you want to apply the SVD
% Q Link side
% TH Motor side

if strcmp(Q_or_TH,'Q') 
    qth=1;
else
    qth=4;
end
    
for i_sims=1:max(size(MAT_SIMS))
    if i_sims < FirstSimulationstoCut
    else
        if i_sims > FinalSimulationstoCut
        else
            Xhl = [Xhl MAT_SIMS{qth,i_sims}(1,1:NumSamples).'];
            Xkl = [Xkl MAT_SIMS{qth,i_sims}(2,1:NumSamples).'];
            Xal = [Xal MAT_SIMS{qth,i_sims}(3,1:NumSamples).'];
            
            Xhr = [Xhr MAT_SIMS{qth,i_sims}(4,1:NumSamples).'];
            Xkr = [Xkr MAT_SIMS{qth,i_sims}(5,1:NumSamples).'];
            Xar = [Xar MAT_SIMS{qth,i_sims}(6,1:NumSamples).'];
        end
    end
end
%% Viualize data
Visualize_trajectories;

%% Data SVD

econ = 'econ';
for i_joints=1:NumSignals %for each joint
    
    X = eval(strcat('X',joint_data{i_joints}));
    
    %mean evaluation
    eval(strcat('mu',joint_data{i_joints},'= mean(X,1);'));
    
    %mean subtraction evaluation
    eval(strcat('Xmu',joint_data{i_joints},'= bsxfun(@minus,X,mu',joint_data{i_joints},');'));
    
    %SVD
    eval(strcat('[U',joint_data{i_joints},...
    ',D',joint_data{i_joints},...
    ',V',joint_data{i_joints},'] = svd(Xmu',joint_data{i_joints},',econ);'));

    %SVD_mu
    eval(strcat('[Umu',joint_data{i_joints},...
    ',Dmu',joint_data{i_joints},...
    ',Vmu',joint_data{i_joints},'] = svd(mu',joint_data{i_joints},',econ);'));
end

%% Here we perform svd on all joints

Xmu_all = [ Xmuhl; Xmukl; Xmual; Xmuhr; Xmukr; Xmuar];
mu_all = [ muhl; mukl; mual; muhr; mukr; muar];
%SVD
[U_all, D_all ,V_all] = svd(Xmu_all,econ);
[U_mu_all, D_mu_all ,V_mu_all] = svd(mu_all,econ);

%% SVD Goodness Evaluation -- Movements
figure('Name','SVD Goodness Evaluation -- Trajectories','visible',flag_display_figures);
expl_traj = diag(D_all).^2/sum(diag(D_all).^2); 
n_exp = length(expl_traj);
par = 100*expl_traj(1);
% Check if the length is less of ten or not to adapt the bar
if(n_exp < 10)   
    bar(100*expl_traj);
    idx_par = n_exp;
    axis([0.5 (n_exp+0.5) 0 100]); 
else
    bar(100*expl_traj(1:10)); 
    idx_par = 10;
    axis([0.5 10.5 0 100]);
end
grid on
for i = 2 : idx_par
    par = [par par(i-1)+100*expl_traj(i)];
end
yticks(0:10:100)
hold on, plot(1:idx_par,par,'*-')
set(gca,'FontSize',30)
ylabel('%')
xlabel('PC Number')

if flag_display_figures
    mkdir([STR_INIT,'Figures'])
    name='SVD Goodness Evaluation -- Movements';
    savefig(gcf,[STR_INIT,'Figures/',Q_or_TH,'_',name]);
end
title('Explained Variance Trajectories')
    
%% SVD Goodness Evaluation -- Mean values
figure('Name','SVD Goodness Evaluation -- Poses','visible',flag_display_figures);
expl_poses = diag(D_mu_all).^2/sum(diag(D_mu_all).^2); 
bar(100*expl_poses);
axis([0.5 6.5 0 100])
grid on
par = 100*expl_poses(1);
for i = 2 : 6
    par = [par par(i-1)+100*expl_poses(i)];
end
hold on, plot(1:6,par,'*-')
set(gca,'FontSize',30)
ylabel('%')
xlabel('PC Number')

if flag_display_figures
    mkdir([STR_INIT,'Figures'])
    name='SVD Goodness Evaluation -- Mean values';
    savefig(gcf,[STR_INIT,'Figures/',Q_or_TH,'_',name]);
end
title('Explained Mean Values')

%% Plots synergies vs parameters

% Here we organize simulations as a function of the walk parameters
param_trend

% In particular it returns structures as in the following:
% Velal Velar Velhl Velkl Velkr are all the velocities
% Hal Har Hhl Hkl Hkr Hkl there all the vfoot height
% Lal Lar Lhl Lkl Lkr Lhl there all the Step Length

% Here we visualize trajectory and pose dependence on walk parameters
Visualization_PCs_VS_Task_Parameters


end