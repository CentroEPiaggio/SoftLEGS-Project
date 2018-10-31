N_step=2; % Number of steps that you want to reproduce
Ts=0.004;

N=29;

num=0;
% index_sup=0;

% val=10;
val=60;

VAL=num2str(val);
% CASES=num2str(cases);

% folder_res=['../new_data/G_Large_Dataset_Run/'];
% folder_res=['../new_data/G_Large_Dataset_Run_morefly/'];
% folder_res=['../new_data/ZMP_NO_Results/'];
folder_res=['../ZMP_res/'];

% load([folder_res,'Whole_1_',CASES,'.mat'])
% load([folder_res,'succ_cases.mat'])

% load([folder_res,'JMAT_ZMP_NO.mat'])
load([folder_res,'FMIN_RES_ZMP_NO.mat'])

%%
index_sup=0;
for i=1:size(FMIN_RES,1)
    
    h_foot=FMIN_RES{i,13};
    
    if h_foot<=0.025
        continue
    end
    
%     str=['../Sim_folder_walk/Subfolder_sim_walk_',num2str(i),'/sim_walk_',num2str(i)];
%     load(str);
    res_q__=FMIN_RES{i,2}';
    res_q_=res_q__(:,1:end-1); %6xN
    res_q_F=[flipud(res_q_(1:3,:));res_q_(4:6,:)]; %res_q considering the
% floating base placed on the hip
    res_theta__=FMIN_RES{i,5}';
    res_theta_=res_theta__(:,1:end-1); %6xN
    res_theta_F=[flipud(res_theta_(1:3,:));res_theta_(4:6,:)];
    time=FMIN_RES{i,9};
    T=FMIN_RES{i,17};
    walk_dist=FMIN_RES{i,16};
    h_foot=FMIN_RES{i,13};
    
%     res_ts=mat_ris{successful_cases(1,i),9};
%     T=mat_ris{successful_cases(1,i),5}(3);
%     walk_dist=mat_ris{successful_cases(1,i),5}(1);
%     h_foot=mat_ris{successful_cases(1,i),5}(2);
    K=60*ones(6,1);
    
% return
% end
%%
% folder to put results
new_folder=[folder_res,'To_Vis_St_',VAL,'_Sp_',num2str(walk_dist/T*100),'_Fh_',num2str(h_foot*1000),'/'];

% dispaly figures
display_step_step=0;
display_foot_orientation=0;

% break
% generate interpolated data
% [New_PQ,New_TH,New_time]=Generate_interpolated_data(N_step,res_q,res_theta,res_ts,T,N,Ts,display_step_step,display_foot_orientation);
[New_PQ,New_TH,New_time]=Generate_interpolated_data_ZMP(N_step,res_q_F,res_theta_F,time,Ts,display_step_step,display_foot_orientation);
% run('try_plot.m');
% try_plot
index_sup=index_sup+1;

MAT_SIMS{1,index_sup}=New_PQ;
MAT_SIMS{2,index_sup}=[T,walk_dist,h_foot];
MAT_SIMS{3,index_sup}={'T','walk_dist','h_foot'};

VIS_SIMS{1,index_sup}=new_folder;
VIS_SIMS{2,index_sup}=[T,walk_dist,h_foot];
VIS_SIMS{3,index_sup}={'T','walk_dist','h_foot'};
VIS_SIMS{4,index_sup}=K';
VIS_SIMS{5,index_sup}=[walk_dist/T,h_foot];
VIS_SIMS{6,index_sup}=New_PQ';
VIS_SIMS{7,index_sup}=New_TH';
VIS_SIMS{8,index_sup}=New_time;

mkdir(new_folder);
strm=[new_folder,'VIS'];
save(strm,'New_PQ','New_TH','New_time','T','walk_dist','h_foot','K')

end % for index i

%%

save([folder_res,'ALL_VIS'],'VIS_SIMS');

% SoftLegs_Gui_path=['/home/gian/catkin_ws/src/Qb_Legs_Synergies/zmp_walk_gui/include/zmp_walk_gui/'];
SoftLegs_Gui_path=['../'];
Generate_Optimal_dataset_h(VIS_SIMS,'Optimal_Dataset_ZMP.h',SoftLegs_Gui_path);