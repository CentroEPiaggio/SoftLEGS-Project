global N_step toSkip
% N_step=2; % Number of steps that you want to reproduce
Ts=0.004;

N=30;
num=0;
index_sup=0;
runwalk='walk';
val = 60;
VAL=num2str(val);

% Display the list of availabe datasets and choose the one
cd('..\..\From_dataset_to_PCs_and_CMs\dataset');
wholeStruc = dir('Whole_*.mat');
disp([10,'Available datasets:']);
disp('---------------------------------');
for s=1:length(wholeStruc)
   disp([num2str(s),') ',wholeStruc(s).name]);
end
disp('---------------------------------');
item = input('Choose one data: ','s');
itemN = str2num(item);
disp('');

% Load the Whole_X_xxx data 
load(wholeStruc(s).name);
disp(['LOADING ''',wholeStruc(itemN).name,'''']);
% Load also the succ_cases data
load(strcat('succ_cases',wholeStruc(itemN).name(8:end)));
disp(['LOADING ''',strcat('succ_cases',wholeStruc(itemN).name(8:end)),'''',10]);

cd('..\..\NO_ZMP_dataset_from_matlab_to_cpp\')
addpath('Analysis_files');
%%
index_sup=0;
for i=1:size(successful_cases,2)
 
    h_foot=mat_ris{successful_cases(1,i),5}(2);
    
%     if ~(abs(h_foot-0.03)<eps)
%         continue
%     end

    res_theta=mat_ris{successful_cases(1,i),10};
    res_q=mat_ris{successful_cases(1,i),6};
    res_ts=mat_ris{successful_cases(1,i),9};
    T=mat_ris{successful_cases(1,i),5}(3);
    walk_dist=mat_ris{successful_cases(1,i),5}(1);
    h_foot=mat_ris{successful_cases(1,i),5}(2);
    K=mat_ris{successful_cases(1,i), 2}(:,1);
    

    % folder to put results
    new_folder=['VIS\To_Vis_St_',VAL,'_Sp_',num2str(walk_dist/T*100),'_Fh_',num2str(h_foot*1000),'/'];

    % dispaly figures
    display_step_step=0;
    display_foot_orientation=0;
    % break
    % generate interpolated data
    [New_PQ,New_TH,New_time]=Generate_interpolated_data(N_step,res_q,res_theta,res_ts,T,N,Ts,display_step_step,display_foot_orientation);

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

save('ALL_VIS','VIS_SIMS');

% Generate the .h file
Generate_Optimal_dataset_h(VIS_SIMS,'Optimal_Dataset_NO.h',toSkip);

cd('..');
disp('Finished')