%Here we create a reduced dataset in order keep safe the raw data

% struct returned at the end of the process
MAT_SIMS={};

N=30; %trajectory length 
N_step=2; % Number of steps that you want to reproduce
NumSamples=2*N;
num=0;
index_sup=0;

runwalk='walk';

if data_set==1
    STR_INIT=['Original_']; %for SVD analysis

    % Display the list of availabe datasets and choose the one
    cd('../dataset');
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
    
    cd('..')

    folder_res=['Analysis_Results/'];
else %else data_set==1
    STR_INIT=['New_']; %for SVD analysis
    
%the structure successfull cases is a little different, here I adjust it
MAT1=mat_ris;
succ1={};
succ2={};
sapp={};
inds=1;
for i=1:size(MAT1,1)
    if strcmp(MAT1{i,3},'Solve_Succeeded')
       sapp= [MAT1{i,1};MAT1{i,5}'];
       succ1{1,inds}=MAT1{i,1};
       succ1{2,inds}=MAT1{i,5}(1); %walk dist
       succ1{3,inds}=MAT1{i,5}(2); %h_foot
       succ1{4,inds}=MAT1{i,5}(3); %T
       
        inds=inds+1;
    end
    
end

successful_cases=cell2mat(succ1);

end %end if data_set==1

%%

index_sup=0;
for i=1:size(successful_cases,2)

    
    res_theta=mat_ris{successful_cases(1,i),10};
    res_q=mat_ris{successful_cases(1,i),6};
    res_ts=mat_ris{successful_cases(1,i),9};
    T=mat_ris{successful_cases(1,i),5}(3);
    walk_dist=mat_ris{successful_cases(1,i),5}(1);
    h_foot=mat_ris{successful_cases(1,i),5}(2);
    K=mat_ris{successful_cases(1,i), 2}(:,1);
    

% folder to put results
new_folder=[folder_res,'PC_Results/'];

% dispaly figures
display_step_step=0;
display_foot_orientation=0;

N=30;
Ts=T/N;

% generate interpolated data
[New_PQ,New_TH,New_time]=Generate_interpolated_data(N_step,res_q,res_theta,res_ts,T,N,Ts,display_step_step,display_foot_orientation);

index_sup=index_sup+1;

MAT_SIMS{1,index_sup}=New_PQ(:,4:9)';
MAT_SIMS{2,index_sup}=[T,walk_dist,h_foot];
MAT_SIMS{3,index_sup}={'T','walk_dist','h_foot'};
MAT_SIMS{4,index_sup}=New_TH';
MAT_SIMS{5,index_sup}=New_time;


end % for index i

str_walk=[STR_INIT,'MAT_SIMS_THETA_walk'];
save(str_walk,'MAT_SIMS');


