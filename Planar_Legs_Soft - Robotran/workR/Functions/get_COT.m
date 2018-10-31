% Extract data from optimization results (mat_ris)

% Directory of results from which get the CoT
dirName = 'G_Walk_Soft_K_60_Final_Pos\';

% Load results and successes index vector
cd(dirName);
files = dir('*.mat');
for i=1:length(files)
    load(files(i).name)
end
cd('..');

% Retrieve CoT vector
CoTs = cell2mat(mat_ris(:,4));
walkData = cell2mat(mat_ris(:,5));
succ_idx = successful_cases(1,:);

idx = 1;
CoT_res = [];
speed = [];
step_h = [];
step_l = [];
for i=1:length(succ_idx)
    CoT_res(idx) = CoTs(succ_idx(i));
    speed(idx) = walkData(succ_idx(i),1)/walkData(succ_idx(i),3);
    step_h(idx) = walkData(succ_idx(i),2);
    step_l(idx) = walkData(succ_idx(i),1);
    idx = idx + 1;
end

% Overall matrix with results [speed, length, f_height, CoT]
resMat = [speed' step_l' step_h' CoT_res'];
    
% % Plots of the results to fitting purposes
% figure
% plot3(resMat(:,1),resMat(:,2),resMat(:,4),'*');
% xlabel('speed');
% ylabel('step_l');
% zlabel('minCoT');
% 
% figure
% plot(resMat(:,1),resMat(:,4),'*');
% xlabel('speed');
% ylabel('minCoT');
% 
% figure
% plot(resMat(:,2),resMat(:,4),'*');
% xlabel('step_l');
% ylabel('minCoT');





