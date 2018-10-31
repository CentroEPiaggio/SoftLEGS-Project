%% Generate the trajectory from velocity and foot high to test it in Robotran
function runTraj(SLeg_Path,v,fh,fl,setName,Robotran_Simul_Path)
    addpath('From_dataset_to_PCs_and_CMs');
    load(strcat('.\Final_Maps\',setName)); %
    
    % PCs
    PCs = set_SVD_TH{1};
    % mus
    mus = set_SVD_TH{4};
    
    % Trajectory derived from the PC maps
    TH = Original_From_CM_To_Traj_TP(v,fh,fl,set_SVD_TH{1},set_SVD_TH{4});
    
    % Compute the sample time [30 sample per gait]
    t = abs(fl/v);                  %   gait time 
    Ts = t/30;                      %   sample time
    grav_y = 0;
    grav_z = -9.81;
    
    % Compute derivatives
    dTH = gradient(TH, Ts);     % vel
    ddTH = gradient(dTH, Ts);   % acc
    
    % Save
    save(strcat(SLeg_Path,'\Robotran_Traj\Trajectory'),'TH','dTH','grav_y','grav_z','ddTH','Ts','v','fh','fl');
    % Save also to the Robotran folder used to simulate the results
    save(strcat(Robotran_Simul_Path,'workR\Trajectory'),'TH','dTH','grav_y','grav_z','ddTH','Ts','v','fh','fl');
    
    rmpath('From_dataset_to_PCs_and_CMs');
end