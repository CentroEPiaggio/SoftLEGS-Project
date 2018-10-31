%% THIS SCRIPT RUNS ALL THE PLANAR_SoftLEGS CODE (PROVIDED BY G.GASPARRI) FROM ONE LOCATION AND COME BACK THERE
% The contents are reorganized by Riccardo Mengacci starting form the orignal work of Gian Maria Gasparri
% @email: riccardo.mengacci@gmail.com
%
%   !!!!!!!!!BEFORE YOU CONTINUE!!!!!!!!!!!
%
% You should have: 
% 1) one folder with the overall Robotran project with its relative path 
%    "Robotran_SLeg_Path" (generally is in \Documents)
% 2) one folder containing the main script and all the optimization
%    and post-processing code from SoftLegs project 
%    (provided by GianMaria Gasparri) with its relative path "SLegPath"

%% First of all we define the overall path of the code
clear all
close all
clc
global Main_Path;
Main_Path = pwd;
SLeg_Path = 'C:\Users\richi\OneDrive\Desktop\SoftLegs PROJECT - CLEAN VERSION\Main Scripts SOFTLegs\';
Robotran_SLeg_Path = 'C:\Users\richi\Documents\MBProjects\Planar_Legs_Soft\';

%% Run the optimization procedure inside the folder "workR" of the
%  Robotran project
% @Param:
%       Walk_Speed; the range of speed that you want to use for the maps
%       Foot_H; the range of step high that you want to use for the maps
% NB: the parameters are passed inside the function
% @Return:
%       Whole_*_*; overall OPT dataset (inside the results folder) 
%       succ_cases; the success cases of the OPT process

global fldName walkS stepH stepL beta useBeta

% Name of the folder to create
fldName = 'G_Walk_Soft_TEST_LAST\';

% Parameters to use for the optimizations
walkS = [0.04,0.5];%[0.04:0.05:0.5]; 
stepL = [0.2,0.05];%[0.01:0.02:0.3];
stepH = [0.01:0.025:0.1];

% 0) Use the above parameters for speed/stepL profile
% 1) Use the beta-shaped region for speed/stepL profile
useBeta = 1; 
beta = 0.34;%[0.15:0.05:0.56]; 

%run
% runOpt_BETA(Robotran_SLeg_Path); 
runOpt(Robotran_SLeg_Path);
cd(Main_Path);

%% 
%   NB: Copy the OPT data into the dataset folder in "From_dataset_to_PCs_and_CMs"

%% Run the map generation procedure inside the folder "From_dataset_to_PCs_and_CMs" of the
%  SoftLegs project
% @Param:
%       Whole_*_* (line 19); all the data saved from the optimizations 
%       succ_cases (line 20); the success cases to be used
% NB: the parameters are loaded inside the function named "Process_Signals_Pre_PCA"
% @Return:
%       set_SVD_TH; contains a struct with the same name that have first and second PC and mu
%       *_MAT_SIMS_THETA_*; contains MAT_SIMS a struct with all interpolated simulations data
%       First_and_Second_PCs_and_mus; Matlab data of the first and second PC and mu
% 		Original_From_CM_To_Traj; matlab function to generate trajectories from CMs
global N_PCs polyDegree
% Set the number of principal component to be used
N_PCs = 3; 
% Set the order of the polynomial curve for fitting the maps
polyDegree = 3;

%run
% runMaps_BETA(SLeg_Path); 
runMaps(SLeg_Path);
cd('..');

%% 
%   NB: Copy the file set_SVD_TH (conteining the CMs) into the Final_Maps as backup

%% Translate the dataset from Matlab to ROS code (Cpp/h) [FOR NOW ONLY THE OPT WALK!!]
% @Param:
%       Whole_*_* (line 22); all the data saved from the optimizations 
%       succ_cases (line 23); the success cases to be used
% NB: the parameters are loaded inside the function named "Create_dataset_NO_dataset"
% @Return:
%       Optimal_Dataset_NO; header file to use the OPT walk with the SoftLegs GUI

global N_Step toSkip

% Set numbers of step ro replicate
N_Step = 2; 
% and the trajectories to skip (in case you have a huge database, otherwise
% the .h file will be to big)
toSkip = 10;

%run
runConv(SLeg_Path); 
cd(Main_Path);

%% Generate the motor trajectories for simulations (Robotran)
% @Param:
%       v; forward velocity desired
%       fh; foot high of the step
%       set_SVD_TH; contains a struct with the same name that have first and second PC and mu
% NB: the parameters are loaded automatically
% @Return:
%       Trajectory; the motor trajectory for simulation (pos, vel, acc and Ts)

% Define the velocity and the foot high to be used
v = 0.1;
fh = 0.01;

% Additionals
fl = 0.15;
% bet = 0.56;
% slope = 0.1;
% Define the name of the set to load
setName = 'set_SVD_TH';
% Path of the folder used to simulate the results
Robotran_Simul_Path = 'C:\Users\richi\Documents\MBProjects\Camminata\';

% NOTE: Make sure to pass the right parameter as third params!!!
%run
runTraj(SLeg_Path,v,fh,fl,setName,Robotran_Simul_Path); 
% runTraj_GRAVITY(SLeg_Path,v,fh,fl,slope,setName);
cd(Main_Path);
