%% Run the translation procedure from Matlab to Cpp/h
function runConv(SLeg_Path)
    cd(strcat(SLeg_Path,'NO_ZMP_dataset_from_matlab_to_cpp'));
    % run
    Generate_NO_Optimal_Dataset_SoftLegs_GUI;
   %Generate_ZMP_Dataset_SoftLegs_GUI; [DATA NOT CONSISTENT!]
end