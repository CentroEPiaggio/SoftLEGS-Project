% Here we organize simulations as function of the walk parameters

NumSignals = 6;
FirstSimulationstoCut = 1;
FinalSimulationstoCut = max(size(MAT_SIMS));

% Joint name
joint_data = {'hl','kl','al','hr','kr','ar'};

for i_joints = 1:NumSignals
    i_sims1 = 0;
    for i_sims = 1:length(vect_sims)
        if vect_sims(i_sims) < FirstSimulationstoCut
        else
            if vect_sims(i_sims) > FinalSimulationstoCut
            else
                
                i_sims1 = i_sims1+1;
                eval(strcat('Vel',joint_data{i_joints},'(i_sims1)=MAT_SIMS{2,vect_sims(i_sims)}(2)/MAT_SIMS{2,vect_sims(i_sims)}(1);'));
                eval(strcat('H',joint_data{i_joints},'(i_sims1)=MAT_SIMS{2,vect_sims(i_sims)}(3);'));
                eval(strcat('L',joint_data{i_joints},'(i_sims1)=MAT_SIMS{2,vect_sims(i_sims)}(2);'));

            end
        end
    end
end
