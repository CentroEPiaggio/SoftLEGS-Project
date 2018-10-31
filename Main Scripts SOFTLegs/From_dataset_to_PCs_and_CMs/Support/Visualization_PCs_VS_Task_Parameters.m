% Here we visualize PCs and pose dependence w.r.t. walk parameters
% and hence we identify the Component Mapping Functions CMs

global N_PCs polyDegree

joint_data = {'hl','kl','al','hr','kr','ar'};
NumSignals = 6;
all = 1;

% for each joint
for i_joints = 1:1
    
    % Stride Parameters which are equal for each joint so I take the first
    Speed = eval(strcat('Vel',joint_data{i_joints}));
    H_Step = eval(strcat('H',joint_data{i_joints}));
    L_Step = eval(strcat('L',joint_data{i_joints}));    
    
    PC_Coeff_all = eval(strcat('D_all*transpose(V_all)')); %
    PC_Coeff_mu_all = eval(strcat('D_mu_all*transpose(V_mu_all)'));
    
    Mu = eval(strcat('mu',joint_data{i_joints})); 
    
    %% Plot of speed/step_length curve region
    figure('Name','Speed/Step_L')
    plot(Speed,L_Step,'*'); hold on
    x_val = [0.04:0.01:0.5];
    y_val_1 = (x_val.^0.56).*0.3;
    y_val_2 = (x_val.^0.15).*0.3;
    plot(x_val, y_val_1, 'k','LineWidth',2);
    plot(x_val, y_val_2, 'k','LineWidth',2);
    xlim([0 0.6]);
    ylim([0 0.3]);
        
    for i_pc = 1:N_PCs+1             
        
        %% GIANMARIA METHOD
%         [fitresult_Pose, gof_Pose, type_Pose] = PlaneFit(Speed, H_Step, PC_Coeff_mu_all(i_pc,:), i_pc,'Pose',flag_display_figures,Q_or_TH,STR_INIT);
%         [fitresult_Trajectory, gof_Trajectory, type_Trajectory] = PlaneFit(Speed, H_Step, PC_Coeff_all(i_pc,:), i_pc,'Trajectory',flag_display_figures,Q_or_TH,STR_INIT);
% 
%         str=['Support/',STR_INIT,'N_',num2str(i_pc)]; 
%         save(str,'fitresult_Pose','gof_Pose','fitresult_Trajectory','gof_Trajectory','type_Pose','type_Trajectory');
% 
%         if gof_Pose.rsquare<=0.7
%             disp(['Pose interpolation for joint ',joint_data{i_joints},' PC ',num2str(i_pc),' is not representative'])
%             input(' ')
%         elseif gof_Trajectory.rsquare<=0.7
%             disp(['Trajectory interpolation for joint ',joint_data{i_joints},' PC ',num2str(i_pc),'  is not representative'])
%             input(' ')
%         end        
        
        %% EDITED By RICCARDO MENGACCI TO ADD THE THIRD PARAMETER FOR FITTING                
        fitresult_Trajectory = MultiPolyRegress([Speed',H_Step',L_Step'],PC_Coeff_all(i_pc,:)',polyDegree);
        fitresult_Pose = MultiPolyRegress([Speed',H_Step',L_Step'],PC_Coeff_mu_all(i_pc,:)',polyDegree);
        % Plots fitting results
        R_t = PC_Coeff_all(i_pc,:)';
        yhat_t = fitresult_Trajectory.yhat;
        R_p = PC_Coeff_mu_all(i_pc,:)';
        yhat_p = fitresult_Pose.yhat;
        figure('Name',strcat('Principal Component n°',num2str(i_pc)))
        subplot(1,2,1)                                                      % Trajectory
        title('Trajectory');
        line(yhat_t,R_t,'LineStyle','none','Marker','o','Color','r'); hold on; grid on;
        line([min([yhat_t,R_t]),max([yhat_t,R_t])],[min([yhat_t,R_t]),max([yhat_t,R_t])],'LineWidth',3,'color','black');
        subplot(1,2,2)                                                      % Pose
        title('Pose')
        line(yhat_p,R_p,'LineStyle','none','Marker','o','Color','r'); hold on; grid on;
        line([min([yhat_p,R_p]),max([yhat_p,R_p])],[min([yhat_p,R_p]),max([yhat_p,R_p])],'LineWidth',3,'color','black');

        % Saving the fitting results
        str=['Support/',STR_INIT,'N_',num2str(i_pc),'_TP']; 
        save(str,'fitresult_Pose','fitresult_Trajectory');

        if fitresult_Pose.RSquare<=0.7
            disp(['Pose interpolation for joint ',joint_data{i_joints},' PC ',num2str(i_pc),' is not representative'])
        elseif fitresult_Trajectory.RSquare<=0.7
            disp(['Trajectory interpolation for joint ',joint_data{i_joints},' PC ',num2str(i_pc),'  is not representative'])
        end
        %%         
    end % end for i_pc = 1:N_PCs+1 
    input(''); % Wait for user to see the results
end % end for i_joints 
