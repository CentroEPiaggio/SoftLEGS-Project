% This function generate the fitting surface for a fixed foot_h parameters
% passed as foot_h.
% The fitting is done with N_PCs components and the order passed as
% parameters
% NOTE that, to be used properly the set_SVD_TH has to be generated with
% the fixedStepH option (check inside the From_dataset_to_PCs_and_CMs
% script, line ~35), otherwise the dimensions are not consistent with the
% success cases.

function curveFit_Maps(foot_h, N_PCs, order)
    clc
    close all
    
    % Load the data from the Final_Maps folder (remember that they have to
    % be generated with the fixedStepH option)
    load('Final_Maps\set_SVD_TH.mat');
    
    % Set the foot_h value to be imposed
    % NOTE: the admittable values depend on the ones use for the optimizations

    D_all = cell2mat(set_SVD_TH(2));
    V_all = cell2mat(set_SVD_TH(3));
    D_mu_all = cell2mat(set_SVD_TH(5));
    V_mu_all = cell2mat(set_SVD_TH(6));
    
    PC_Coeff_all = D_all*transpose(V_all); 
    PC_Coeff_mu_all = D_mu_all*transpose(V_mu_all);
    
    % Retrieve parameters
    v = successful_cases(3,:);
    fh = successful_cases(4,:);
    fl = successful_cases(5,:);
    
    % Variables for storing the new results with only one foot_h params
    v_new = [];
    fl_new = [];
    idx = 1;
    for i=1:length(v)
        if (fh(i)==foot_h)
           v_new(idx) = v(i);
           fl_new(idx) = fl(i);
           idx = idx + 1;
        end
    end

    % Check dimensions
    if (size(successful_cases,2) ~= length(v_new))
        disp([10,'Wrong dimensions, please use the ''fixedStepH''',10,'option to generate the set_SVD_TH data',10]);
        return
    end
    
    % Compute and plot the fitting curves
    polyDegree = order;
    
    for i_pc=1:N_PCs
        % Fit the data with MultiPolyRegress function
        fitresult_Trajectory = MultiPolyRegress([v_new',fl_new'],PC_Coeff_all(i_pc,:)',polyDegree);
        fitresult_Pose = MultiPolyRegress([v_new',fl_new'],PC_Coeff_mu_all(i_pc,:)',polyDegree);
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
    
        % Display surface results from reconstructed data
        v_rec = [min(v):(max(v)-min(v))/100:max(v)];
        fl_rec = [min(fl):(max(fl)-min(fl))/100:max(fl)];
        % Get polynomials
        polyFun_t = fitresult_Trajectory.PolynomialExpression;
        polyFun_p = fitresult_Pose.PolynomialExpression;

        % Trajectory
        for i=1:length(v_rec)
            for j=1:length(fl_rec)
                PC_Coeff_all_rec(j,i) = polyFun_t(v_rec(i),fl_rec(j));
            end
        end
        % Comparison
        figure('Name',['Trajectory - PC n° ',num2str(i_pc),' Step_h: ',num2str(foot_h)])
        surf(v_rec,fl_rec,PC_Coeff_all_rec);
        hold on
        plot3(v_new,fl_new,PC_Coeff_all(i_pc,:)','*');
        hold on
        xlabel('speed');
        ylabel('step_L');
        zlabel('PC coeff');  
        
        % Pose
        for i=1:length(v_rec)
            for j=1:length(fl_rec)
                PC_Coeff_mu_all_rec(j,i) = polyFun_p(v_rec(i),fl_rec(j));
            end
        end
        % Comparison
        figure('Name',['Pose - PC n° ',num2str(i_pc),' Step_h: ',num2str(foot_h)])
        surf(v_rec,fl_rec,PC_Coeff_mu_all_rec);
        hold on
        plot3(v_new,fl_new,PC_Coeff_mu_all(i_pc,:)','*');
        hold on
        xlabel('speed');
        ylabel('step_L');
        zlabel('PC coeff');
    end
end
