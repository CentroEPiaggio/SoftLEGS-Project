% close all
% clear all
clc
global N_PCs

cd('..');
for hp=1:2
    
    if hp==1
    load('Original_MAT_SIMS_THETA_walk')
    str='Original_';
    else
    load('New_MAT_SIMS_THETA_walk'); 
    str='New_';
    end

    % Load the maps data from the inside the folder
    load('..\Final_Maps\set_SVD_TH');
    U_all=set_SVD_TH{1};
    U_mu_all=set_SVD_TH{4};

    NumSamples = 60;
    str_val='CM_Validation_Figures';
    PC_Str = {'First_','Second_','Third_','Fourth_','Fifth_','Sixth_'};
    pc_toSave = '';
    mkdir(str_val)

    for i=1:N_PCs   % n° of synergy
        eval(strcat('s',num2str(i),'=',mat2str(U_all(:,i)'),';'));          % as s1 = U_all(:,1)
        eval(strcat(PC_Str{i},'PC = s',num2str(i),';'));

        eval(strcat('s',num2str(i),'_mu=',mat2str(U_mu_all(:,i)'),';'));    % as First_C = s1
        eval(strcat(PC_Str{i},'mu = s',num2str(i),'_mu;'));
        
        eval(strcat('dot',num2str(i),'=',num2str(0),';'));                  % as dotX = 0
    end

    Dataset = [];
    Dataset_m = [];
    Means = [];

    for i = 1 : size(MAT_SIMS,2)
        act_traj = MAT_SIMS{4,i};
        act_traj = act_traj(:,1:NumSamples);
        act_traj_v_m = [act_traj(1,:) act_traj(2,:) act_traj(3,:) act_traj(4,:) act_traj(5,:) act_traj(6,:)];
        for j = 1 : 6
            Means(j,i) = mean(act_traj(j,:));
            act_traj(j,:) = act_traj(j,:) - Means(j,i);
        end
        act_traj_v = [act_traj(1,:) act_traj(2,:) act_traj(3,:) act_traj(4,:) act_traj(5,:) act_traj(6,:)];
        Dataset = [Dataset; act_traj_v];
        Dataset_m = [Dataset_m; act_traj_v_m];

        for k=1:N_PCs   % n° of synergy
            eval(strcat('dot',num2str(k),'=','[dot',num2str(k),',','dot(act_traj_v,','s',num2str(k),')];'));          % dotX = [dotX dot(act_traj_v,sX)];
        end
    end

%% try one reconstruction
% numsample = 113;
% rec1 = dot1(numsample)*s1;
% rec2 = dot1(numsample)*s1+dot2(numsample)*s2;
% rec3 = dot1(numsample)*s1+dot2(numsample)*s2+dot3(numsample)*s3;
% 
% rec1_r = Means(:,numsample) + reshape(rec1,[NumSamples,6])';
% rec2_r = Means(:,numsample) + reshape(rec2,[NumSamples,6])';
% rec3_r = Means(:,numsample) + reshape(rec3,[NumSamples,6])';
% 
% act_traj = MAT_SIMS{4,numsample};act_traj = act_traj(:,1:NumSamples);
% act_traj_v = [act_traj(1,:) act_traj(2,:) act_traj(3,:) act_traj(4,:) act_traj(5,:) act_traj(6,:)];
%     
% rec1_v = reshape(rec1_r',[1,6*NumSamples]);
% rec2_v = reshape(rec2_r',[1,6*NumSamples]);
% rec3_v = reshape(rec3_r',[1,6*NumSamples]);
% 
% figure, plot(act_traj_v','k','LineWidth',3)
% hold on, plot(rec1_v,'r'), plot(rec2_v,'g'), plot(rec3_v,'b') 

%% Check the reconstruction using exact gains
Err = [];
recTot = [];
for i = 1 : max(size(MAT_SIMS))
    
    for k=1:N_PCs   % n° of synergy
        eval(strcat('rec',num2str(k),'=dot',num2str(k),'(i)*','s',num2str(k),';'));     % recX = dotX(i)*sX;
        eval(strcat('recTot = [recTot;','rec',num2str(k),'];'));                        % recTot = [rec1; rec2;....]                              
    end
    
    for k=1:N_PCs   % n° of synergy
        if (k==1) 
            rec(k,:) = rec1;
        else
            rec(k,:) = sum(recTot(1:k,:));                                              % sum of the recTot component
        end
    end
    
    recTot = [];
    err = [];
    
    % Compute errors
    for k=1:N_PCs   % n° of synergy
        eval(strcat('err',num2str(k),'= norm(rec(',num2str(k),',:)-Dataset(i,:))/length(rec(',num2str(k),',:));')); % errX = norm(recX-Dataset(i,:))/length(recX)
        err = [err; eval(['err',num2str(k)])];
    end
    
    Err = [Err err];
end

% figure, hold on, plot(Err','*'), 
% figure, plot(Err(2,:)','ok','MarkerSize',7,'MarkerFaceColor','g'),hold on, 
% xlabel('Simulation #'), ylabel('Normalized Error per frame (rad)')
% legend('1 PC', '2 PC', '3PC')
% title('Trajectory Reconstruction for Validation Set')
% name='Trajectory Reconstruction for Validation Set';
% savefig(gcf,['Figures_Validation/TH_',name]);

%% check the predictability of maps
vect_sims=1:max(size(MAT_SIMS));
Vels=zeros(1,length(vect_sims));
Hs=zeros(1,length(vect_sims));
Ls=zeros(1,length(vect_sims));
for i_sims = 1 : length(vect_sims)
    Vels(i_sims) = MAT_SIMS{2,vect_sims(i_sims)}(2)/MAT_SIMS{2,vect_sims(i_sims)}(1);
    Hs  (i_sims) = MAT_SIMS{2,vect_sims(i_sims)}(3);
    Ls  (i_sims) = MAT_SIMS{2,vect_sims(i_sims)}(2);
end

Err_maps = [];
for i = 1 : max(size(MAT_SIMS))
    rec_maps = Original_From_CM_To_Traj_TP(Vels(i),Hs(i),Ls(i),U_all,U_mu_all);
    rec_maps_lin = reshape(rec_maps',1,360);   
    err_maps = norm(rec_maps_lin-Dataset_m(i,:))/length(rec_maps_lin); 
    Err_maps = [Err_maps err_maps];
end

figure, hold on, 
plot(Err_maps','ok','MarkerSize',7,'MarkerFaceColor','r'), 
ylim([0 0.008])
if hp==2
xlabel('New Simulation #')
else
end
grid on
%, ylabel('Normalized Error per frame (rad)')
ylabel('Error per frame (rad)')
%legend('1 PC', '2 PC', '3PC')

legend('2 PCs','MCs')
name='Trajectory Reconstruction PCs Vs Maps';
set(gca,'FontSize',20)
savefig(gcf,[str_val,'/TH_',str,name]);

end

% %
% for hp=1:2
%     
%     if hp==1
%     load('Original_MAT_SIMS_THETA_walk')
%     str='Original_';
%     else
%     load('New_MAT_SIMS_THETA_walk'); 
%     str='New_';
%     end
% 
%     Load the maps data from the inside the folder
%     load('..\Final_Maps\set_SVD_TH');
%     U_all=set_SVD_TH{1};
%     U_mu_all=set_SVD_TH{4};
%     NumSamples = 60;
%     mkdir(str_val)
% 
%     s1 = U_all(:,1)';
%     s2 = U_all(:,2)';
%     s3 = U_all(:,3)';
%     s1_mu = U_mu_all(:,1)';
%     s2_mu = U_mu_all(:,2)';
%     s3_mu = U_mu_all(:,3)';
% 
%     dot1 = []; dot2 = []; dot3 = [];
%     Dataset = [];
%     Dataset_m = [];
%     Means = [];
%     for i = 1 : size(MAT_SIMS,2)
%         act_traj = MAT_SIMS{4,i};
%         act_traj = act_traj(:,1:NumSamples);
%         act_traj_v_m = [act_traj(1,:) act_traj(2,:) act_traj(3,:) act_traj(4,:) act_traj(5,:) act_traj(6,:)];
%         for j = 1 : 6
%             Means(j,i) = mean(act_traj(j,:));
%             act_traj(j,:) = act_traj(j,:) - Means(j,i);
%         end
%         act_traj_v = [act_traj(1,:) act_traj(2,:) act_traj(3,:) act_traj(4,:) act_traj(5,:) act_traj(6,:)];
%         Dataset = [Dataset; act_traj_v];
%         Dataset_m = [Dataset_m; act_traj_v_m];
% 
%         dot1 = [dot1 dot(act_traj_v,s1)];
%         dot2 = [dot2 dot(act_traj_v,s2)];
%         dot3 = [dot3 dot(act_traj_v,s3)];
%     end
% 
%     % Check the reconstruction using exact gains
%     Err = [];
%     recTot = [];
%     vect_sims=1 : max(size(MAT_SIMS));
%     Vels=zeros(1,length(vect_sims));
%     Hs=zeros(1,length(vect_sims));
%     for i = vect_sims
% 
%         Vels(i) = MAT_SIMS{2,vect_sims(i)}(2)/MAT_SIMS{2,vect_sims(i)}(1);
%         Hs(i) = MAT_SIMS{2,vect_sims(i)}(3);
%         Ls(i) = MAT_SIMS{2,vect_sims(i)}(2);
% 
%         for k=1:N_PCs   % n° of synergy
%         	eval(strcat('rec',num2str(k),'=dot',num2str(k),'(i)*','s',num2str(k),';'));     % recX = dotX(i)*sX;
%         	eval(strcat('recTot = [recTot;','rec',num2str(k),'];'));                        % recTot = [rec1; rec2;....]                              
%         end
% 
%         for k=1:N_PCs   % n° of synergy
%             if (k==1) 
%                 rec(k,:) = rec1;
%             else
%                 rec(k,:) = sum(recTot(1:k,:));                                              % sum of the recTot component
%             end
%         end
% 
%         recTot = [];
%         err = [];
% 
%         Compute errors
%         for k=1:N_PCs   % n° of synergy
%             eval(strcat('err',num2str(k),'= norm(rec(',num2str(k),',:)-Dataset(i,:))/length(rec(',num2str(k),',:));')); % errX = norm(recX-Dataset(i,:))/length(recX)
%             err = [err; eval(['err',num2str(k)])];
%         end
%     end
% 
%     Err = [Err err];
%     figure, hold on, plot(Err','*'), 
%     % figure, plot3(Vels,Hs,Err(2,:)','ok','MarkerSize',7,'MarkerFaceColor','g'),hold on, 
%     % max(Vels)
%     % xlabel('Forward Speed'), ylabel('Foot Height'),zlabel('Error per frame (rad)')
%     % grid on
%     legend('1 PC', '2 PC', '3PC')
%     title('Trajectory Reconstruction for Validation Set')
%     name='Trajectory Reconstruction for Validation Set';
%     savefig(gcf,[str_val,'/TH_',name]);
% 
%     % check the applicability of maps
%     vect_sims=1:max(size(MAT_SIMS));
%     Vels=zeros(1,length(vect_sims));
%     Hs=zeros(1,length(vect_sims));
%     Ls=zeros(1,length(vect_sims));
% 
%     for i_sims = 1 : length(vect_sims)
%         Vels(i_sims) = MAT_SIMS{2,vect_sims(i_sims)}(2)/MAT_SIMS{2,vect_sims(i_sims)}(1);
%         Hs  (i_sims) = MAT_SIMS{2,vect_sims(i_sims)}(3);
%         Ls  (i_sims) = MAT_SIMS{2,vect_sims(i_sims)}(2);
%     end
% 
%     Err_maps = [];
%     for i = 1 : max(size(MAT_SIMS))
%         rec_maps = Original_From_CM_To_Traj_TP(Vels(i),Hs(i),Ls(i),U_all,U_mu_all);   
%         rec_maps_lin = reshape(rec_maps',1,360);
%         err_maps = norm(rec_maps_lin-Dataset_m(i,:))/length(rec_maps_lin); 
%         Err_maps = [Err_maps err_maps];
%     end
% 
%     % % figure, hold on, 
%     % plot3(Vels,Hs,Err_maps','ok','MarkerSize',7,'MarkerFaceColor','r'), 
%     % zlim([0 0.008])
%     % view([-57 9])
%     % % xlabel('Simulation #')
%     % %, ylabel('Normalized Error per frame (rad)')
%     % % ylabel('Error per frame (rad)')
%     % %legend('1 PC', '2 PC', '3PC')
%     % % title('Trajectory Reconstruction using Maps for Validation Set')
%     % legend('2 PCs','MCs')
%     % name='Trajectory Reconstruction PCs Vs Maps';
%     % set(gca,'FontSize',20)
%     % savefig(gcf,[str_val,'/TH_',str,'3D_',name]);
% 
% end