import casadi.*

close all
% Plot results
dim = size(casadi_struct2vec(w_block));
res_split = vertsplit(sol.x,[0,(N_trans-1)*dim(1),sol.x.size1()]);
res_r = res_split{1};
res_t_b = res_split{2};

res_r_split = vertsplit(res_r,dim(1));

dim_t = size(casadi_struct2vec(w_block_t));
res_split_t_b = vertsplit(res_t_b,[0,dim_t(1),res_t_b.size1()]);
res_t_split = res_split_t_b{1};
res_b_split = vertsplit(res_split_t_b{2},dim(1));

res_U = {};
res_Z = {};
res_Z2 = {};
for r = res_r_split(1:end)
    rs = casadi_vec2struct(w_block,r{1});
    res_U = {res_U{:} rs.U};
    res_Z = {res_Z{:} rs.Z(:,1)};
    res_Z2 = {res_Z2{:} rs.Z(:,2)};
end

rs = casadi_vec2struct(w_block_t,res_t_split);
res_U = {res_U{:} rs.U};
res_Z = {res_Z{:} rs.Z(:,1)};
res_Z2 = {res_Z2{:} rs.Z(:,2)};

for r = res_b_split(1:end-1)
    rs = casadi_vec2struct(w_block,r{1});
    res_U = {res_U{:} rs.U};
    res_Z = {res_Z{:} rs.Z(:,1)};
    res_Z2 = {res_Z2{:} rs.Z(:,2)};
end

res_split = full([res_U{:}]);
res_K = res_split(1:6,:);    % RES_K
res_inputs = res_split(7:12,:);              
res_ts = res_split(end,:);


res_X = {};
for r = res_r_split(1:end)
    rs = casadi_vec2struct(w_block,r{1});
    res_X = {res_X{:} rs.X};
end

rs = casadi_vec2struct(w_block_t,res_t_split);
res_X = {res_X{:} rs.X};

for r = res_b_split(1:end-1)
    rs = casadi_vec2struct(w_block,r{1});
    res_X = {res_X{:} rs.X};
end
res_X = {res_X{:} res_b_split{end}};

invariants0 = Function('invariants0',{dae_s.x},{vertcat(c_0(2:3), cd_0(2:3), alpha_0, alphad_0)});
invariants1 = Function('invariants1',{dae_s.x, dist},{vertcat(c_1_diff(2:3), cd_1(2:3), alpha_1, alphad_1)});

res_q = [];
inv_err0 = [];
inv_err1 = [];
res_q = [];
res_qd = [];
res_theta = [];
res_thetad = [];
res_thetadd =[];
%     res_inputs = [];

for hd = 1:size(res_X,2) 

    x = res_X{hd};
    out0 = invariants0(x);
    inv_err0 = [inv_err0 full(out0)];
    out1 = invariants1(x, walk_dist);
    inv_err1 = [inv_err1 full(out1)];
    sol_x = casadi_vec2struct(dae_x,x);
    res_q = [res_q full(sol_x.q)];
    res_qd = [res_qd full(sol_x.qd)];
    res_theta = [res_theta full(sol_x.theta)];
    res_thetad = [res_thetad full(sol_x.thetad)];
end

res_Fc = [];
res_qdd = [];
for z = res_Z2
    z = z{1};    
    sol_z = casadi_vec2struct(dae_z,z);
    res_Fc = [res_Fc full(sol_z.Fc)];
    res_qdd = [res_qdd full(sol_z.qdd)];
    res_thetadd=[res_thetadd full(sol_z.thetadd)];
end

    
time = vertcat(T/N*[0:N]);
% optimal value (COT)
val_opt=sol.f;
% torques
tau=res_inputs(1:6,:);
%status
status=solver.stats.return_status;

val=full(val_opt);
% Folder name to save solutions
name_sim_folder=[name1,name2,num2str(index)];

if exist(name_sim_folder)
else mkdir(name_sim_folder)
end


name_sim=[name_sim_folder,'/sim_walk_',num2str(index)];
    
if exist('res_K','var')
% [Kc_swing, eig_values_swing, eig_vect_swing, detJJT_swing] = Cartesian_Stiffness_right(MBS_data, res_q, res_K);
% [Kc_stance, eig_values_stance, eig_vect_stance, detJJT_stance] = Cartesian_Stiffness_left(MBS_data, res_q, res_K);
else
    res_K=[];
end
    Kc_swing=[];
    eig_values_swing=[];
    eig_vect_swing=[];
    detJJT_swing=[];
    Kc_stance=[];
    eig_values_stance=[];
    eig_vect_stance=[];
    detJJT_stance=[];
% end



% save the single simulation
save(name_sim,'index','res_K','h_foot','T','status','val','walk_dist','res_q','res_qd','res_qdd','res_ts','res_theta','res_thetad','tau','res_Fc','inv_err0','inv_err1','Kc_swing', 'eig_values_swing', 'eig_vect_swing', 'detJJT_swing','Kc_stance', 'eig_values_stance', 'eig_vect_stance', 'detJJT_stance');

% store the results
mat_ris{Mindex,1}=index;
mat_ris{Mindex,2}=res_K;
mat_ris{Mindex,3}=status;
mat_ris{Mindex,4}=full(val_opt);
mat_ris{Mindex,5}=[walk_dist , h_foot, T];
mat_ris{Mindex,6}=res_q;
mat_ris{Mindex,7}=res_qd;
mat_ris{Mindex,8}=res_qdd;
mat_ris{Mindex,9}=res_ts;
mat_ris{Mindex,10}=res_theta;
mat_ris{Mindex,11}=res_thetad;
mat_ris{Mindex,12}=tau;
mat_ris{Mindex,13}=res_Fc; 
mat_ris{Mindex,14}= [gamma_x gamma_y gamma_alpha]; 
Mindex=Mindex+1;
    
% save matrix to display
time = vertcat(T/N*[0:N]);
inv_err_pos_x = inv_err0(1,:);
inv_err_pos_y = inv_err0(2,:);
inv_err_pos_gamma = inv_err0(5,:);
% % % % A = vertcat(time, res_q);
% % % % fid = fopen('poseList.dat','wt');
% % % % for ii = 1:size(A,1)
% % % %     fprintf(fid,'%g\t',A(ii,:));
% % % %     fprintf(fid,'\n');
% % % % end
% % % % fclose(fid);
% % % % 
% % % % % plot invariants
% % % % figure(1);
% % % % subplot(3,1,1);
% % % % plot(time, [inv_err0(1,:);inv_err0(3,:)]')
% % % % legend('c_0(x)', 'cd_0(x)')
% % % % 
% % % % subplot(3,1,2);
% % % % plot(time, [inv_err0(2,:);inv_err0(4,:)]')
% % % % legend('c_0(y)', 'cd_0(y)')
% % % % 
% % % % subplot(3,1,3);
% % % % plot(time, [inv_err0(5,:);inv_err0(6,:)]')
% % % % legend('alpha_0', 'alphad_0')
% % % % xlabel('time (s)')
% % % % title('invariants')
% % % % figname = [name_sim_folder,'/inv_err.fig'];
% % % % savefig(figname);
% % % % 
% % % % % plot invariants
% % % % figure(1);
% % % % subplot(3,1,1);
% % % % inv_err_pos_x = inv_err0(1,:);
% % % % plot(time, inv_err_pos_x')
% % % % legend('c_0(x)')
% % % % 
% % % % subplot(3,1,2);
% % % % inv_err_pos_y = inv_err0(2,:);
% % % % plot(time, inv_err_pos_y')
% % % % legend('c_0(y)')
% % % % 
% % % % subplot(3,1,3);
% % % % inv_err_pos_gamma = inv_err0(5,:);
% % % % plot(time, inv_err_pos_gamma')
% % % % legend('alpha_0')
% % % % xlabel('time (s)')
% % % % title('invariants')
% % % % figname = [name_sim_folder,'/pos_inv_err.fig'];
% % % % savefig(figname);
% % % % 
% % % % % stiffness analysis
% % % % [Kc_swing, eig_values_swing, eig_vect_swing, detJJT_swing] = Cartesian_Stiffness_right(MBS_data, res_q, res_K);
% % % % [Kc_stance, eig_values_stance, eig_vect_stance, detJJT_stance] = Cartesian_Stiffness_left(MBS_data, res_q, res_K);
% % % % 
% % % % ind_sing_swing = find(eig_values_swing >= 1e10);
% % % % ind_sing_stance = find(eig_values_stance >= 1e10);
% % % % 
% % % % time = T/N*[0:N];
% % % % 
% % % % swing_ind_ok=setdiff(1:size(eig_values_swing,1),ind_sing_swing);
% % % % eig_values_swing_free = eig_values_swing(swing_ind_ok,:);
% % % % time_swing = time(swing_ind_ok);
% % % % 
% % % % stance_ind_ok=setdiff(1:size(eig_values_stance,1),ind_sing_stance);
% % % % eig_values_stance_free = eig_values_stance(stance_ind_ok,:);
% % % % time_stance = time(stance_ind_ok);
% % % % 
% % % % % plot results
% % % % figure;
% % % % plot(time_swing,eig_values_swing_free(:,1)')
% % % % hold on
% % % % plot(time_stance,eig_values_stance_free(:,1)')
% % % % legend('e_1(swing)','e_1(stance)')
% % % % xlabel('time(s)')
% % % % figname = [name_sim_folder,'/Kc_vert.fig'];
% % % % savefig(figname)
% % % % 
% % % % figure;
% % % % plot(time_swing,eig_values_swing_free(:,2)')
% % % % hold on
% % % % plot(time_stance,eig_values_stance_free(:,2)')
% % % % legend('e_2(swing)','e_2(stance)')
% % % % xlabel('time(s)')
% % % % figname = [name_sim_folder,'/Kc_hor.fig'];
% % % % savefig(figname)
% % % % 
% % % % figure;
% % % % plot(time_swing,eig_values_swing_free(:,3)')
% % % % hold on
% % % % plot(time_stance,eig_values_stance_free(:,3)')
% % % % legend('e_3(swing)','e_3(stance)')
% % % % xlabel('time(s)')
% % % % figname = [name_sim_folder,'/Kc_tors.fig'];
% % % % savefig(figname)