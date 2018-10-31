% Analyse results to check CoP
COP_single = [];
COP_double = [];

for k=1:N
    F_k = res_Fc(:,k);
    if k <= N_trans
        F_y_left = F_k(2);
        M_left = F_k(3);
        COP_single_k = M_left/F_y_left;
        COP_single = [COP_single; COP_single_k];
        M_LB = Mx_left_LBfun(F_k);
        M_UB = Mx_left_UBfun(F_k);
    else
        F_y_left = F_k(2);
        M_left = F_k(3);
        F_y_right = F_k(5);
        M_right = F_k(6);
        COP_double_k = (M_left + M_right + F_y_right*walk_dist)/(F_y_left + F_y_right);
        COP_double = [COP_double; COP_double_k];
        M_LB = Mx_double_LBfun(F_k);
        M_UB = Mx_double_UBfun(F_k);
    end
    
%     %check lower bound
%     if full(M_LB) < 0
%         disp('lower bound not met')
%         k
%         successful_comb{i_succ,4} = 'lower bound not met';
%         break
%     end
%     
%     % check upper bound
%     if full(M_UB) <0
%         disp('upper bound not met')
%         k
%         successful_comb{i_succ,4} = 'upper bound not met';
%         break
%     end
    
end

% plot COP
close all
time = T/N*[0:N-1]';
COP_lb = -ones(N,1)*a_r;
COP_ub = [ ones(N_trans,1)*a_f; ones(N-N_trans,1)*(walk_dist+a_f) ];
figure('Visible','off');
plot(time, [COP_single; COP_double])
hold on
plot(time, COP_ub)
plot(time, COP_lb)
legend('COP', 'UB', 'LB')
figname = [name_sim_folder,'/COP.fig'];
savefig(figname);
saveas(gcf,[name_sim_folder,'/COP'],'png')