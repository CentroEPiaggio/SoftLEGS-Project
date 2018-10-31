function curveFit_Maps(foot_h, succ_cases, set_SVD_TH)
    clc

    % Set the foot high value to be imposed
    % NOTE: the admittable values depend on the ones use for the optimizations
    % last case: [0.01:0.025:0.1] = [0.0100    0.0350    0.0600    0.0850]
    D_all = cell2mat(set_SVD_TH(2));
    V_all = cell2mat(set_SVD_TH(3));
    D_mu_all = cell2mat(set_SVD_TH(5));
    V_mu_all = cell2mat(set_SVD_TH(6));
    
    PC_Coeff_all = D_all*transpose(V_all); 
    PC_Coeff_mu_all = D_mu_all*transpose(V_mu_all);
    
    % Variables for storing the results
    v_new = [];
    fl_new = [];
    CoT_new = [];

    % Retrieve parameters
    CoT = succ_cases(2,:);
    v = succ_cases(3,:);
    fh = succ_cases(4,:);
    fl = succ_cases(5,:);

    idx = 1;
    for i=1:length(v)
        if (fh(i)==foot_h)
           v_new(idx) = v(i);
           fl_new(idx) = fl(i);
           CoT_new(idx) = CoT(i);
           idx = idx + 1;
        end
    end

    % Fit the data with MultiPolyRegress function
    polyDegree = 3;
    res = MultiPolyRegress([v',fh',fl'],PC_Coeff_all(2,:)',polyDegree,'figure');

%     % Display surface results from reconstructed data
%     v_rec = [min(v):(max(v)-min(v))/100:max(v)];
%     fl_rec = [min(fl):(max(fl)-min(fl))/100:max(fl)];
%     polyFun = res.PolynomialExpression;
% 
%     for i=1:length(v_rec)
%         for j=1:length(fl_rec)
%             CoT_rec(j,i) = polyFun(v_rec(i),fl_rec(j));
%         end
%     end

    % Comparison
    % plot3(v_rec,fl_rec,CoT_rec);
%     figure('Name',['Step_h: ',num2str(foot_h)])
%     surf(v_rec,fl_rec,CoT_rec);
%     hold on
%     plot3(v_new,fl_new,CoT_new,'*');
%     hold on
%     xlabel('speed');
%     ylabel('step_L');
%     zlabel('CoT');
end
