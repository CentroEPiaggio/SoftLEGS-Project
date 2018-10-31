function MAT_SIMS_cut = fixedStepH(MAT_SIMS)

% User display
disp(['USING FIXED StepH PARAMETER',10]);

% Retrieve the step_h params used
list_param = cell2mat(MAT_SIMS(2,:)');
list_fh = list_param(:,3);
% Define the number of params to show
if (length(list_fh) > 8)
    n_par = 8;
else 
    n_par = length(list_fh);
end

% Interactive menu
disp('Choose the parameter to use among the first eight:');
disp('---------------------------------');
for s=1:n_par % display the first 10 values
   disp([num2str(s),') ',num2str(list_fh(s))]);
end
disp('---------------------------------');
item = input('Choose one param: ','s');
itemN = str2num(item);
des_fh = list_fh(itemN); % assign the choice to des_fh
disp('');

idx = 1;
MAT_SIMS_cut = {};
% Selection of only the data with the desired foot_h
for i=1:length(MAT_SIMS)
   speed = (MAT_SIMS{2,i}(2))/(MAT_SIMS{2,i}(1)); % speed
   l_step = MAT_SIMS{2,i}(2);
   h_step = MAT_SIMS{2,i}(3);

   if (h_step == des_fh)
      MAT_SIMS_cut{1,idx} = MAT_SIMS{1,i};
      MAT_SIMS_cut{2,idx} = [l_step/speed l_step h_step];
      MAT_SIMS_cut{3,idx} = MAT_SIMS{3,i};
      MAT_SIMS_cut{4,idx} = MAT_SIMS{4,i};
      MAT_SIMS_cut{5,idx} = MAT_SIMS{5,i};
      idx = idx + 1;
   end
end

% % Reconstruct the map to show the goodness and to plot the surface
% v = list_param(:,1);
% fl = list_param(:,2);
% 
% v_rec = [min(v):(max(v)-min(v))/100:max(v)];
% fl_rec = [min(fl):(max(fl)-min(fl))/100:max(fl)];
% polyFun = res.PolynomialExpression;
% 
% figure('Name',['Step_h: ',num2str(des_fh)])

