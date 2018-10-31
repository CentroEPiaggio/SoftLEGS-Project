% Plot the point to test before cut
figure('Name','Dataset Region Pre-Cut')
plot(SIMMAT(:,2)./SIMMAT(:,1),SIMMAT(:,2),'*'); hold on
x_val = [0.04:0.01:0.5];
y_val_1 = (x_val.^0.56).*0.3;
y_val_2 = (x_val.^0.15).*0.3;
plot(x_val, y_val_1, 'k','LineWidth',2);
plot(x_val, y_val_2, 'k','LineWidth',2);
xlim([0 0.6]);
ylim([0 0.3]);

% Remove the point out of the admittable area
SIMMAT_cut=[];
beta_up = 0.56;
beta_dw = 0.15;
leg_l = 0.3;
idx = 1;
% Elimination of points out of the speed-length admittable region
for i=1:length(SIMMAT)
   speed = SIMMAT(i,2)/SIMMAT(i,1);   % speed
   l_step = SIMMAT(i,2);
   h_step = SIMMAT(i,3);

   l_step_admit_up = (speed^beta_up)*leg_l;
   l_step_admit_dw = (speed^beta_dw)*leg_l;

   if (l_step >= l_step_admit_up && l_step <= l_step_admit_dw)
      SIMMAT_cut(idx,1) = l_step/speed;
      SIMMAT_cut(idx,2) = l_step;
      SIMMAT_cut(idx,3) = h_step;
      idx = idx + 1;
   end     
end

% Plot the point to test after cut
figure('Name','Dataset Region Post-Cut')
plot(SIMMAT_cut(:,2)./SIMMAT_cut(:,1),SIMMAT_cut(:,2),'*'); hold on
x_val = [0.04:0.01:0.5];
y_val_1 = (x_val.^0.56).*0.3;
y_val_2 = (x_val.^0.15).*0.3;
plot(x_val, y_val_1, 'k','LineWidth',2);
plot(x_val, y_val_2, 'k','LineWidth',2);
xlim([0 0.6]);
ylim([0 0.3]);

% Uncomment the following line to use the original dataset
SIMMAT = [];
SIMMAT = SIMMAT_cut;