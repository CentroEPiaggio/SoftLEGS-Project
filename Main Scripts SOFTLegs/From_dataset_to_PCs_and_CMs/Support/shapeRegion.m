function MAT_SIMS_cut = shapeRegion(MAT_SIMS)

% Remove outlines
beta_up = 0.56;
beta_dw = 0.15;
leg_l = 0.3;
idx = 1;

% User display
disp(['PERFORM SHAPING OF THE Speed/StepL REGION',10]);

MAT_SIMS_cut = {};
% Elimination of points out of the speed-length admittable region
for i=1:length(MAT_SIMS)
   speed = (MAT_SIMS{2,i}(2))/(MAT_SIMS{2,i}(1)); % speed
   l_step = MAT_SIMS{2,i}(2);
   h_step = MAT_SIMS{2,i}(3);

   l_step_admit_up = (speed^beta_up)*leg_l;
   l_step_admit_dw = (speed^beta_dw)*leg_l;

   if (l_step >= l_step_admit_up && l_step <= l_step_admit_dw)
      MAT_SIMS_cut{1,idx} = MAT_SIMS{1,i};
      MAT_SIMS_cut{2,idx} = [l_step/speed l_step h_step];
      MAT_SIMS_cut{3,idx} = MAT_SIMS{3,i};
      MAT_SIMS_cut{4,idx} = MAT_SIMS{4,i};
      MAT_SIMS_cut{5,idx} = MAT_SIMS{5,i};
      idx = idx + 1;
   end     
end
% Done!
disp(['SHAPING DONE!',10]);