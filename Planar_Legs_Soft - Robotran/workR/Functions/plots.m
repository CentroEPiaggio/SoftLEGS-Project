%% Plot the 3D points [v,fl,fh] with color and dimension of each point 
% dependent on the CoT derived from the optimization solutions

figure('Name','Param vs. CoT')
cmap = colormap;
c = successful_cases(2,:);
c = round(1+(size(cmap,1)-1)*(c - min(c))/(max(c)-min(c)));
for i=1:length(successful_cases)
    plot3(successful_cases(3,i),successful_cases(5,i),successful_cases(4,i),'*','Color',cmap(c(i),:),'LineWidth',successful_cases(2,i));
    hold on; grid on;
    xlabel('speed');
    ylabel('step_L');
    zlabel('step_H');
end
colorbar
caxis([ min(successful_cases(2,:)) , max(successful_cases(2,:))]) % colorbar limits 



% Plot only v and fl vs. CoT
figure('Name','Speed/StepL vs. CoT')
cmap = colormap;
c = successful_cases(2,:);
c = round(1+(size(cmap,1)-1)*(c - min(c))/(max(c)-min(c)));
for i=1:length(successful_cases)
    plot3(successful_cases(3,i),successful_cases(5,i),successful_cases(2,i),'*','Color',cmap(c(i),:),'LineWidth',successful_cases(2,i));
    hold on; grid on;
    xlabel('speed');
    ylabel('step_L');
    zlabel('CoT');
end
colorbar
caxis([ min(successful_cases(2,:)) , max(successful_cases(2,:))]) % colorbar limits 

