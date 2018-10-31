%plot foot


dx=res_q(1,:);
dy=res_q(2,:);
q0=res_q(3,:);

q1=res_q(4,:);
q2=res_q(5,:);
q3=res_q(6,:);

q4=res_q(7,:);
q5=res_q(8,:);
q6=res_q(9,:);

LinkFemur=0.12;
LinkTibia=0.18;
LinkPelvis=0.05;
hfoot=1*0.074;

FLx=dx+LinkPelvis*sin(q0)+LinkFemur*sin(q0+q1)+LinkTibia*sin(q0+q1+q2)+hfoot*sin(q0+q1+q2+q3);
FRx=dx+LinkPelvis*sin(q0)+LinkFemur*sin(q0+q4)+LinkTibia*sin(q0+q4+q5)+hfoot*sin(q0+q4+q5+q6);

FLy=dy-LinkPelvis*cos(q0)-LinkFemur*cos(q0+q1)-LinkTibia*cos(q0+q1+q2)-hfoot*cos(q0+q1+q2+q3);
FRy=dy-LinkPelvis*cos(q0)-LinkFemur*cos(q0+q4)-LinkTibia*cos(q0+q4+q5)-hfoot*cos(q0+q4+q5+q6);

% close all
figure('Name',['Vel_',num2str(walk_dist/T),'_Fh_',num2str(h_foot),'__','Wdist_',num2str(round(walk_dist,3)),'_T_',num2str(round(T,2))],'Visible','off')
plot(FLx,FLy,'-o','LineWidth',2)
hold on
plot(FRx,FRy,'-o','LineWidth',2)
grid on
title('Foot Shape')
xlabel('x')
ylabel('y')



% saveas(gcf,'StepShape.fig')
