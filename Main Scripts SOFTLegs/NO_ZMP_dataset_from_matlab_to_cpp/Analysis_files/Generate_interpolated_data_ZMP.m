function [New_PQ,New_TH,New_time]=Generate_interpolated_data_ZMP(N_step,res_q,res_theta,time,Ts,display_step_step,foot_display)
% display_step_step=1;

res_q=[res_q(1:3,:)*0;res_q];

FB=res_q(1:3,:);
Px=FB(1,1:end);
Py=FB(2,1:end);
Pq0=FB(3,1:end);


L=res_q(4:6,:);
R=res_q(7:9,:);

LTH=res_theta(1:3,:);
RTH=res_theta(4:6,:);


% Ttime=res_ts*T/N;
% Ttime=[0 Ttime];
% 
% time=Ttime;
% for j=3:size(Ttime,2)
% time(j)=time(j)+time(j-1);
% end


timeb=time(2:end);
Pyb=Py(2:end);
Pxb=Px(2:end)-Px(1);
Pq0b=Pq0(2:end);

L_1=L(:,2:end);
R_1=R(:,2:end);

LTH_1=LTH(:,2:end);
RTH_1=RTH(:,2:end);

PPx=Px;
PPy=Py;
Ptime=time;
PPq0=Pq0;

PL=L;
PR=R;

PLTH=LTH;
PRTH=RTH;



% time start by 0

for i=1:N_step

    if i>1
        PPx=[PPx,PPx(end)+Pxb];
        Ptime=[Ptime,timeb+Ptime(end)];
        PPy=[PPy,Pyb];
        PPq0=[PPq0,Pq0b];
        if mod(i,2) %odd
            PL=[PL,L_1];
            PR=[PR,R_1];
            %
            PLTH=[PLTH,LTH_1];
            PRTH=[PRTH,RTH_1];
        else %even
            PL=[PL,R_1];
            PR=[PR,L_1];
            %
            PLTH=[PLTH,RTH_1];
            PRTH=[PRTH,LTH_1];

        end
    end

if display_step_step
    
        display(i,'N_step')
        PR(1,:)
    
    figure(1)
clf
subplot(2,1,1)
plot(PL(1,:),'-o')
legend('Left')
    title('Hip Joint Links')
grid on
subplot(2,1,2)
plot(PR(1,:),'-o')
legend('Right')
grid on
xlabel('Samples');

figure(2)
clf
subplot(2,1,1)
plot(Ptime,PL(1,:),'-o')
title('Hip Joint Links')
legend('Left')
grid on
subplot(2,1,2)
plot(Ptime,PR(1,:),'-o')
legend('Right')
grid on
xlabel('Time');
%     input(' ')
end


end
%%
if foot_display
figure(3)
clf
title('Foot Orientation')
plot(Ptime,sum([PPq0;PL],1))
hold on
plot(Ptime,sum([PPq0;PR],1))
legend('Left','Right')
grid on
end
%% Interpolate

% PPx=Px;
% PPy=Py;
% Ptime=time;
% PPq0=Pq0;
% PL=L;
% PR=R;

PQ=[PPx;PPy;PPq0;PL;PR]';

PTH=[PLTH;PRTH]';

Tf=Ptime(end);
New_time=0:Ts:Tf;

New_PQ=zeros(size(New_time,2),size(PQ,2));
New_TH=zeros(size(New_time,2),size(PTH,2));

save('test','New_time','New_PQ','New_TH','Ptime','PQ','PTH')

for hq=1:9
New_PQ(:,hq)=interp1(Ptime,PQ(:,hq),New_time,'spline');
end
for hq=1:6
New_TH(:,hq)=interp1(Ptime,PTH(:,hq),New_time,'spline');
end
