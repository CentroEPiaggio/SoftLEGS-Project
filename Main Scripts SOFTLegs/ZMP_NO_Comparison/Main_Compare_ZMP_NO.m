% This file compare energy efficiency for walks provided by two different 
% methods: NO and MPC Kajita 2007.

clc
clearvars
close all

% Load NO data
load('Whole_1_250')
load('succ_cases')


% To store results
FLD_BASE='Res/';
mkdir(FLD_BASE)

% Matrices
JMAT=[];
Q0=[];
FMIN_RES={};

% Now considering successful case set I may choose to evaluate the whole
% set or a reduced section of it.

Vel_range=[0.1:0.1:0.3]; %speed
foot_h_range=[0.02:0.005:0.03]; %foot cleareance

tic
for sim_i=1:size(successful_cases,2)

    % take the data
res_theta=mat_ris{successful_cases(1,sim_i),10};
res_q=mat_ris{successful_cases(1,sim_i),6};
res_ts=mat_ris{successful_cases(1,sim_i),9};
T=mat_ris{successful_cases(1,sim_i),5}(3);
walk_dist=mat_ris{successful_cases(1,sim_i),5}(1);
foot_h=mat_ris{successful_cases(1,sim_i),5}(2);
K=mat_ris{successful_cases(1,sim_i), 2}(:,1);
tau=mat_ris{successful_cases(1,sim_i),12};
res_qd=mat_ris{successful_cases(1,sim_i),7};

Vel=2*walk_dist/T;

ncond1 = isempty(find(abs(Vel-Vel_range)<=eps));
ncond2 = isempty(find(abs(foot_h-foot_h_range)<=eps));

if  ncond1 || ncond2
    continue
end

% Simulation Data
disp('Simulation data N° Vel Foot H')
display([sim_i,Vel,foot_h])


%% Elaborate data of optimal locomotion set
% In the numerical optimization 28 of 30 steps are dedicated to the single
% phase (SP)

N=30;
NSP=28; %N Single Phase

SP_vector=1:NSP;

% Calculate the real time vector
res_T_real=[0 res_ts*0];

for ti=1:size(res_ts,2)
res_T_real(ti+1)=res_T_real(ti)+res_ts(ti)*T/size(res_ts,2);
end

% Take Time, torques and link configuration during the SP
Res_T_SP=res_T_real(1:SP_vector(end)+1);

Tau_SP=tau(:,1:SP_vector(end)+1);

Q=res_q(4:end,1:SP_vector(end)+1);
Q_all=res_q(4:end,1:end);


% Signals have to be switched because in the NO the model is floating base 
% while in the following is a classic serial 
% NO conf: Hip Knee Ankle Hip Knee Ankle MPC conf: Ankle Knee Hip Hip Knee Ankle 
Q_SP=[flipud(Q(1:3,:)); Q(4:6,:)];
Q_ALL=[flipud(Q_all(1:3,:)); Q_all(4:6,:)];

% We need the step and COM trajectories

PF=[];
PC=[];

for j=1:size(Q_SP,2)
[FOOT,COM]=DK_fun_fmincon(Q_SP(:,j));
PF=[PF FOOT];
PC=[PC COM];
end


% Step Length
Step_L=PF(1,end);

% Calculate the COT of the NO just for the single phase

g=9.81;
MassFemur=0.7;
MassTibia=1;
MassTibia2=1; 
MassPelvis=2.7;
MassTrunk= 0;
MassFoot=0.55;
m_robot=2*MassFemur+2*MassTibia+2*MassFoot+ MassPelvis;
NEW_J_SP=sum(sum((Tau_SP.^2)*(res_T_real(2)-res_T_real(1)))/(m_robot*g*Step_L));

%%
% Now we calculate the COT for the MPC approach. First we solve the MPC
% problem, than we take a single step when the robot is considered at
% regime.

% COM height
Com_h=mean(PC(2,:));
% Ref Time
Tref=Res_T_SP(end);    


% N° of samples in a step
step=size(Res_T_SP,2);
%Prediction Horizon
N=step*3; % 3 step length
% Sample time
Ts=Tref/N;

Step_W=0.10; % pelvis dimensions

% Execute Kajita MPC

%Number of step
N_step=14;

% test_Kajita_GM_with_Bounds_QbLegs;
[Cx,DCx,DDCx,Zx,Cy,DCy,DDCy,Zy,Zx_ref]=KAJITA_MPC(N_step,Ts,N,NSP,Com_h,Step_L,Step_W);


%% Take just one single phase when limit cycle is reached

ZV=find(diff(Zx_ref)~=0); % Find the instant when the step is provided

ind1=ZV(round(size(ZV,1)/2))+1;
ind2=ind1+2*step;

% constraint only the X component
KAJ_COM_X=Cx(ind1:ind2)-Cx(ind1+round((ind2-ind1+1)/2));
KAJ_DCOM_X=DCx(ind1:ind2);
KAJ_FOOT=[PF];
KAJ_COM_X=resample(KAJ_COM_X,size(PF(1,:),2),size(KAJ_COM_X,1),0);
KAJ_COM_Y=resample(KAJ_COM_X*0+Com_h,size(PF(1,:),2),size(KAJ_COM_X,1),0);

KAJ_COM=[KAJ_COM_X';KAJ_COM_Y';KAJ_COM_X'*0];

%% Inverse Dynamic 

TS=diff(Res_T_SP);
Ts=TS(1);

Nsolver=10;

[ERRF,ERRC,Q]=solve_optimization(KAJ_COM,KAJ_FOOT,Ts,Nsolver);

% Check the errors referred to the desired signals provided by the NO
cond1=max(max(abs(ERRF)))>0.0085;
cond2=max(max(abs(ERRC(1,:))))>0.015;
cond3=max(max(abs(ERRC(2,:))))>0.02;
 
if ( cond1 || cond2 || cond3 )
%     input('')
    if cond1
        disp('Foot>0.0085')
    elseif cond2
        disp('CoMx>0.01')
    elseif cond3
        disp('COMy>0.02')
    end
   
   disp([' Sim not succeeded ', num2str(sim_i)])
   clear PFdes PCdes
   continue
end

% Calculate the torque vector
q_=Q';
Dq_=[zeros(1,6);diff(q_)]./(Ts);
DDq_=[zeros(2,6);diff(q_,2)]./(Ts^2);

%Dynamic parameters

LinkFemur=0.12;
CMFemur=0.01;
CMFemur2=LinkFemur-CMFemur;
LinkTibia=0.18;
CMTibia=0.125;
CMTibia2=LinkTibia-CMTibia;
LinkPelvis=0.1;
CMPelvis=0.1;
CMTrunk=0;
LinkTrunk=0;
CMFoot=0.05;
LinkFoot=CMFoot;

params=[ LinkPelvis CMPelvis CMTrunk LinkTrunk CMFoot LinkFoot...
   MassFemur MassTibia MassTibia2 MassPelvis MassTrunk MassFoot...
   LinkFemur CMFemur CMFemur2 LinkTibia CMTibia CMTibia2];

    n=6;
    Fv = eye(n)*0.01; %friction
    theta_=[];

%Dynamics

for iq=1:size(q_,1)

    M = M_fun(q_(iq,:),Dq_(iq,:),params);
    G = G_fun(q_(iq,:),Dq_(iq,:),params);
    C = C_fun(q_(iq,:),Dq_(iq,:),params);
    
    tau_ = M * DDq_(iq,:)' + C*Dq_(iq,:)' + Fv*Dq_(iq,:)' + G;
    
    K=60;
    theta=tau_/K+q_(iq,:)';
    
    %obtain the motor reference
    theta_=[theta_;theta'];
end    

Dtheta_=[zeros(1,6); diff(theta_)]./(Ts);
DDtheta_=[zeros(2,6);diff(theta_,2)]./(Ts^2);

%% Calculate MPC COT


J_mot = 5.55e-7; % Inertia
red = 205; % reduction ratio
damping=0.3;
%Joint stiffness value
K=60;
el_term=K*(theta_'-q_');

tau_sot = J_mot*red^2*DDtheta_' + damping*Dtheta_' + el_term;

% New COT for MPC KAJITA
NEW_J_SP_SOT=sum(sum((tau_sot.^2)*(Ts))/(m_robot*g*Step_L));

% Store the results 
% JMAT matrix of Index cost J 
% FMIN_RES all the resultant signals that I consider useful for further analysis

JMAT=[JMAT;2*walk_dist/T NEW_J_SP NEW_J_SP_SOT];
new_time=NaN;

v_fim_res={NEW_J_SP_SOT, q_,Dq_,DDq_,tau_sot,new_time,Tau_SP,res_T_real,2*walk_dist/T,foot_h,KAJ_COM,KAJ_FOOT};
FMIN_RES=[FMIN_RES;v_fim_res];

end %end for sim_i 

% supp=['_1'];
supp=[];

save([FLD_BASE,'FMIN_RES',supp],'FMIN_RES')
save([FLD_BASE,'JMAT',supp],'JMAT')

%% Plot results

if isempty(JMAT)
    disp('No results')
    return
end

figure('Name','COT')
plot(JMAT(:,1),JMAT(:,2),'o','LineWidth',2)
hold on
plot(JMAT(:,1),JMAT(:,3),'o','LineWidth',2)
xx=xlim
legend('NO','ZMP')
set(gca,'FontSize',18)
xlabel('Speed [m/s]')
ylabel('CoT')
grid on
xlim(xx)
saveas(gcf,[FLD_BASE,'ZMP_NO_Comparison_results'],'fig')
saveas(gcf,[FLD_BASE,'ZMP_NO_Comparison_results'],'png')

%%
M=[];
for i=Vel_range
   V=find(abs(JMAT(:,1)-i)<=eps);
   if isempty(V)
   else
       m1=mean(JMAT(V,2));
       m2=mean(JMAT(V,3));
       M=[M;i,m1,m2];
   end
end

figure('Name','COT')

plot(M(:,1),M(:,3),'-d','LineWidth',2,'MarkerSize',8)
hold on
plot(M(:,1),M(:,2),'-o','LineWidth',2)
xx=xlim
legend('ZMP','NO','Location','northwest')
set(gca,'FontSize',18)
xlabel('Speed [m/s]')
ylabel('CoT')
grid on
xlim([M(1,1) M(end,1)])
saveas(gcf,[FLD_BASE,'ZMP_NO_Comparison_results_mean'],'fig')
saveas(gcf,[FLD_BASE,'ZMP_NO_Comparison_results_mean'],'png')

V=[];
tau_lim=10;
for i=1:size(FMIN_RES,1)
   if max(max(abs(FMIN_RES{i,5})))<=tau_lim
       
       V=[V; i FMIN_RES{i,9} FMIN_RES{i,10}];
   end
end

if isempty(V)
    disp(['No simulations with torques minor than ',num2str(tau_lim), 'N'])
    return
end

V2=sortrows(V,[-2 -3]);
sim_i=V2(1);

tau_sot=FMIN_RES{sim_i,5};
Tau_SP=FMIN_RES{sim_i,7};
time=FMIN_RES{sim_i,8}(1:29);
q_=FMIN_RES{sim_i,2};
Dq_=FMIN_RES{sim_i,3};
DDq_=FMIN_RES{sim_i,4};
        
 STR_title={'Stance Hip','Stance Knee','Stance Ankle','Swing Hip','Swing Knee','Swing Ankle'};             
 
 %joint number
 nj=6;
 
 figure
for i=1:nj
    
subplot(2,3,i)

if i<=3
    
plot(time,tau_sot(3-i+1,1:end).^2,'LineWidth',2,'Marker','d','MarkerSize',8)

else
    plot(time,tau_sot(i,1:end).^2,'LineWidth',2,'Marker','d','MarkerSize',8)
end
   hold on 
plot(time,Tau_SP(i,:).^2,'LineWidth',2,'Marker','o')
title(STR_title{i})
xlim([time(1) time(end)])
grid on
set(gca,'FontSize',18)

if i>3
xlabel('Time [s]')
end
if i==1 || i==4
ylabel('\tau^2 [Nm^2]')
legend('ZMP','NO','Location','northwest')
end
end


saveas(gcf,[FLD_BASE,'ZMP_NO_Comparison_results_joints'],'fig')

toc