
MBS_data_G=MBS_data;
import casadi.*
[M, c] = mbs_dirdyna_Planar_Legs_casadi(MBS_data_G,'void','void');
MBS_data_G.M = cell2mat_casadi(M);
MBS_data_G.c = cell2mat_casadi(c);

Mfun = Function('Mfun',{q},{MBS_data_G.M});
cfun = Function('cfun', {q, qd, SFor},{MBS_data_G.c});

NTAU_G=[];
NTAU=[];
NTAU_MC=[];

for i=2:31 %PRIMA ERA 1:10
GRAV=cfun(res_q(:, i), zeros( MBS_data_G.Njoint,1), zeros(6,1) );
% ALL_CG=cfun(res_q(:, i), res_qd(:, i), zeros(6,1) );
CQD=cfun(res_q(:, i), res_qd(:, i), zeros(6,1) )-GRAV;

ALL_CG=cfun(res_q(:, i), res_qd(:, i), res_Fc(:,i-1)' );
ALL_MQDD=Mfun(res_q(:,i))*res_qdd(:,i-1); %PRIMA ERA i

NTAU=[NTAU,full(ALL_MQDD)+full(ALL_CG)];
NTAU_G=[NTAU_G,full(GRAV)];

NTAU_MC=[NTAU_MC,full(ALL_MQDD)+full(CQD)];

end

ntau_g=NTAU_G(4:9,:);
ntau=NTAU(4:9,:);
ntau_mc=NTAU_MC(4:9,:);

nt_tg=ntau-ntau_g;

NT_TG=zeros(6,1);
NT=zeros(6,1);
NTG=zeros(6,1);
NTMC=zeros(6,1);

for i=1:6
NT_TG(i)=norm(nt_tg(i,:),2);
NT(i)=norm(ntau(i,:),2);

NTG(i)=norm(ntau_g(i,:),2);
NTMC(i)=norm(ntau_mc(i,:),2);
end

save([name_sim_folder,'/taug_taumc'],'NT','NTG','NTMC','NT_TG','ntau','ntau_g','ntau_mc','nt_tg')

%%
J_mot = 5.55e-7; % kg m^2
red = 205; % reduction ratio
b = 9.18e-7; % Nm/(rad/s)      damping
% 
% dynAct = J_mot*red^2*res_thetadd + damping*res_thetad + el_term - inputs;

tau_el=60*(res_theta(:,2:31)-res_q(4:end,2:31));
tau_jb=J_mot*red^2*res_thetadd(:,1:30) + damping*res_thetad(:,2:31);

NT_EL=zeros(6,1);
NT_JB=zeros(6,1);

for i=1:6
    
NT_EL(i)=norm(tau_el(i,:),2);
NT_JB(i)=norm(tau_jb(i,:),2);

end


save([name_sim_folder,'/tauel_taujb'],'NT_EL','NT_JB','tau_el','tau_jb','tau')

