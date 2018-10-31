
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

for i=1:30
GRAV=cfun(res_q(:, i), zeros( MBS_data_G.Njoint,1), zeros(6,1) );
% ALL_CG=cfun(res_q(:, i), res_qd(:, i), zeros(6,1) );
CQD=cfun(res_q(:, i), res_qd(:, i), zeros(6,1) )-GRAV;

ALL_CG=cfun(res_q(:, i), res_qd(:, i), res_Fc(:,i)' );
ALL_MQDD=Mfun(res_q(:,i))*res_qdd(:,i);

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





