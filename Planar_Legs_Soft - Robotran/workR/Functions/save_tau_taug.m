TAU_G=[];
for i=1:30
G_MBS_data=MBS_data;
G_MBS_data.q=res_q(:,i);
G_MBS_data.qd=res_qd(:,i)*0;
G_MBS_data.qdd=res_qdd(:,i)*0;

G_MBS_data.frc=zeros(3,9);
G_MBS_data.trq=zeros(3,9);


[MG,cG] = mbs_dirdyna_Planar_Legs(G_MBS_data,'void','void');

TAU_G=[TAU_G,cG];
end

tau_g=TAU_G(4:end,:);

NT=[];
NTG=[];

for i=1:6
NT=[NT;norm(tau(i,:),2)];
NTG=[NTG;norm(tau_g(i,:),2)];
end

save([name_sim_folder,'tau_tau_g'],'NT','NTG','tau','tau_g')