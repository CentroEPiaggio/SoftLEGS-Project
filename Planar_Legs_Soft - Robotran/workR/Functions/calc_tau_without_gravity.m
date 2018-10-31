
% G_Struct=struct('q',{res_q(:,1:end-1)},'qd',{res_qd(:,1:end-1)},'qdd',{res_qdd(:,1:end)},'g',{0,0,0},'frc',{res_FC()})

G_MBS_data=MBS_data;
G_MBS_data.q=res_q(:,1);
G_MBS_data.qd=res_qd(:,1);
G_MBS_data.qdd=res_qdd(:,1);
% G_MBS_data.g=[0;0;0];
%%

% [frc,trq] = mbs_extforces_Planar_Legs_casadi(MBS_data,'void','void')
i=1;
q_1=res_q(i,1);
q_2=res_q(i,2);
q_3=res_q(i,3);
q_4=res_q(i,4);
q_5=res_q(i,5);
q_6=res_q(i,6);
q_7=res_q(i,7);
q_8=res_q(i,8);
q_9=res_q(i,9);

F1=0; F2=cos(q_5); F3=cos(q_4); F4=cos(q_2); F5=cos(q_3);
F6=sin(q_2); F7=sin(q_3); F8=((F4*F5)-(F6*F7)); F9=sin(q_4);
F10=((F4*F7)+(F6*F5)); F11=((F3*F8)-(F9*F10)); F12=sin(q_5);
F13=((F3*F10)+(F9*F8)); F14=((F2*F11)-(F12*F13)); F15=((F2*F13)+(F12*F11));
F16=-9.80526e-12; F17=cos(q_8); F18=cos(q_7); F19=cos(q_6); F20=sin(q_6);
F21=((F4*F19)-(F6*F20)); F22=sin(q_7); F23=((F4*F20)+(F6*F19));
F24=((F18*F21)-(F22*F23)); F25=sin(q_8); F26=((F18*F23)+(F22*F21));
F27=((F17*F24)-(F25*F26)); F28=((F17*F26)+(F25*F24));

f_ExtForce_Left_0=res_Fc(i,1);
f_ExtForce_Left_1=res_Fc(i,2);
f_ExtForce_Right_0=res_Fc(i,4);
f_ExtForce_Right_1=res_Fc(i,5);

%%
FRC=[F1, F1, F1, F1, F1, F1, F1, F1, F1;
 F1, F1, F1, F1, F1, (((f_ExtForce_Left_0*F14)+(f_ExtForce_Left_1*F15))+(f_ExtForce_Left_0+(F16*f_ExtForce_Left_1))), F1, F1, ((f_ExtForce_Right_0*F27)+(f_ExtForce_Right_1*F28)); 
 F1, F1, F1, F1, F1, (-(((f_ExtForce_Left_0*F15)-(f_ExtForce_Left_1*F14))+((F16*f_ExtForce_Left_0)-f_ExtForce_Left_1))), F1, F1, (-((f_ExtForce_Right_0*F28)-(f_ExtForce_Right_1*F27)))];

T1=0; T2=-0.024; T3=cos(q_5); T4=cos(q_4); T5=cos(q_2); T6=cos(q_3);
T7=sin(q_2); T8=sin(q_3); T9=((T5*T6)-(T7*T8)); T10=sin(q_4); 
T11=((T5*T8)+(T7*T6)); T12=((T4*T9)-(T10*T11)); T13=sin(q_5);
T14=((T4*T11)+(T10*T9)); T15=cos(q_8); T16=cos(q_7); T17=cos(q_6);
T18=sin(q_6); T19=((T5*T17)-(T7*T18)); T20=sin(q_7);
T21=((T5*T18)+(T7*T17)); T22=((T16*T19)-(T20*T21)); 
T23=sin(q_8); T24=((T16*T21)+(T20*T19));

M_ExtForce_Left=res_Fc(i,3);
M_ExtForce_Right=res_Fc(i,6);

TRC=[T1, T1, T1, T1, T1, (((M_ExtForce_Left-(T2*((f_ExtForce_Left_0*((T3*T12)-(T13*T14)))+(f_ExtForce_Left_1*((T3*T14)+(T13*T12))))))+M_ExtForce_Left)-(T2*(f_ExtForce_Left_0+(-9.80526e-12*f_ExtForce_Left_1)))), T1, T1, (M_ExtForce_Right-(T2*((f_ExtForce_Right_0*((T15*T22)-(T23*T24)))+(f_ExtForce_Right_1*((T15*T24)+(T23*T22)))))); 
 T1, T1, T1, T1, T1, T1, T1, T1, T1; 
 T1, T1, T1, T1, T1, T1, T1, T1, T1];


%%
FRC=zeros(3,9);
TRQ=zeros(3,9);
FRC=
G_MBS_data.frc=res_q(:,1);
%%

[QqLHS] = mbs_invdyna_Planar_Legs(G_Struct,'void','void')