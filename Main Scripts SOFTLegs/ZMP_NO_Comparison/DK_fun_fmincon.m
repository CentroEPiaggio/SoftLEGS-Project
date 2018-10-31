function [PF,PC] = DK_fun_fmincon(q)

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

MassFemur=0.7;
MassTibia=1;
MassTibia2=1; % 0.25 is the QBmove weight
MassPelvis=2.7;
MassTrunk= 0;
MassFoot=0.55;

l1=LinkTibia;
l2=LinkFemur;
l3=LinkPelvis;
l4=LinkFemur;
l5=LinkTibia;
l6=LinkFoot;
l7=LinkTrunk;
a1=CMTibia;
a2=CMFemur;
a3=CMPelvis;
a4=CMFemur2;
a5=CMTibia2;
a6=CMFoot;
m1=MassTibia;
m2=MassFemur;
m3=MassPelvis;
m4=MassFemur;
m5=MassTibia;
m6=MassFoot;
m7=MassTrunk;

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);

PF=[l2*sin(q1 + q2) - a6*sin(q1 + q2 + q3 - q4 - q5 - q6) - l5*sin(q1 + q2 + q3 - q4 - q5) + l1*sin(q1) - l4*sin(q1 + q2 + q3 - q4),  ; l2*cos(q1 + q2) - a6*cos(q1 + q2 + q3 - q4 - q5 - q6) - l5*cos(q1 + q2 + q3 - q4 - q5) + l1*cos(q1) - l4*cos(q1 + q2 + q3 - q4),  ; q1 + q2 + q3 - q4 - q5 - q6,  ; ];

PC=[(m4*(l2*sin(q1 + q2) + l1*sin(q1) - a4*sin(q1 + q2 + q3 - q4)) - m6*(a6*sin(q1 + q2 + q3 - q4 - q5 - q6) - l2*sin(q1 + q2) + l5*sin(q1 + q2 + q3 - q4 - q5) - l1*sin(q1) + l4*sin(q1 + q2 + q3 - q4)) + m7*(l2*sin(q1 + q2) + l1*sin(q1) + l3*sin(q1 + q2 + q3) + l7*sin(q1 + q2 + q3)) + m3*(l2*sin(q1 + q2) + l1*sin(q1) + a3*sin(q1 + q2 + q3)) + m2*(a2*sin(q1 + q2) + l1*sin(q1)) + m5*(l2*sin(q1 + q2) - a5*sin(q1 + q2 + q3 - q4 - q5) + l1*sin(q1) - l4*sin(q1 + q2 + q3 - q4)) + a1*m1*sin(q1))/(m1 + m2 + m3 + m4 + m5 + m6 + m7),  ; (m3*(l2*cos(q1 + q2) + l1*cos(q1) + a3*cos(q1 + q2 + q3)) + m2*(a2*cos(q1 + q2) + l1*cos(q1)) + m5*(l2*cos(q1 + q2) - a5*cos(q1 + q2 + q3 - q4 - q5) + l1*cos(q1) - l4*cos(q1 + q2 + q3 - q4)) - m6*(a6*cos(q1 + q2 + q3 - q4 - q5 - q6) - l2*cos(q1 + q2) + l5*cos(q1 + q2 + q3 - q4 - q5) - l1*cos(q1) + l4*cos(q1 + q2 + q3 - q4)) + m4*(l2*cos(q1 + q2) + l1*cos(q1) - a4*cos(q1 + q2 + q3 - q4)) + m7*(l2*cos(q1 + q2) + l1*cos(q1) + l3*cos(q1 + q2 + q3) + l7*cos(q1 + q2 + q3)) + a1*m1*cos(q1))/(m1 + m2 + m3 + m4 + m5 + m6 + m7),  ; (m2*(q1 + q2) + m5*(q1 + q2 + q3 - q4 - q5) + m1*q1 + m4*(q1 + q2 + q3 - q4) + m3*(q1 + q2 + q3) + m7*(q1 + q2 + q3) + m6*(q1 + q2 + q3 - q4 - q5 - q6))/(m1 + m2 + m3 + m4 + m5 + m6 + m7),  ; ];
 
PF(2)=PF(2)+0.05;
end