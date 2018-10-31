function G = G_fun( q , Dq , params)

g=-9.81;
LinkPelvis=params(1);
CMPelvis=params(2);
CMTrunk=params(3);
LinkTrunk=params(4);
CMFoot=params(5);
LinkFoot=params(6);
MassFemur=params(7);
MassTibia=params(8);
MassTibia2=params(9);
MassPelvis=params(10);
MassTrunk=params(11);
MassFoot=params(12);
LinkFemur=params(13);
CMFemur=params(14);
CMFemur2=params(15);
LinkTibia=params(16);
CMTibia=params(17);
CMTibia2=params(18);

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

Dq1=Dq(1);
Dq2=Dq(2);
Dq3=Dq(3);
Dq4=Dq(4);
Dq5=Dq(5);
Dq6=Dq(6);

G=[g*m6*(a6*sin(q1 + q2 + q3 - q4 - q5 - q6) - l2*sin(q1 + q2) + l5*sin(q1 + q2 + q3 - q4 - q5) - l1*sin(q1) + l4*sin(q1 + q2 + q3 - q4)) - g*m5*(l2*sin(q1 + q2) - a5*sin(q1 + q2 + q3 - q4 - q5) + l1*sin(q1) - l4*sin(q1 + q2 + q3 - q4)) - g*m4*(l2*sin(q1 + q2) + l1*sin(q1) - a4*sin(q1 + q2 + q3 - q4)) - g*m7*(l2*sin(q1 + q2) + l1*sin(q1) + l3*sin(q1 + q2 + q3) + l7*sin(q1 + q2 + q3)) - g*m3*(l2*sin(q1 + q2) + l1*sin(q1) + a3*sin(q1 + q2 + q3)) - g*m2*(a2*sin(q1 + q2) + l1*sin(q1)) - a1*g*m1*sin(q1),  ; g*m5*(a5*sin(q1 + q2 + q3 - q4 - q5) - l2*sin(q1 + q2) + l4*sin(q1 + q2 + q3 - q4)) + g*m6*(a6*sin(q1 + q2 + q3 - q4 - q5 - q6) - l2*sin(q1 + q2) + l5*sin(q1 + q2 + q3 - q4 - q5) + l4*sin(q1 + q2 + q3 - q4)) - g*m4*(l2*sin(q1 + q2) - a4*sin(q1 + q2 + q3 - q4)) - g*m7*(l2*sin(q1 + q2) + l3*sin(q1 + q2 + q3) + l7*sin(q1 + q2 + q3)) - g*m3*(l2*sin(q1 + q2) + a3*sin(q1 + q2 + q3)) - a2*g*m2*sin(q1 + q2),  ; g*m6*(a6*sin(q1 + q2 + q3 - q4 - q5 - q6) + l5*sin(q1 + q2 + q3 - q4 - q5) + l4*sin(q1 + q2 + q3 - q4)) + g*m5*(a5*sin(q1 + q2 + q3 - q4 - q5) + l4*sin(q1 + q2 + q3 - q4)) + a4*g*m4*sin(q1 + q2 + q3 - q4) - g*m7*sin(q1 + q2 + q3)*(l3 + l7) - a3*g*m3*sin(q1 + q2 + q3),  ; - g*m6*(a6*sin(q1 + q2 + q3 - q4 - q5 - q6) + l5*sin(q1 + q2 + q3 - q4 - q5) + l4*sin(q1 + q2 + q3 - q4)) - g*m5*(a5*sin(q1 + q2 + q3 - q4 - q5) + l4*sin(q1 + q2 + q3 - q4)) - a4*g*m4*sin(q1 + q2 + q3 - q4),  ; - g*m6*(a6*sin(q1 + q2 + q3 - q4 - q5 - q6) + l5*sin(q1 + q2 + q3 - q4 - q5)) - a5*g*m5*sin(q1 + q2 + q3 - q4 - q5),  ; -a6*g*m6*sin(q1 + q2 + q3 - q4 - q5 - q6),  ; ];
 end