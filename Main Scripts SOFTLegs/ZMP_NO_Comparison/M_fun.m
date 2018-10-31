function M = M_fun( q , Dq , params)

g=9.81;
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

M=[a1^2*m1 + a2^2*m2 + a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l1^2*m2 + l1^2*m3 + l1^2*m4 + l2^2*m3 + l1^2*m5 + l2^2*m4 + l1^2*m6 + l2^2*m5 + l1^2*m7 + l2^2*m6 + l2^2*m7 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 - 2*a4*l1*m4*cos(q2 + q3 - q4) - 2*a5*l2*m5*cos(q4 - q3 + q5) - 2*l1*l4*m5*cos(q2 + q3 - q4) - 2*l1*l4*m6*cos(q2 + q3 - q4) - 2*l2*l5*m6*cos(q4 - q3 + q5) + 2*a3*l1*m3*cos(q2 + q3) + 2*a6*l4*m6*cos(q5 + q6) + 2*l1*l3*m7*cos(q2 + q3) + 2*l1*l7*m7*cos(q2 + q3) - 2*a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) + 2*a2*l1*m2*cos(q2) + 2*a3*l2*m3*cos(q3) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + 2*l1*l2*m3*cos(q2) + 2*l1*l2*m4*cos(q2) + 2*l1*l2*m5*cos(q2) + 2*l1*l2*m6*cos(q2) + 2*l1*l2*m7*cos(q2) + 2*l2*l3*m7*cos(q3) + 2*l2*l7*m7*cos(q3) + 2*l4*l5*m6*cos(q5) - 2*a6*l2*m6*cos(q4 - q3 + q5 + q6) - 2*a4*l2*m4*cos(q3 - q4) - 2*l2*l4*m5*cos(q3 - q4) - 2*l2*l4*m6*cos(q3 - q4) - 2*a5*l1*m5*cos(q2 + q3 - q4 - q5) - 2*l1*l5*m6*cos(q2 + q3 - q4 - q5), a2^2*m2 + a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l2^2*m3 + l2^2*m4 + l2^2*m5 + l2^2*m6 + l2^2*m7 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 - a4*l1*m4*cos(q2 + q3 - q4) - 2*a5*l2*m5*cos(q4 - q3 + q5) - l1*l4*m5*cos(q2 + q3 - q4) - l1*l4*m6*cos(q2 + q3 - q4) - 2*l2*l5*m6*cos(q4 - q3 + q5) + a3*l1*m3*cos(q2 + q3) + 2*a6*l4*m6*cos(q5 + q6) + l1*l3*m7*cos(q2 + q3) + l1*l7*m7*cos(q2 + q3) - a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) + a2*l1*m2*cos(q2) + 2*a3*l2*m3*cos(q3) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + l1*l2*m3*cos(q2) + l1*l2*m4*cos(q2) + l1*l2*m5*cos(q2) + l1*l2*m6*cos(q2) + l1*l2*m7*cos(q2) + 2*l2*l3*m7*cos(q3) + 2*l2*l7*m7*cos(q3) + 2*l4*l5*m6*cos(q5) - 2*a6*l2*m6*cos(q4 - q3 + q5 + q6) - 2*a4*l2*m4*cos(q3 - q4) - 2*l2*l4*m5*cos(q3 - q4) - 2*l2*l4*m6*cos(q3 - q4) - a5*l1*m5*cos(q2 + q3 - q4 - q5) - l1*l5*m6*cos(q2 + q3 - q4 - q5), a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 - a4*l1*m4*cos(q2 + q3 - q4) - a5*l2*m5*cos(q4 - q3 + q5) - l1*l4*m5*cos(q2 + q3 - q4) - l1*l4*m6*cos(q2 + q3 - q4) - l2*l5*m6*cos(q4 - q3 + q5) + a3*l1*m3*cos(q2 + q3) + 2*a6*l4*m6*cos(q5 + q6) + l1*l3*m7*cos(q2 + q3) + l1*l7*m7*cos(q2 + q3) - a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) + a3*l2*m3*cos(q3) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + l2*l3*m7*cos(q3) + l2*l7*m7*cos(q3) + 2*l4*l5*m6*cos(q5) - a6*l2*m6*cos(q4 - q3 + q5 + q6) - a4*l2*m4*cos(q3 - q4) - l2*l4*m5*cos(q3 - q4) - l2*l4*m6*cos(q3 - q4) - a5*l1*m5*cos(q2 + q3 - q4 - q5) - l1*l5*m6*cos(q2 + q3 - q4 - q5), a4*l1*m4*cos(q2 + q3 - q4) - a5^2*m5 - a6^2*m6 - l4^2*m5 - l4^2*m6 - l5^2*m6 - a4^2*m4 + a5*l2*m5*cos(q4 - q3 + q5) + l1*l4*m5*cos(q2 + q3 - q4) + l1*l4*m6*cos(q2 + q3 - q4) + l2*l5*m6*cos(q4 - q3 + q5) - 2*a6*l4*m6*cos(q5 + q6) + a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) - 2*a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - 2*l4*l5*m6*cos(q5) + a6*l2*m6*cos(q4 - q3 + q5 + q6) + a4*l2*m4*cos(q3 - q4) + l2*l4*m5*cos(q3 - q4) + l2*l4*m6*cos(q3 - q4) + a5*l1*m5*cos(q2 + q3 - q4 - q5) + l1*l5*m6*cos(q2 + q3 - q4 - q5), a5*l2*m5*cos(q4 - q3 + q5) - a6^2*m6 - l5^2*m6 - a5^2*m5 + l2*l5*m6*cos(q4 - q3 + q5) - a6*l4*m6*cos(q5 + q6) + a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) - a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - l4*l5*m6*cos(q5) + a6*l2*m6*cos(q4 - q3 + q5 + q6) + a5*l1*m5*cos(q2 + q3 - q4 - q5) + l1*l5*m6*cos(q2 + q3 - q4 - q5), -a6*m6*(a6 + l4*cos(q5 + q6) - l1*cos(q4 - q3 - q2 + q5 + q6) + l5*cos(q6) - l2*cos(q4 - q3 + q5 + q6)),  ; a2^2*m2 + a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l2^2*m3 + l2^2*m4 + l2^2*m5 + l2^2*m6 + l2^2*m7 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 - a4*l1*m4*cos(q2 + q3 - q4) - 2*a5*l2*m5*cos(q4 - q3 + q5) - l1*l4*m5*cos(q2 + q3 - q4) - l1*l4*m6*cos(q2 + q3 - q4) - 2*l2*l5*m6*cos(q4 - q3 + q5) + a3*l1*m3*cos(q2 + q3) + 2*a6*l4*m6*cos(q5 + q6) + l1*l3*m7*cos(q2 + q3) + l1*l7*m7*cos(q2 + q3) - a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) + a2*l1*m2*cos(q2) + 2*a3*l2*m3*cos(q3) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + l1*l2*m3*cos(q2) + l1*l2*m4*cos(q2) + l1*l2*m5*cos(q2) + l1*l2*m6*cos(q2) + l1*l2*m7*cos(q2) + 2*l2*l3*m7*cos(q3) + 2*l2*l7*m7*cos(q3) + 2*l4*l5*m6*cos(q5) - 2*a6*l2*m6*cos(q4 - q3 + q5 + q6) - 2*a4*l2*m4*cos(q3 - q4) - 2*l2*l4*m5*cos(q3 - q4) - 2*l2*l4*m6*cos(q3 - q4) - a5*l1*m5*cos(q2 + q3 - q4 - q5) - l1*l5*m6*cos(q2 + q3 - q4 - q5), a2^2*m2 + a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l2^2*m3 + l2^2*m4 + l2^2*m5 + l2^2*m6 + l2^2*m7 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 - 2*a5*l2*m5*cos(q4 - q3 + q5) - 2*l2*l5*m6*cos(q4 - q3 + q5) + 2*a6*l4*m6*cos(q5 + q6) + 2*a3*l2*m3*cos(q3) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + 2*l2*l3*m7*cos(q3) + 2*l2*l7*m7*cos(q3) + 2*l4*l5*m6*cos(q5) - 2*a6*l2*m6*cos(q4 - q3 + q5 + q6) - 2*a4*l2*m4*cos(q3 - q4) - 2*l2*l4*m5*cos(q3 - q4) - 2*l2*l4*m6*cos(q3 - q4), a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 - a5*l2*m5*cos(q4 - q3 + q5) - l2*l5*m6*cos(q4 - q3 + q5) + 2*a6*l4*m6*cos(q5 + q6) + a3*l2*m3*cos(q3) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + l2*l3*m7*cos(q3) + l2*l7*m7*cos(q3) + 2*l4*l5*m6*cos(q5) - a6*l2*m6*cos(q4 - q3 + q5 + q6) - a4*l2*m4*cos(q3 - q4) - l2*l4*m5*cos(q3 - q4) - l2*l4*m6*cos(q3 - q4), a5*l2*m5*cos(q4 - q3 + q5) - a5^2*m5 - a6^2*m6 - l4^2*m5 - l4^2*m6 - l5^2*m6 - a4^2*m4 + l2*l5*m6*cos(q4 - q3 + q5) - 2*a6*l4*m6*cos(q5 + q6) - 2*a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - 2*l4*l5*m6*cos(q5) + a6*l2*m6*cos(q4 - q3 + q5 + q6) + a4*l2*m4*cos(q3 - q4) + l2*l4*m5*cos(q3 - q4) + l2*l4*m6*cos(q3 - q4), a5*l2*m5*cos(q4 - q3 + q5) - a6^2*m6 - l5^2*m6 - a5^2*m5 + l2*l5*m6*cos(q4 - q3 + q5) - a6*l4*m6*cos(q5 + q6) - a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - l4*l5*m6*cos(q5) + a6*l2*m6*cos(q4 - q3 + q5 + q6), -a6*m6*(a6 + l4*cos(q5 + q6) + l5*cos(q6) - l2*cos(q4 - q3 + q5 + q6)),  ; a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 - a4*l1*m4*cos(q2 + q3 - q4) - a5*l2*m5*cos(q4 - q3 + q5) - l1*l4*m5*cos(q2 + q3 - q4) - l1*l4*m6*cos(q2 + q3 - q4) - l2*l5*m6*cos(q4 - q3 + q5) + a3*l1*m3*cos(q2 + q3) + 2*a6*l4*m6*cos(q5 + q6) + l1*l3*m7*cos(q2 + q3) + l1*l7*m7*cos(q2 + q3) - a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) + a3*l2*m3*cos(q3) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + l2*l3*m7*cos(q3) + l2*l7*m7*cos(q3) + 2*l4*l5*m6*cos(q5) - a6*l2*m6*cos(q4 - q3 + q5 + q6) - a4*l2*m4*cos(q3 - q4) - l2*l4*m5*cos(q3 - q4) - l2*l4*m6*cos(q3 - q4) - a5*l1*m5*cos(q2 + q3 - q4 - q5) - l1*l5*m6*cos(q2 + q3 - q4 - q5), a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 - a5*l2*m5*cos(q4 - q3 + q5) - l2*l5*m6*cos(q4 - q3 + q5) + 2*a6*l4*m6*cos(q5 + q6) + a3*l2*m3*cos(q3) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + l2*l3*m7*cos(q3) + l2*l7*m7*cos(q3) + 2*l4*l5*m6*cos(q5) - a6*l2*m6*cos(q4 - q3 + q5 + q6) - a4*l2*m4*cos(q3 - q4) - l2*l4*m5*cos(q3 - q4) - l2*l4*m6*cos(q3 - q4), a3^2*m3 + a4^2*m4 + a5^2*m5 + a6^2*m6 + l4^2*m5 + l3^2*m7 + l4^2*m6 + l5^2*m6 + l7^2*m7 + 2*l3*l7*m7 + 2*a6*l4*m6*cos(q5 + q6) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + 2*l4*l5*m6*cos(q5), - a4^2*m4 - a5^2*m5 - a6^2*m6 - l4^2*m5 - l4^2*m6 - l5^2*m6 - 2*a6*l4*m6*cos(q5 + q6) - 2*a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - 2*l4*l5*m6*cos(q5), - a5^2*m5 - a6^2*m6 - l5^2*m6 - a6*l4*m6*cos(q5 + q6) - a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - l4*l5*m6*cos(q5), -a6*m6*(a6 + l4*cos(q5 + q6) + l5*cos(q6)),  ; a4*l1*m4*cos(q2 + q3 - q4) - a5^2*m5 - a6^2*m6 - l4^2*m5 - l4^2*m6 - l5^2*m6 - a4^2*m4 + a5*l2*m5*cos(q4 - q3 + q5) + l1*l4*m5*cos(q2 + q3 - q4) + l1*l4*m6*cos(q2 + q3 - q4) + l2*l5*m6*cos(q4 - q3 + q5) - 2*a6*l4*m6*cos(q5 + q6) + a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) - 2*a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - 2*l4*l5*m6*cos(q5) + a6*l2*m6*cos(q4 - q3 + q5 + q6) + a4*l2*m4*cos(q3 - q4) + l2*l4*m5*cos(q3 - q4) + l2*l4*m6*cos(q3 - q4) + a5*l1*m5*cos(q2 + q3 - q4 - q5) + l1*l5*m6*cos(q2 + q3 - q4 - q5), a5*l2*m5*cos(q4 - q3 + q5) - a5^2*m5 - a6^2*m6 - l4^2*m5 - l4^2*m6 - l5^2*m6 - a4^2*m4 + l2*l5*m6*cos(q4 - q3 + q5) - 2*a6*l4*m6*cos(q5 + q6) - 2*a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - 2*l4*l5*m6*cos(q5) + a6*l2*m6*cos(q4 - q3 + q5 + q6) + a4*l2*m4*cos(q3 - q4) + l2*l4*m5*cos(q3 - q4) + l2*l4*m6*cos(q3 - q4), - a4^2*m4 - a5^2*m5 - a6^2*m6 - l4^2*m5 - l4^2*m6 - l5^2*m6 - 2*a6*l4*m6*cos(q5 + q6) - 2*a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - 2*l4*l5*m6*cos(q5), a4^2*m4 + a5^2*m5 + a6^2*m6 + l4^2*m5 + l4^2*m6 + l5^2*m6 + 2*a6*l4*m6*cos(q5 + q6) + 2*a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + 2*l4*l5*m6*cos(q5), a5^2*m5 + a6^2*m6 + l5^2*m6 + a6*l4*m6*cos(q5 + q6) + a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + l4*l5*m6*cos(q5), a6*m6*(a6 + l4*cos(q5 + q6) + l5*cos(q6)),  ; a5*l2*m5*cos(q4 - q3 + q5) - a6^2*m6 - l5^2*m6 - a5^2*m5 + l2*l5*m6*cos(q4 - q3 + q5) - a6*l4*m6*cos(q5 + q6) + a6*l1*m6*cos(q4 - q3 - q2 + q5 + q6) - a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - l4*l5*m6*cos(q5) + a6*l2*m6*cos(q4 - q3 + q5 + q6) + a5*l1*m5*cos(q2 + q3 - q4 - q5) + l1*l5*m6*cos(q2 + q3 - q4 - q5), a5*l2*m5*cos(q4 - q3 + q5) - a6^2*m6 - l5^2*m6 - a5^2*m5 + l2*l5*m6*cos(q4 - q3 + q5) - a6*l4*m6*cos(q5 + q6) - a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - l4*l5*m6*cos(q5) + a6*l2*m6*cos(q4 - q3 + q5 + q6), - a5^2*m5 - a6^2*m6 - l5^2*m6 - a6*l4*m6*cos(q5 + q6) - a5*l4*m5*cos(q5) - 2*a6*l5*m6*cos(q6) - l4*l5*m6*cos(q5), a5^2*m5 + a6^2*m6 + l5^2*m6 + a6*l4*m6*cos(q5 + q6) + a5*l4*m5*cos(q5) + 2*a6*l5*m6*cos(q6) + l4*l5*m6*cos(q5), a5^2*m5 + a6^2*m6 + l5^2*m6 + 2*a6*l5*m6*cos(q6), a6*m6*(a6 + l5*cos(q6)),  ; -a6*m6*(a6 + l4*cos(q5 + q6) - l1*cos(q4 - q3 - q2 + q5 + q6) + l5*cos(q6) - l2*cos(q4 - q3 + q5 + q6)), -a6*m6*(a6 + l4*cos(q5 + q6) + l5*cos(q6) - l2*cos(q4 - q3 + q5 + q6)), -a6*m6*(a6 + l4*cos(q5 + q6) + l5*cos(q6)), a6*m6*(a6 + l4*cos(q5 + q6) + l5*cos(q6)), a6*m6*(a6 + l5*cos(q6)), a6^2*m6,  ; ];
 end