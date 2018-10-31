%
%	MBsysTran - Release 8.1 (built: August 08, 2015)
%
%	Copyright 
%	Universite catholique de Louvain (UCL) 
%	Center for Research in Energy and Mechatronics (CEREM) 
%	2, Place du Levant
%	1348 Louvain-la-Neuve 
%	Belgium 
%
%	http://www.robotran.be 
%
%	==> Generation Date: Thu Oct 13 09:50:17 2016
%
%	==> Project name: Planar_Legs
%
%	==> Number of joints: 11
%
%	==> Function: F10 - Direct Dynamics of constrained MBS
%

function [udd,q,qd,qdd,Qq] = mbs_accelred_Planar_Legs(s,tsim)

% udd : independent accelerations
% q   : generalized coordinates (closed MBS : q = {u,v})
% qd  : generalized velocities (closed MBS : qd = {ud,vd})
% qdd : generalized accelerations (closed MBS: : qdd = {udd,vdd})
% Qq  : generalized forces (free and driven joints)

q = s.q;
qd = s.qd;
qdd = cell(s.Njoint,1);

dpt = s.dpt;
l = s.l;

m = s.m;
In = s.In;

g = s.g;

udd = cell(3,1);
Qq = cell(11,1);

% iter = 0;
% MAX_NR_ITER = 20;
% NR_ERR = 1e-20;

% Driven Joints 

[q,qd,qdd] = user_DrivenJoints(s,tsim);
 
NRh2 = 1.0; 
% while((NRh2 > NR_ERR) && (iter < MAX_NR_ITER)) 
%    iter = iter + 1; 
% if iter == MAX_NR_ITER 
%    error('Loop closure : no convergence'); 
% end 
for iter = 1:3
% Trigonometric functions

S3 = sin(q{3});
C3 = cos(q{3});
S6 = sin(q{6});
C6 = cos(q{6});
S7 = sin(q{7});
C7 = cos(q{7});
S4 = sin(q{4});
C4 = cos(q{4});
S5 = sin(q{5});
C5 = cos(q{5});
 
% Augmented Joint Position Vectors

 
% Constraints and Constraints Jacobian

ROlp1_54 = C3*C6-S3*S6;
ROlp1_64 = C3*S6+S3*C6;
ROlp1_84 = -C3*S6-S3*C6;
ROlp1_94 = C3*C6-S3*S6;
ROlp1_85 = -ROlp1_54*S7+ROlp1_84*C7;
ROlp1_95 = -ROlp1_64*S7+ROlp1_94*C7;
RLlp1_24 = -dpt(3,4)*S3;
RLlp1_34 = dpt(3,4)*C3;
POlp1_24 = q{1}+RLlp1_24;
POlp1_34 = q{2}+RLlp1_34;
RLlp1_25 = ROlp1_84*dpt(3,7);
RLlp1_35 = ROlp1_94*dpt(3,7);
POlp1_25 = POlp1_24+RLlp1_25;
POlp1_35 = POlp1_34+RLlp1_35;
JTlp1_25_3 = -RLlp1_34-RLlp1_35;
JTlp1_35_3 = RLlp1_24+RLlp1_25;
RLlp1_26 = ROlp1_85*dpt(3,8);
RLlp1_36 = ROlp1_95*dpt(3,8);
POlp1_26 = POlp1_25+RLlp1_26;
POlp1_36 = POlp1_35+RLlp1_36;
JTlp1_26_3 = JTlp1_25_3-RLlp1_36;
JTlp1_36_3 = JTlp1_35_3+RLlp1_26;
JTlp1_26_4 = -RLlp1_35-RLlp1_36;
JTlp1_36_4 = RLlp1_25+RLlp1_26;
h_2 = -q{8}+POlp1_26;
h_3 = -q{9}+POlp1_36;
ROlp3_54 = C3*C4-S3*S4;
ROlp3_64 = C3*S4+S3*C4;
ROlp3_84 = -C3*S4-S3*C4;
ROlp3_94 = C3*C4-S3*S4;
ROlp3_85 = -ROlp3_54*S5+ROlp3_84*C5;
ROlp3_95 = -ROlp3_64*S5+ROlp3_94*C5;
RLlp3_24 = -dpt(3,3)*S3;
RLlp3_34 = dpt(3,3)*C3;
POlp3_24 = q{1}+RLlp3_24;
POlp3_34 = q{2}+RLlp3_34;
RLlp3_25 = ROlp3_84*dpt(3,5);
RLlp3_35 = ROlp3_94*dpt(3,5);
POlp3_25 = POlp3_24+RLlp3_25;
POlp3_35 = POlp3_34+RLlp3_35;
JTlp3_25_3 = -RLlp3_34-RLlp3_35;
JTlp3_35_3 = RLlp3_24+RLlp3_25;
RLlp3_26 = ROlp3_85*dpt(3,6);
RLlp3_36 = ROlp3_95*dpt(3,6);
POlp3_26 = POlp3_25+RLlp3_26;
POlp3_36 = POlp3_35+RLlp3_36;
JTlp3_26_3 = JTlp3_25_3-RLlp3_36;
JTlp3_36_3 = JTlp3_35_3+RLlp3_26;
JTlp3_26_4 = -RLlp3_35-RLlp3_36;
JTlp3_36_4 = RLlp3_25+RLlp3_26;
h_5 = -q{10}+POlp3_26;
h_6 = -q{11}+POlp3_36;
 
% Loop Closure Thresold

NRh2_1 = h_2*h_2;
NRh2_2 = NRh2_1+h_3*h_3;
NRh2_3 = NRh2_2+h_5*h_5;
NRh2 = NRh2_3+h_6*h_6;
 
% LU Factorisation of the Jacobian (Jv)

Jvlu2_1 = JTlp1_36_4/JTlp1_26_4;
Jvlu2_2 = RLlp1_26+Jvlu2_1*RLlp1_36;
Jvlu4_3 = JTlp3_36_4/JTlp3_26_4;
Jvlu4_4 = RLlp3_26+Jvlu4_3*RLlp3_36;
% 

% Newton-Raphson: Loop Closure

Jv_hy2 = h_3-Jvlu2_1*h_2;
Jv_hy4 = h_6-Jvlu4_3*h_5;
Jv_hx4 = Jv_hy4/Jvlu4_4;
Jv_hx3 = (h_5+Jv_hx4*RLlp3_36)/JTlp3_26_4;
Jv_hx2 = Jv_hy2/Jvlu2_2;
Jv_hx1 = (h_2+Jv_hx2*RLlp1_36)/JTlp1_26_4;
q{6} = q{6}-Jv_hx1;
q{7} = q{7}-Jv_hx2;
q{4} = q{4}-Jv_hx3;
q{5} = q{5}-Jv_hx4;
 
% end 
end

 
% Bvu Matrix

Bvu2y3 = -JTlp1_36_3+JTlp1_26_3*Jvlu2_1;
Bvu4y3 = -JTlp3_36_3+JTlp3_26_3*Jvlu4_3;
Bvu4x1 = Jvlu4_3/Jvlu4_4;
Bvu4x2 = -(1.0)/Jvlu4_4;
Bvu4x3 = Bvu4y3/Jvlu4_4;
Bvu4x6 = -Jvlu4_3/Jvlu4_4;
Bvu4x7 = (1.0)/Jvlu4_4;
Bvu3x1 = (-(1.0)+Bvu4x1*RLlp3_36)/JTlp3_26_4;
Bvu3x2 = (Bvu4x2*RLlp3_36)/JTlp3_26_4;
Bvu3x3 = (-JTlp3_26_3+Bvu4x3*RLlp3_36)/JTlp3_26_4;
Bvu3x6 = ((1.0)+Bvu4x6*RLlp3_36)/JTlp3_26_4;
Bvu3x7 = (Bvu4x7*RLlp3_36)/JTlp3_26_4;
Bvu2x1 = Jvlu2_1/Jvlu2_2;
Bvu2x2 = -(1.0)/Jvlu2_2;
Bvu2x3 = Bvu2y3/Jvlu2_2;
Bvu2x4 = -Jvlu2_1/Jvlu2_2;
Bvu2x5 = (1.0)/Jvlu2_2;
Bvu1x1 = (-(1.0)+Bvu2x1*RLlp1_36)/JTlp1_26_4;
Bvu1x2 = (Bvu2x2*RLlp1_36)/JTlp1_26_4;
Bvu1x3 = (-JTlp1_26_3+Bvu2x3*RLlp1_36)/JTlp1_26_4;
Bvu1x4 = ((1.0)+Bvu2x4*RLlp1_36)/JTlp1_26_4;
Bvu1x5 = (Bvu2x5*RLlp1_36)/JTlp1_26_4;
 
% Dependent Velocities

qd{6} = qd{1}*Bvu1x1+qd{2}*Bvu1x2+qd{3}*Bvu1x3;
qd{7} = qd{1}*Bvu2x1+qd{2}*Bvu2x2+qd{3}*Bvu2x3;
qd{4} = qd{1}*Bvu3x1+qd{2}*Bvu3x2+qd{3}*Bvu3x3;
qd{5} = qd{1}*Bvu4x1+qd{2}*Bvu4x2+qd{3}*Bvu4x3;
 
% Constraints Quadratic Terms

ROjdqd1_54 = C3*C6-S3*S6;
ROjdqd1_64 = C3*S6+S3*C6;
ROjdqd1_84 = -C3*S6-S3*C6;
ROjdqd1_94 = C3*C6-S3*S6;
ROjdqd1_85 = -ROjdqd1_54*S7+ROjdqd1_84*C7;
ROjdqd1_95 = -ROjdqd1_64*S7+ROjdqd1_94*C7;
RLjdqd1_24 = -dpt(3,4)*S3;
RLjdqd1_34 = dpt(3,4)*C3;
OMjdqd1_14 = qd{3}+qd{6};
ORjdqd1_24 = -qd{3}*RLjdqd1_34;
ORjdqd1_34 = qd{3}*RLjdqd1_24;
Apqpjdqd1_24 = -qd{3}*ORjdqd1_34;
Apqpjdqd1_34 = qd{3}*ORjdqd1_24;
RLjdqd1_25 = ROjdqd1_84*dpt(3,7);
RLjdqd1_35 = ROjdqd1_94*dpt(3,7);
OMjdqd1_15 = OMjdqd1_14+qd{7};
ORjdqd1_25 = -OMjdqd1_14*RLjdqd1_35;
ORjdqd1_35 = OMjdqd1_14*RLjdqd1_25;
Apqpjdqd1_25 = Apqpjdqd1_24-OMjdqd1_14*ORjdqd1_35;
Apqpjdqd1_35 = Apqpjdqd1_34+OMjdqd1_14*ORjdqd1_25;
RLjdqd1_26 = ROjdqd1_85*dpt(3,8);
RLjdqd1_36 = ROjdqd1_95*dpt(3,8);
ORjdqd1_26 = -OMjdqd1_15*RLjdqd1_36;
ORjdqd1_36 = OMjdqd1_15*RLjdqd1_26;
Apqpjdqd1_26 = Apqpjdqd1_25-OMjdqd1_15*ORjdqd1_36;
Apqpjdqd1_36 = Apqpjdqd1_35+OMjdqd1_15*ORjdqd1_26;
ROjdqd3_54 = C3*C4-S3*S4;
ROjdqd3_64 = C3*S4+S3*C4;
ROjdqd3_84 = -C3*S4-S3*C4;
ROjdqd3_94 = C3*C4-S3*S4;
ROjdqd3_85 = -ROjdqd3_54*S5+ROjdqd3_84*C5;
ROjdqd3_95 = -ROjdqd3_64*S5+ROjdqd3_94*C5;
RLjdqd3_24 = -dpt(3,3)*S3;
RLjdqd3_34 = dpt(3,3)*C3;
OMjdqd3_14 = qd{3}+qd{4};
ORjdqd3_24 = -qd{3}*RLjdqd3_34;
ORjdqd3_34 = qd{3}*RLjdqd3_24;
Apqpjdqd3_24 = -qd{3}*ORjdqd3_34;
Apqpjdqd3_34 = qd{3}*ORjdqd3_24;
RLjdqd3_25 = ROjdqd3_84*dpt(3,5);
RLjdqd3_35 = ROjdqd3_94*dpt(3,5);
OMjdqd3_15 = OMjdqd3_14+qd{5};
ORjdqd3_25 = -OMjdqd3_14*RLjdqd3_35;
ORjdqd3_35 = OMjdqd3_14*RLjdqd3_25;
Apqpjdqd3_25 = Apqpjdqd3_24-OMjdqd3_14*ORjdqd3_35;
Apqpjdqd3_35 = Apqpjdqd3_34+OMjdqd3_14*ORjdqd3_25;
RLjdqd3_26 = ROjdqd3_85*dpt(3,6);
RLjdqd3_36 = ROjdqd3_95*dpt(3,6);
ORjdqd3_26 = -OMjdqd3_15*RLjdqd3_36;
ORjdqd3_36 = OMjdqd3_15*RLjdqd3_26;
Apqpjdqd3_26 = Apqpjdqd3_25-OMjdqd3_15*ORjdqd3_36;
Apqpjdqd3_36 = Apqpjdqd3_35+OMjdqd3_15*ORjdqd3_26;
 
% b Vector

b1y2 = -Apqpjdqd1_36+Apqpjdqd1_26*Jvlu2_1;
b1y4 = -Apqpjdqd3_36+Apqpjdqd3_26*Jvlu4_3;
b1x4 = b1y4/Jvlu4_4;
b1x3 = (-Apqpjdqd3_26+RLlp3_36*b1x4)/JTlp3_26_4;
b1x2 = b1y2/Jvlu2_2;
b1x1 = (-Apqpjdqd1_26+RLlp1_36*b1x2)/JTlp1_26_4;
 
% Forces and Torques computation 

s.q = q;   % Update of the mbs structure (positions) 
s.qd = qd; % Update of the mbs structure (velocities)

% Joint Forces 

Qq = user_JointForces(s,tsim);
 
% Forward Kinematics

BS93 = -qd{3}*qd{3};
AlF23 = -g(3)*S3;
AlF33 = -g(3)*C3;
OM14 = qd{3}+qd{4};
BS94 = -OM14*OM14;
AlF24 = AlF23*C4+S4*(AlF33+BS93*dpt(3,3));
AlF34 = -AlF23*S4+C4*(AlF33+BS93*dpt(3,3));
AlM24_1 = C3*C4-S3*S4;
AlM34_1 = -C3*S4-S3*C4;
AlM24_2 = C3*S4+S3*C4;
AlM34_2 = C3*C4-S3*S4;
AlM24_3 = -dpt(3,3)*C4;
AlM34_3 = dpt(3,3)*S4;
OM15 = OM14+qd{5};
BS95 = -OM15*OM15;
AlF25 = AlF24*C5+S5*(AlF34+BS94*dpt(3,5));
AlF35 = -AlF24*S5+C5*(AlF34+BS94*dpt(3,5));
AlM25_1 = AlM24_1*C5+AlM34_1*S5;
AlM35_1 = -AlM24_1*S5+AlM34_1*C5;
AlM25_2 = AlM24_2*C5+AlM34_2*S5;
AlM35_2 = -AlM24_2*S5+AlM34_2*C5;
AlM25_3 = AlM34_3*S5+C5*(AlM24_3-dpt(3,5));
AlM35_3 = AlM34_3*C5-S5*(AlM24_3-dpt(3,5));
AlM25_4 = -dpt(3,5)*C5;
AlM35_4 = dpt(3,5)*S5;
OM16 = qd{3}+qd{6};
BS96 = -OM16*OM16;
AlF26 = AlF23*C6+S6*(AlF33+BS93*dpt(3,4));
AlF36 = -AlF23*S6+C6*(AlF33+BS93*dpt(3,4));
AlM26_1 = C3*C6-S3*S6;
AlM36_1 = -C3*S6-S3*C6;
AlM26_2 = C3*S6+S3*C6;
AlM36_2 = C3*C6-S3*S6;
AlM26_3 = -dpt(3,4)*C6;
AlM36_3 = dpt(3,4)*S6;
OM17 = OM16+qd{7};
BS97 = -OM17*OM17;
AlF27 = AlF26*C7+S7*(AlF36+BS96*dpt(3,7));
AlF37 = -AlF26*S7+C7*(AlF36+BS96*dpt(3,7));
AlM27_1 = AlM26_1*C7+AlM36_1*S7;
AlM37_1 = -AlM26_1*S7+AlM36_1*C7;
AlM27_2 = AlM26_2*C7+AlM36_2*S7;
AlM37_2 = -AlM26_2*S7+AlM36_2*C7;
AlM27_3 = AlM36_3*S7+C7*(AlM26_3-dpt(3,7));
AlM37_3 = AlM36_3*C7-S7*(AlM26_3-dpt(3,7));
AlM27_6 = -dpt(3,7)*C7;
AlM37_6 = dpt(3,7)*S7;
 
% Backward Dynamics

FA27 = m(7)*AlF27;
FA37 = m(7)*(AlF37+BS97*l(3,7));
CF17 = -FA27*l(3,7);
FB27_1 = m(7)*AlM27_1;
FB37_1 = m(7)*AlM37_1;
CM17_1 = -FB27_1*l(3,7);
FB27_2 = m(7)*AlM27_2;
FB37_2 = m(7)*AlM37_2;
CM17_2 = -FB27_2*l(3,7);
FB27_3 = m(7)*(AlM27_3-l(3,7));
FB37_3 = m(7)*AlM37_3;
CM17_3 = In(1,7)-FB27_3*l(3,7);
FB27_6 = m(7)*(AlM27_6-l(3,7));
FB37_6 = m(7)*AlM37_6;
CM17_6 = In(1,7)-FB27_6*l(3,7);
FB27_7 = -m(7)*l(3,7);
CM17_7 = In(1,7)-FB27_7*l(3,7);
FA26 = m(6)*AlF26;
FA36 = m(6)*(AlF36+BS96*l(3,6));
FF26 = FA26+FA27*C7-FA37*S7;
FF36 = FA36+FA27*S7+FA37*C7;
CF16 = CF17-FA26*l(3,6)-dpt(3,7)*(FA27*C7-FA37*S7);
FB26_1 = m(6)*AlM26_1;
FB36_1 = m(6)*AlM36_1;
FM26_1 = FB26_1+FB27_1*C7-FB37_1*S7;
FM36_1 = FB36_1+FB27_1*S7+FB37_1*C7;
CM16_1 = CM17_1-FB26_1*l(3,6)-dpt(3,7)*(FB27_1*C7-FB37_1*S7);
FB26_2 = m(6)*AlM26_2;
FB36_2 = m(6)*AlM36_2;
FM26_2 = FB26_2+FB27_2*C7-FB37_2*S7;
FM36_2 = FB36_2+FB27_2*S7+FB37_2*C7;
CM16_2 = CM17_2-FB26_2*l(3,6)-dpt(3,7)*(FB27_2*C7-FB37_2*S7);
FB26_3 = m(6)*(AlM26_3-l(3,6));
FB36_3 = m(6)*AlM36_3;
FM26_3 = FB26_3+FB27_3*C7-FB37_3*S7;
FM36_3 = FB36_3+FB27_3*S7+FB37_3*C7;
CM16_3 = In(1,6)+CM17_3-FB26_3*l(3,6)-dpt(3,7)*(FB27_3*C7-FB37_3*S7);
FB26_6 = -m(6)*l(3,6);
CM16_6 = In(1,6)+CM17_6-FB26_6*l(3,6)-dpt(3,7)*(FB27_6*C7-FB37_6*S7);
FA25 = m(5)*AlF25;
FA35 = m(5)*(AlF35+BS95*l(3,5));
CF15 = -FA25*l(3,5);
FB25_1 = m(5)*AlM25_1;
FB35_1 = m(5)*AlM35_1;
CM15_1 = -FB25_1*l(3,5);
FB25_2 = m(5)*AlM25_2;
FB35_2 = m(5)*AlM35_2;
CM15_2 = -FB25_2*l(3,5);
FB25_3 = m(5)*(AlM25_3-l(3,5));
FB35_3 = m(5)*AlM35_3;
CM15_3 = In(1,5)-FB25_3*l(3,5);
FB25_4 = m(5)*(AlM25_4-l(3,5));
FB35_4 = m(5)*AlM35_4;
CM15_4 = In(1,5)-FB25_4*l(3,5);
FB25_5 = -m(5)*l(3,5);
CM15_5 = In(1,5)-FB25_5*l(3,5);
FA24 = m(4)*AlF24;
FA34 = m(4)*(AlF34+BS94*l(3,4));
FF24 = FA24+FA25*C5-FA35*S5;
FF34 = FA34+FA25*S5+FA35*C5;
CF14 = CF15-FA24*l(3,4)-dpt(3,5)*(FA25*C5-FA35*S5);
FB24_1 = m(4)*AlM24_1;
FB34_1 = m(4)*AlM34_1;
FM24_1 = FB24_1+FB25_1*C5-FB35_1*S5;
FM34_1 = FB34_1+FB25_1*S5+FB35_1*C5;
CM14_1 = CM15_1-FB24_1*l(3,4)-dpt(3,5)*(FB25_1*C5-FB35_1*S5);
FB24_2 = m(4)*AlM24_2;
FB34_2 = m(4)*AlM34_2;
FM24_2 = FB24_2+FB25_2*C5-FB35_2*S5;
FM34_2 = FB34_2+FB25_2*S5+FB35_2*C5;
CM14_2 = CM15_2-FB24_2*l(3,4)-dpt(3,5)*(FB25_2*C5-FB35_2*S5);
FB24_3 = m(4)*(AlM24_3-l(3,4));
FB34_3 = m(4)*AlM34_3;
FM24_3 = FB24_3+FB25_3*C5-FB35_3*S5;
FM34_3 = FB34_3+FB25_3*S5+FB35_3*C5;
CM14_3 = In(1,4)+CM15_3-FB24_3*l(3,4)-dpt(3,5)*(FB25_3*C5-FB35_3*S5);
FB24_4 = -m(4)*l(3,4);
CM14_4 = In(1,4)+CM15_4-FB24_4*l(3,4)-dpt(3,5)*(FB25_4*C5-FB35_4*S5);
FA23 = m(3)*AlF23;
FA33 = m(3)*(AlF33+BS93*l(3,3));
FF23 = FA23+FF24*C4+FF26*C6-FF34*S4-FF36*S6;
FF33 = FA33+FF24*S4+FF26*S6+FF34*C4+FF36*C6;
CF13 = CF14+CF16-FA23*l(3,3)-dpt(3,3)*(FF24*C4-FF34*S4)-dpt(3,4)*(FF26*C6-FF36*S6);
FB23_1 = m(3)*C3;
FB33_1 = -m(3)*S3;
FM23_1 = FB23_1+FM24_1*C4+FM26_1*C6-FM34_1*S4-FM36_1*S6;
FM33_1 = FB33_1+FM24_1*S4+FM26_1*S6+FM34_1*C4+FM36_1*C6;
CM13_1 = CM14_1+CM16_1-FB23_1*l(3,3)-dpt(3,3)*(FM24_1*C4-FM34_1*S4)-dpt(3,4)*(FM26_1*C6-FM36_1*S6);
FB23_2 = m(3)*S3;
FB33_2 = m(3)*C3;
FM23_2 = FB23_2+FM24_2*C4+FM26_2*C6-FM34_2*S4-FM36_2*S6;
FM33_2 = FB33_2+FM24_2*S4+FM26_2*S6+FM34_2*C4+FM36_2*C6;
CM13_2 = CM14_2+CM16_2-FB23_2*l(3,3)-dpt(3,3)*(FM24_2*C4-FM34_2*S4)-dpt(3,4)*(FM26_2*C6-FM36_2*S6);
FB23_3 = -m(3)*l(3,3);
CM13_3 = In(1,3)+CM14_3+CM16_3-FB23_3*l(3,3)-dpt(3,3)*(FM24_3*C4-FM34_3*S4)-dpt(3,4)*(FM26_3*C6-FM36_3*S6);
FF22 = FF23*C3-FF33*S3;
FF32 = FF23*S3+FF33*C3;
FM22_1 = FM23_1*C3-FM33_1*S3;
FM32_1 = FM23_1*S3+FM33_1*C3;
FM32_2 = FM23_2*S3+FM33_2*C3;
 
% Reduction (Coordinate Partitioning)

MMB1_1 = CM16_1+Bvu1x1*CM16_6+Bvu2x1*CM17_6;
MMB1_2 = CM16_2+Bvu1x2*CM16_6+Bvu2x2*CM17_6;
MMB1_3 = CM16_3+Bvu1x3*CM16_6+Bvu2x3*CM17_6;
MMB1_4 = Bvu1x4*CM16_6+Bvu2x4*CM17_6;
MMB1_5 = Bvu1x5*CM16_6+Bvu2x5*CM17_6;
MMB2_1 = CM17_1+Bvu1x1*CM17_6+Bvu2x1*CM17_7;
MMB2_2 = CM17_2+Bvu1x2*CM17_6+Bvu2x2*CM17_7;
MMB2_3 = CM17_3+Bvu1x3*CM17_6+Bvu2x3*CM17_7;
MMB2_4 = Bvu1x4*CM17_6+Bvu2x4*CM17_7;
MMB2_5 = Bvu1x5*CM17_6+Bvu2x5*CM17_7;
MMB3_1 = CM14_1+Bvu3x1*CM14_4+Bvu4x1*CM15_4;
MMB3_2 = CM14_2+Bvu3x2*CM14_4+Bvu4x2*CM15_4;
MMB3_3 = CM14_3+Bvu3x3*CM14_4+Bvu4x3*CM15_4;
MMB3_6 = Bvu3x6*CM14_4+Bvu4x6*CM15_4;
MMB3_7 = Bvu3x7*CM14_4+Bvu4x7*CM15_4;
MMB4_1 = CM15_1+Bvu3x1*CM15_4+Bvu4x1*CM15_5;
MMB4_2 = CM15_2+Bvu3x2*CM15_4+Bvu4x2*CM15_5;
MMB4_3 = CM15_3+Bvu3x3*CM15_4+Bvu4x3*CM15_5;
MMB4_6 = Bvu3x6*CM15_4+Bvu4x6*CM15_5;
MMB4_7 = Bvu3x7*CM15_4+Bvu4x7*CM15_5;
Mr1_1 = FM22_1+Bvu1x1*MMB1_1+Bvu2x1*MMB2_1+Bvu3x1*MMB3_1+Bvu4x1*MMB4_1+Bvu1x1*CM16_1+Bvu2x1*CM17_1+Bvu3x1*CM14_1+Bvu4x1*CM15_1;
Mr1_2 = FM32_1+Bvu1x1*MMB1_2+Bvu2x1*MMB2_2+Bvu3x1*MMB3_2+Bvu4x1*MMB4_2+Bvu1x2*CM16_1+Bvu2x2*CM17_1+Bvu3x2*CM14_1+Bvu4x2*CM15_1;
Mr1_3 = CM13_1+Bvu1x1*MMB1_3+Bvu2x1*MMB2_3+Bvu3x1*MMB3_3+Bvu4x1*MMB4_3+Bvu1x3*CM16_1+Bvu2x3*CM17_1+Bvu3x3*CM14_1+Bvu4x3*CM15_1;
Mr1_4 = Bvu1x1*MMB1_4+Bvu2x1*MMB2_4+Bvu1x4*CM16_1+Bvu2x4*CM17_1;
Mr1_5 = Bvu1x1*MMB1_5+Bvu2x1*MMB2_5+Bvu1x5*CM16_1+Bvu2x5*CM17_1;
Mr1_6 = Bvu3x1*MMB3_6+Bvu4x1*MMB4_6+Bvu3x6*CM14_1+Bvu4x6*CM15_1;
Mr1_7 = Bvu3x1*MMB3_7+Bvu4x1*MMB4_7+Bvu3x7*CM14_1+Bvu4x7*CM15_1;
Mr2_2 = FM32_2+Bvu1x2*MMB1_2+Bvu2x2*MMB2_2+Bvu3x2*MMB3_2+Bvu4x2*MMB4_2+Bvu1x2*CM16_2+Bvu2x2*CM17_2+Bvu3x2*CM14_2+Bvu4x2*CM15_2;
Mr2_3 = CM13_2+Bvu1x2*MMB1_3+Bvu2x2*MMB2_3+Bvu3x2*MMB3_3+Bvu4x2*MMB4_3+Bvu1x3*CM16_2+Bvu2x3*CM17_2+Bvu3x3*CM14_2+Bvu4x3*CM15_2;
Mr2_4 = Bvu1x2*MMB1_4+Bvu2x2*MMB2_4+Bvu1x4*CM16_2+Bvu2x4*CM17_2;
Mr2_5 = Bvu1x2*MMB1_5+Bvu2x2*MMB2_5+Bvu1x5*CM16_2+Bvu2x5*CM17_2;
Mr2_6 = Bvu3x2*MMB3_6+Bvu4x2*MMB4_6+Bvu3x6*CM14_2+Bvu4x6*CM15_2;
Mr2_7 = Bvu3x2*MMB3_7+Bvu4x2*MMB4_7+Bvu3x7*CM14_2+Bvu4x7*CM15_2;
Mr3_3 = CM13_3+Bvu1x3*MMB1_3+Bvu2x3*MMB2_3+Bvu3x3*MMB3_3+Bvu4x3*MMB4_3+Bvu1x3*CM16_3+Bvu2x3*CM17_3+Bvu3x3*CM14_3+Bvu4x3*CM15_3;
Mr3_4 = Bvu1x3*MMB1_4+Bvu2x3*MMB2_4+Bvu1x4*CM16_3+Bvu2x4*CM17_3;
Mr3_5 = Bvu1x3*MMB1_5+Bvu2x3*MMB2_5+Bvu1x5*CM16_3+Bvu2x5*CM17_3;
Mr3_6 = Bvu3x3*MMB3_6+Bvu4x3*MMB4_6+Bvu3x6*CM14_3+Bvu4x6*CM15_3;
Mr3_7 = Bvu3x3*MMB3_7+Bvu4x3*MMB4_7+Bvu3x7*CM14_3+Bvu4x7*CM15_3;
FMB1 = CF16-Qq{6}+CM16_6*b1x1+CM17_6*b1x2;
FMB2 = CF17-Qq{7}+CM17_6*b1x1+CM17_7*b1x2;
FMB3 = CF14-Qq{4}+CM14_4*b1x3+CM15_4*b1x4;
FMB4 = CF15-Qq{5}+CM15_4*b1x3+CM15_5*b1x4;
Fr1 = FF22-Qq{1}+Bvu4x1*FMB4+CM16_1*b1x1+CM14_1*b1x3+CM17_1*b1x2+Bvu1x1*FMB1+CM15_1*b1x4+Bvu2x1*FMB2+Bvu3x1*FMB3;
Fr2 = FF32-Qq{2}+Bvu4x2*FMB4+CM16_2*b1x1+CM14_2*b1x3+CM17_2*b1x2+Bvu1x2*FMB1+CM15_2*b1x4+Bvu2x2*FMB2+Bvu3x2*FMB3;
Fr3 = CF13-Qq{3}+Bvu4x3*FMB4+CM16_3*b1x1+CM14_3*b1x3+CM17_3*b1x2+Bvu1x3*FMB1+CM15_3*b1x4+Bvu2x3*FMB2+Bvu3x3*FMB3;
Fr4 = -Qq{8}+Bvu1x4*FMB1+Bvu2x4*FMB2;
Fr5 = -Qq{9}+Bvu1x5*FMB1+Bvu2x5*FMB2;
Fr6 = -Qq{10}+Bvu3x6*FMB3+Bvu4x6*FMB4;
Fr7 = -Qq{11}+Bvu3x7*FMB3+Bvu4x7*FMB4;
 
% Linear System Resolution

Mrlu2_1 = Mr1_2/Mr1_1;
Mrlu2_2 = Mr2_2-Mr1_2*Mrlu2_1;
Mrlu2_3 = Mr2_3-Mr1_3*Mrlu2_1;
Mrlu3_1 = Mr1_3/Mr1_1;
Mrlu1_3_2 = Mr2_3-Mr1_2*Mrlu3_1;
Mrlu1_3_3 = Mr3_3-Mr1_3*Mrlu3_1;
Mrlu3_2 = Mrlu1_3_2/Mrlu2_2;
Mrlu3_3 = Mrlu1_3_3-Mrlu2_3*Mrlu3_2;
qppuy2 = -Fr2+Fr1*Mrlu2_1;
qppuy3 = -Fr3+Fr1*Mrlu3_1-Mrlu3_2*qppuy2;
qppux3 = qppuy3/Mrlu3_3;
qppux2 = (qppuy2-Mrlu2_3*qppux3)/Mrlu2_2;
qppux1 = (-Fr1-Mr1_3*qppux3-Mr1_2*qppux2)/Mr1_1;
 
% Symbolic Output number 1

qdd{1} = qppux1;
qdd{2} = qppux2;
qdd{3} = qppux3;
 
% Symbolic Output number 2

udd{1} = qppux1;
udd{2} = qppux2;
udd{3} = qppux3;
 
% Symbolic Output number 3

qdd{6} = b1x1+Bvu1x1*qdd{1}+Bvu1x2*qdd{2}+Bvu1x3*qdd{3};
qdd{7} = b1x2+Bvu2x1*qdd{1}+Bvu2x2*qdd{2}+Bvu2x3*qdd{3};
qdd{4} = b1x3+Bvu3x1*qdd{1}+Bvu3x2*qdd{2}+Bvu3x3*qdd{3};
qdd{5} = b1x4+Bvu4x1*qdd{1}+Bvu4x2*qdd{2}+Bvu4x3*qdd{3};
 
% Symbolic Output number 4

Qc1 = Fr4+Mr1_4*qdd{1}+Mr2_4*qdd{2}+Mr3_4*qdd{3};
Qc2 = Fr5+Mr1_5*qdd{1}+Mr2_5*qdd{2}+Mr3_5*qdd{3};
Qc3 = Fr6+Mr1_6*qdd{1}+Mr2_6*qdd{2}+Mr3_6*qdd{3};
Qc4 = Fr7+Mr1_7*qdd{1}+Mr2_7*qdd{2}+Mr3_7*qdd{3};

Qq{8} = Qc1;   % Horizontal force for right leg
Qq{9} = Qc2;   % Vertical force for right leg
Qq{10} = Qc3;  % Horizontal force for left leg
Qq{11} = Qc4;  % Vertical force for left leg
 



