%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Thu Dec 22 16:03:05 2016
%
%	==> Project name : Planar_Legs
%	==> using XML input file 
%
%	==> Number of joints : 9
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 290
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [sens] = gensensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,9);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C3 = cos(q(3));
  S3 = sin(q(3));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));
  C5 = cos(q(5));
  S5 = sin(q(5));
  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C7 = cos(q(7));
  S7 = sin(q(7));
  C8 = cos(q(8));
  S8 = sin(q(8));
  C9 = cos(q(9));
  S9 = sin(q(9));

% = = Block_0_0_0_4_0_2 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;

% = = Block_0_0_0_5_0_2 = = 
 
% Trigonometric Variables  

  S5p3p4 = C5*S3p4+S5*C3p4;
  C5p3p4 = C5*C3p4-S5*S3p4;

% = = Block_0_0_0_6_0_2 = = 
 
% Trigonometric Variables  

  S6p5p3p4 = C6*S5p3p4+S6*C5p3p4;
  C6p5p3p4 = C6*C5p3p4-S6*S5p3p4;

% = = Block_0_0_0_7_0_3 = = 
 
% Trigonometric Variables  

  S3p7 = C3*S7+S3*C7;
  C3p7 = C3*C7-S3*S7;

% = = Block_0_0_0_8_0_3 = = 
 
% Trigonometric Variables  

  S8p3p7 = C8*S3p7+S8*C3p7;
  C8p3p7 = C8*C3p7-S8*S3p7;

% = = Block_0_0_0_9_0_3 = = 
 
% Trigonometric Variables  

  S9p8p3p7 = C9*S8p3p7+S9*C8p3p7;
  C9p8p3p7 = C9*C8p3p7-S9*S8p3p7;

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = q(1);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(2) = qd(1);
    sens.A(2) = qdd(1);
 
% 
case 2, 


% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = q(1);
    sens.P(3) = q(2);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(2) = qd(1);
    sens.V(3) = qd(2);
    sens.A(2) = qdd(1);
    sens.A(3) = qdd(2);
 
% 
case 3, 


% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = q(1);
    sens.P(3) = q(2);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C3;
    sens.R(2,3) = S3;
    sens.R(3,2) = -S3;
    sens.R(3,3) = C3;
    sens.V(2) = qd(1);
    sens.V(3) = qd(2);
    sens.OM(1) = qd(3);
    sens.A(2) = qdd(1);
    sens.A(3) = qdd(2);
    sens.OMP(1) = qdd(3);
 
% 
case 4, 


% = = Block_1_0_0_4_0_2 = = 
 
% Sensor Kinematics 


    RLcp3_24 = -s.dpt(3,1)*S3;
    RLcp3_34 = s.dpt(3,1)*C3;
    POcp3_24 = RLcp3_24+q(1);
    POcp3_34 = RLcp3_34+q(2);
    OMcp3_14 = qd(3)+qd(4);
    ORcp3_24 = -RLcp3_34*qd(3);
    ORcp3_34 = RLcp3_24*qd(3);
    VIcp3_24 = ORcp3_24+qd(1);
    VIcp3_34 = ORcp3_34+qd(2);
    OPcp3_14 = qdd(3)+qdd(4);
    ACcp3_24 = qdd(1)-ORcp3_34*qd(3)-RLcp3_34*qdd(3);
    ACcp3_34 = qdd(2)+ORcp3_24*qd(3)+RLcp3_24*qdd(3);

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,1);
    sens.P(2) = POcp3_24;
    sens.P(3) = POcp3_34;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C3p4;
    sens.R(2,3) = S3p4;
    sens.R(3,2) = -S3p4;
    sens.R(3,3) = C3p4;
    sens.V(2) = VIcp3_24;
    sens.V(3) = VIcp3_34;
    sens.OM(1) = OMcp3_14;
    sens.A(2) = ACcp3_24;
    sens.A(3) = ACcp3_34;
    sens.OMP(1) = OPcp3_14;
 
% 
case 5, 


% = = Block_1_0_0_5_0_2 = = 
 
% Sensor Kinematics 


    RLcp4_24 = -s.dpt(3,1)*S3;
    RLcp4_34 = s.dpt(3,1)*C3;
    OMcp4_14 = qd(3)+qd(4);
    ORcp4_24 = -RLcp4_34*qd(3);
    ORcp4_34 = RLcp4_24*qd(3);
    OPcp4_14 = qdd(3)+qdd(4);
    RLcp4_25 = -s.dpt(3,3)*S3p4;
    RLcp4_35 = s.dpt(3,3)*C3p4;
    POcp4_25 = RLcp4_24+RLcp4_25+q(1);
    POcp4_35 = RLcp4_34+RLcp4_35+q(2);
    OMcp4_15 = OMcp4_14+qd(5);
    ORcp4_25 = -OMcp4_14*RLcp4_35;
    ORcp4_35 = OMcp4_14*RLcp4_25;
    VIcp4_25 = ORcp4_24+ORcp4_25+qd(1);
    VIcp4_35 = ORcp4_34+ORcp4_35+qd(2);
    OPcp4_15 = OPcp4_14+qdd(5);
    ACcp4_25 = qdd(1)-OMcp4_14*ORcp4_35-OPcp4_14*RLcp4_35-ORcp4_34*qd(3)-RLcp4_34*qdd(3);
    ACcp4_35 = qdd(2)+OMcp4_14*ORcp4_25+OPcp4_14*RLcp4_25+ORcp4_24*qd(3)+RLcp4_24*qdd(3);

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,1);
    sens.P(2) = POcp4_25;
    sens.P(3) = POcp4_35;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C5p3p4;
    sens.R(2,3) = S5p3p4;
    sens.R(3,2) = -S5p3p4;
    sens.R(3,3) = C5p3p4;
    sens.V(2) = VIcp4_25;
    sens.V(3) = VIcp4_35;
    sens.OM(1) = OMcp4_15;
    sens.A(2) = ACcp4_25;
    sens.A(3) = ACcp4_35;
    sens.OMP(1) = OPcp4_15;
 
% 
case 6, 


% = = Block_1_0_0_6_0_2 = = 
 
% Sensor Kinematics 


    RLcp5_24 = -s.dpt(3,1)*S3;
    RLcp5_34 = s.dpt(3,1)*C3;
    OMcp5_14 = qd(3)+qd(4);
    ORcp5_24 = -RLcp5_34*qd(3);
    ORcp5_34 = RLcp5_24*qd(3);
    OPcp5_14 = qdd(3)+qdd(4);
    RLcp5_25 = -s.dpt(3,3)*S3p4;
    RLcp5_35 = s.dpt(3,3)*C3p4;
    OMcp5_15 = OMcp5_14+qd(5);
    ORcp5_25 = -OMcp5_14*RLcp5_35;
    ORcp5_35 = OMcp5_14*RLcp5_25;
    OPcp5_15 = OPcp5_14+qdd(5);
    RLcp5_26 = -s.dpt(3,4)*S5p3p4;
    RLcp5_36 = s.dpt(3,4)*C5p3p4;
    POcp5_26 = RLcp5_24+RLcp5_25+RLcp5_26+q(1);
    POcp5_36 = RLcp5_34+RLcp5_35+RLcp5_36+q(2);
    OMcp5_16 = OMcp5_15+qd(6);
    ORcp5_26 = -OMcp5_15*RLcp5_36;
    ORcp5_36 = OMcp5_15*RLcp5_26;
    VIcp5_26 = ORcp5_24+ORcp5_25+ORcp5_26+qd(1);
    VIcp5_36 = ORcp5_34+ORcp5_35+ORcp5_36+qd(2);
    OPcp5_16 = OPcp5_15+qdd(6);
    ACcp5_26 = qdd(1)-OMcp5_14*ORcp5_35-OMcp5_15*ORcp5_36-OPcp5_14*RLcp5_35-OPcp5_15*RLcp5_36-ORcp5_34*qd(3)-RLcp5_34*qdd(3);
    ACcp5_36 = qdd(2)+OMcp5_14*ORcp5_25+OMcp5_15*ORcp5_26+OPcp5_14*RLcp5_25+OPcp5_15*RLcp5_26+ORcp5_24*qd(3)+RLcp5_24*qdd(3);

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,1);
    sens.P(2) = POcp5_26;
    sens.P(3) = POcp5_36;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C6p5p3p4;
    sens.R(2,3) = S6p5p3p4;
    sens.R(3,2) = -S6p5p3p4;
    sens.R(3,3) = C6p5p3p4;
    sens.V(2) = VIcp5_26;
    sens.V(3) = VIcp5_36;
    sens.OM(1) = OMcp5_16;
    sens.A(2) = ACcp5_26;
    sens.A(3) = ACcp5_36;
    sens.OMP(1) = OPcp5_16;
 
% 
case 7, 


% = = Block_1_0_0_7_0_3 = = 
 
% Sensor Kinematics 


    RLcp6_27 = -s.dpt(3,2)*S3;
    RLcp6_37 = s.dpt(3,2)*C3;
    POcp6_27 = RLcp6_27+q(1);
    POcp6_37 = RLcp6_37+q(2);
    OMcp6_17 = qd(3)+qd(7);
    ORcp6_27 = -RLcp6_37*qd(3);
    ORcp6_37 = RLcp6_27*qd(3);
    VIcp6_27 = ORcp6_27+qd(1);
    VIcp6_37 = ORcp6_37+qd(2);
    OPcp6_17 = qdd(3)+qdd(7);
    ACcp6_27 = qdd(1)-ORcp6_37*qd(3)-RLcp6_37*qdd(3);
    ACcp6_37 = qdd(2)+ORcp6_27*qd(3)+RLcp6_27*qdd(3);

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,2);
    sens.P(2) = POcp6_27;
    sens.P(3) = POcp6_37;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C3p7;
    sens.R(2,3) = S3p7;
    sens.R(3,2) = -S3p7;
    sens.R(3,3) = C3p7;
    sens.V(2) = VIcp6_27;
    sens.V(3) = VIcp6_37;
    sens.OM(1) = OMcp6_17;
    sens.A(2) = ACcp6_27;
    sens.A(3) = ACcp6_37;
    sens.OMP(1) = OPcp6_17;
 
% 
case 8, 


% = = Block_1_0_0_8_0_3 = = 
 
% Sensor Kinematics 


    RLcp7_27 = -s.dpt(3,2)*S3;
    RLcp7_37 = s.dpt(3,2)*C3;
    OMcp7_17 = qd(3)+qd(7);
    ORcp7_27 = -RLcp7_37*qd(3);
    ORcp7_37 = RLcp7_27*qd(3);
    OPcp7_17 = qdd(3)+qdd(7);
    RLcp7_28 = -s.dpt(3,8)*S3p7;
    RLcp7_38 = s.dpt(3,8)*C3p7;
    POcp7_28 = RLcp7_27+RLcp7_28+q(1);
    POcp7_38 = RLcp7_37+RLcp7_38+q(2);
    OMcp7_18 = OMcp7_17+qd(8);
    ORcp7_28 = -OMcp7_17*RLcp7_38;
    ORcp7_38 = OMcp7_17*RLcp7_28;
    VIcp7_28 = ORcp7_27+ORcp7_28+qd(1);
    VIcp7_38 = ORcp7_37+ORcp7_38+qd(2);
    OPcp7_18 = OPcp7_17+qdd(8);
    ACcp7_28 = qdd(1)-OMcp7_17*ORcp7_38-OPcp7_17*RLcp7_38-ORcp7_37*qd(3)-RLcp7_37*qdd(3);
    ACcp7_38 = qdd(2)+OMcp7_17*ORcp7_28+OPcp7_17*RLcp7_28+ORcp7_27*qd(3)+RLcp7_27*qdd(3);

% = = Block_1_0_0_8_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,2);
    sens.P(2) = POcp7_28;
    sens.P(3) = POcp7_38;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C8p3p7;
    sens.R(2,3) = S8p3p7;
    sens.R(3,2) = -S8p3p7;
    sens.R(3,3) = C8p3p7;
    sens.V(2) = VIcp7_28;
    sens.V(3) = VIcp7_38;
    sens.OM(1) = OMcp7_18;
    sens.A(2) = ACcp7_28;
    sens.A(3) = ACcp7_38;
    sens.OMP(1) = OPcp7_18;
 
% 
case 9, 


% = = Block_1_0_0_9_0_3 = = 
 
% Sensor Kinematics 


    RLcp8_27 = -s.dpt(3,2)*S3;
    RLcp8_37 = s.dpt(3,2)*C3;
    OMcp8_17 = qd(3)+qd(7);
    ORcp8_27 = -RLcp8_37*qd(3);
    ORcp8_37 = RLcp8_27*qd(3);
    OPcp8_17 = qdd(3)+qdd(7);
    RLcp8_28 = -s.dpt(3,8)*S3p7;
    RLcp8_38 = s.dpt(3,8)*C3p7;
    OMcp8_18 = OMcp8_17+qd(8);
    ORcp8_28 = -OMcp8_17*RLcp8_38;
    ORcp8_38 = OMcp8_17*RLcp8_28;
    OPcp8_18 = OPcp8_17+qdd(8);
    RLcp8_29 = -s.dpt(3,9)*S8p3p7;
    RLcp8_39 = s.dpt(3,9)*C8p3p7;
    POcp8_29 = RLcp8_27+RLcp8_28+RLcp8_29+q(1);
    POcp8_39 = RLcp8_37+RLcp8_38+RLcp8_39+q(2);
    OMcp8_19 = OMcp8_18+qd(9);
    ORcp8_29 = -OMcp8_18*RLcp8_39;
    ORcp8_39 = OMcp8_18*RLcp8_29;
    VIcp8_29 = ORcp8_27+ORcp8_28+ORcp8_29+qd(1);
    VIcp8_39 = ORcp8_37+ORcp8_38+ORcp8_39+qd(2);
    OPcp8_19 = OPcp8_18+qdd(9);
    ACcp8_29 = qdd(1)-OMcp8_17*ORcp8_38-OMcp8_18*ORcp8_39-OPcp8_17*RLcp8_38-OPcp8_18*RLcp8_39-ORcp8_37*qd(3)-RLcp8_37*qdd(3);
    ACcp8_39 = qdd(2)+OMcp8_17*ORcp8_28+OMcp8_18*ORcp8_29+OPcp8_17*RLcp8_28+OPcp8_18*RLcp8_29+ORcp8_27*qd(3)+RLcp8_27*qdd(3);

% = = Block_1_0_0_9_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,2);
    sens.P(2) = POcp8_29;
    sens.P(3) = POcp8_39;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C9p8p3p7;
    sens.R(2,3) = S9p8p3p7;
    sens.R(3,2) = -S9p8p3p7;
    sens.R(3,3) = C9p8p3p7;
    sens.V(2) = VIcp8_29;
    sens.V(3) = VIcp8_39;
    sens.OM(1) = OMcp8_19;
    sens.A(2) = ACcp8_29;
    sens.A(3) = ACcp8_39;
    sens.OMP(1) = OPcp8_19;

end


% ====== END Task 1 ====== 

  

