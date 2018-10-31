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
%	==> Flops complexity : 620
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.020 seconds
%
%-------------------------------------------------------------
%
function [sens] = sensor(s,tsim,usrfun,isens)

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

% = = Block_0_0_0_1_0_2 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;
  S5p3p4 = C5*S3p4+S5*C3p4;
  C5p3p4 = C5*C3p4-S5*S3p4;
  S6p5p3p4 = C6*S5p3p4+S6*C5p3p4;
  C6p5p3p4 = C6*C5p3p4-S6*S5p3p4;

% = = Block_0_0_0_3_0_3 = = 
 
% Trigonometric Variables  

  S3p7 = C3*S7+S3*C7;
  C3p7 = C3*C7-S3*S7;
  S8p3p7 = C8*S3p7+S8*C3p7;
  C8p3p7 = C8*C3p7-S8*S3p7;
  S9p8p3p7 = C9*S8p3p7+S9*C8p3p7;
  C9p8p3p7 = C9*C8p3p7-S9*S8p3p7;

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_0_2 = = 
 
% Sensor Kinematics 


    RLcp0_24 = -s.dpt(3,1)*S3;
    RLcp0_34 = s.dpt(3,1)*C3;
    OMcp0_14 = qd(3)+qd(4);
    ORcp0_24 = -RLcp0_34*qd(3);
    ORcp0_34 = RLcp0_24*qd(3);
    OPcp0_14 = qdd(3)+qdd(4);
    RLcp0_25 = -s.dpt(3,3)*S3p4;
    RLcp0_35 = s.dpt(3,3)*C3p4;
    OMcp0_15 = OMcp0_14+qd(5);
    ORcp0_25 = -OMcp0_14*RLcp0_35;
    ORcp0_35 = OMcp0_14*RLcp0_25;
    OPcp0_15 = OPcp0_14+qdd(5);
    RLcp0_26 = -s.dpt(3,4)*S5p3p4;
    RLcp0_36 = s.dpt(3,4)*C5p3p4;
    OMcp0_16 = OMcp0_15+qd(6);
    ORcp0_26 = -OMcp0_15*RLcp0_36;
    ORcp0_36 = OMcp0_15*RLcp0_26;
    OPcp0_16 = OPcp0_15+qdd(6);
    RLcp0_210 = s.dpt(2,6)*C6p5p3p4-s.dpt(3,6)*S6p5p3p4;
    RLcp0_310 = s.dpt(2,6)*S6p5p3p4+s.dpt(3,6)*C6p5p3p4;
    POcp0_210 = RLcp0_210+RLcp0_24+RLcp0_25+RLcp0_26+q(1);
    POcp0_310 = RLcp0_310+RLcp0_34+RLcp0_35+RLcp0_36+q(2);
    JTcp0_210_3 = -(RLcp0_310+RLcp0_34+RLcp0_35+RLcp0_36);
    JTcp0_310_3 = RLcp0_210+RLcp0_24+RLcp0_25+RLcp0_26;
    JTcp0_210_4 = -(RLcp0_310+RLcp0_35+RLcp0_36);
    JTcp0_310_4 = RLcp0_210+RLcp0_25+RLcp0_26;
    JTcp0_210_5 = -(RLcp0_310+RLcp0_36);
    JTcp0_310_5 = RLcp0_210+RLcp0_26;
    ORcp0_210 = -OMcp0_16*RLcp0_310;
    ORcp0_310 = OMcp0_16*RLcp0_210;
    VIcp0_210 = ORcp0_210+ORcp0_24+ORcp0_25+ORcp0_26+qd(1);
    VIcp0_310 = ORcp0_310+ORcp0_34+ORcp0_35+ORcp0_36+qd(2);
    ACcp0_210 = qdd(1)-OMcp0_14*ORcp0_35-OMcp0_15*ORcp0_36-OMcp0_16*ORcp0_310-OPcp0_14*RLcp0_35-OPcp0_15*RLcp0_36-OPcp0_16*RLcp0_310-ORcp0_34*qd(3)...
 -RLcp0_34*qdd(3);
    ACcp0_310 = qdd(2)+OMcp0_14*ORcp0_25+OMcp0_15*ORcp0_26+OMcp0_16*ORcp0_210+OPcp0_14*RLcp0_25+OPcp0_15*RLcp0_26+OPcp0_16*RLcp0_210+ORcp0_24*qd(3)...
 +RLcp0_24*qdd(3);

% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,1);
    sens.P(2) = POcp0_210;
    sens.P(3) = POcp0_310;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C6p5p3p4;
    sens.R(2,3) = S6p5p3p4;
    sens.R(3,2) = -S6p5p3p4;
    sens.R(3,3) = C6p5p3p4;
    sens.V(2) = VIcp0_210;
    sens.V(3) = VIcp0_310;
    sens.OM(1) = OMcp0_16;
    sens.J(2,1) = (1.0);
    sens.J(2,3) = JTcp0_210_3;
    sens.J(2,4) = JTcp0_210_4;
    sens.J(2,5) = JTcp0_210_5;
    sens.J(2,6) = -RLcp0_310;
    sens.J(3,2) = (1.0);
    sens.J(3,3) = JTcp0_310_3;
    sens.J(3,4) = JTcp0_310_4;
    sens.J(3,5) = JTcp0_310_5;
    sens.J(3,6) = RLcp0_210;
    sens.J(4,3) = (1.0);
    sens.J(4,4) = (1.0);
    sens.J(4,5) = (1.0);
    sens.J(4,6) = (1.0);
    sens.A(2) = ACcp0_210;
    sens.A(3) = ACcp0_310;
    sens.OMP(1) = OPcp0_16;
 
% 
case 2, 


% = = Block_1_0_0_2_0_2 = = 
 
% Sensor Kinematics 


    RLcp1_24 = -s.dpt(3,1)*S3;
    RLcp1_34 = s.dpt(3,1)*C3;
    OMcp1_14 = qd(3)+qd(4);
    ORcp1_24 = -RLcp1_34*qd(3);
    ORcp1_34 = RLcp1_24*qd(3);
    OPcp1_14 = qdd(3)+qdd(4);
    RLcp1_25 = -s.dpt(3,3)*S3p4;
    RLcp1_35 = s.dpt(3,3)*C3p4;
    OMcp1_15 = OMcp1_14+qd(5);
    ORcp1_25 = -OMcp1_14*RLcp1_35;
    ORcp1_35 = OMcp1_14*RLcp1_25;
    OPcp1_15 = OPcp1_14+qdd(5);
    RLcp1_26 = -s.dpt(3,4)*S5p3p4;
    RLcp1_36 = s.dpt(3,4)*C5p3p4;
    OMcp1_16 = OMcp1_15+qd(6);
    ORcp1_26 = -OMcp1_15*RLcp1_36;
    ORcp1_36 = OMcp1_15*RLcp1_26;
    OPcp1_16 = OPcp1_15+qdd(6);
    RLcp1_211 = s.dpt(2,7)*C6p5p3p4-s.dpt(3,7)*S6p5p3p4;
    RLcp1_311 = s.dpt(2,7)*S6p5p3p4+s.dpt(3,7)*C6p5p3p4;
    POcp1_211 = RLcp1_211+RLcp1_24+RLcp1_25+RLcp1_26+q(1);
    POcp1_311 = RLcp1_311+RLcp1_34+RLcp1_35+RLcp1_36+q(2);
    JTcp1_211_3 = -(RLcp1_311+RLcp1_34+RLcp1_35+RLcp1_36);
    JTcp1_311_3 = RLcp1_211+RLcp1_24+RLcp1_25+RLcp1_26;
    JTcp1_211_4 = -(RLcp1_311+RLcp1_35+RLcp1_36);
    JTcp1_311_4 = RLcp1_211+RLcp1_25+RLcp1_26;
    JTcp1_211_5 = -(RLcp1_311+RLcp1_36);
    JTcp1_311_5 = RLcp1_211+RLcp1_26;
    ORcp1_211 = -OMcp1_16*RLcp1_311;
    ORcp1_311 = OMcp1_16*RLcp1_211;
    VIcp1_211 = ORcp1_211+ORcp1_24+ORcp1_25+ORcp1_26+qd(1);
    VIcp1_311 = ORcp1_311+ORcp1_34+ORcp1_35+ORcp1_36+qd(2);
    ACcp1_211 = qdd(1)-OMcp1_14*ORcp1_35-OMcp1_15*ORcp1_36-OMcp1_16*ORcp1_311-OPcp1_14*RLcp1_35-OPcp1_15*RLcp1_36-OPcp1_16*RLcp1_311-ORcp1_34*qd(3)...
 -RLcp1_34*qdd(3);
    ACcp1_311 = qdd(2)+OMcp1_14*ORcp1_25+OMcp1_15*ORcp1_26+OMcp1_16*ORcp1_211+OPcp1_14*RLcp1_25+OPcp1_15*RLcp1_26+OPcp1_16*RLcp1_211+ORcp1_24*qd(3)...
 +RLcp1_24*qdd(3);

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,1);
    sens.P(2) = POcp1_211;
    sens.P(3) = POcp1_311;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C6p5p3p4;
    sens.R(2,3) = S6p5p3p4;
    sens.R(3,2) = -S6p5p3p4;
    sens.R(3,3) = C6p5p3p4;
    sens.V(2) = VIcp1_211;
    sens.V(3) = VIcp1_311;
    sens.OM(1) = OMcp1_16;
    sens.J(2,1) = (1.0);
    sens.J(2,3) = JTcp1_211_3;
    sens.J(2,4) = JTcp1_211_4;
    sens.J(2,5) = JTcp1_211_5;
    sens.J(2,6) = -RLcp1_311;
    sens.J(3,2) = (1.0);
    sens.J(3,3) = JTcp1_311_3;
    sens.J(3,4) = JTcp1_311_4;
    sens.J(3,5) = JTcp1_311_5;
    sens.J(3,6) = RLcp1_211;
    sens.J(4,3) = (1.0);
    sens.J(4,4) = (1.0);
    sens.J(4,5) = (1.0);
    sens.J(4,6) = (1.0);
    sens.A(2) = ACcp1_211;
    sens.A(3) = ACcp1_311;
    sens.OMP(1) = OPcp1_16;
 
% 
case 3, 


% = = Block_1_0_0_3_0_3 = = 
 
% Sensor Kinematics 


    RLcp2_27 = -s.dpt(3,2)*S3;
    RLcp2_37 = s.dpt(3,2)*C3;
    OMcp2_17 = qd(3)+qd(7);
    ORcp2_27 = -RLcp2_37*qd(3);
    ORcp2_37 = RLcp2_27*qd(3);
    OPcp2_17 = qdd(3)+qdd(7);
    RLcp2_28 = -s.dpt(3,8)*S3p7;
    RLcp2_38 = s.dpt(3,8)*C3p7;
    OMcp2_18 = OMcp2_17+qd(8);
    ORcp2_28 = -OMcp2_17*RLcp2_38;
    ORcp2_38 = OMcp2_17*RLcp2_28;
    OPcp2_18 = OPcp2_17+qdd(8);
    RLcp2_29 = -s.dpt(3,9)*S8p3p7;
    RLcp2_39 = s.dpt(3,9)*C8p3p7;
    OMcp2_19 = OMcp2_18+qd(9);
    ORcp2_29 = -OMcp2_18*RLcp2_39;
    ORcp2_39 = OMcp2_18*RLcp2_29;
    OPcp2_19 = OPcp2_18+qdd(9);
    RLcp2_212 = s.dpt(2,10)*C9p8p3p7-s.dpt(3,10)*S9p8p3p7;
    RLcp2_312 = s.dpt(2,10)*S9p8p3p7+s.dpt(3,10)*C9p8p3p7;
    POcp2_212 = RLcp2_212+RLcp2_27+RLcp2_28+RLcp2_29+q(1);
    POcp2_312 = RLcp2_312+RLcp2_37+RLcp2_38+RLcp2_39+q(2);
    JTcp2_212_3 = -(RLcp2_312+RLcp2_37+RLcp2_38+RLcp2_39);
    JTcp2_312_3 = RLcp2_212+RLcp2_27+RLcp2_28+RLcp2_29;
    JTcp2_212_4 = -(RLcp2_312+RLcp2_38+RLcp2_39);
    JTcp2_312_4 = RLcp2_212+RLcp2_28+RLcp2_29;
    JTcp2_212_5 = -(RLcp2_312+RLcp2_39);
    JTcp2_312_5 = RLcp2_212+RLcp2_29;
    ORcp2_212 = -OMcp2_19*RLcp2_312;
    ORcp2_312 = OMcp2_19*RLcp2_212;
    VIcp2_212 = ORcp2_212+ORcp2_27+ORcp2_28+ORcp2_29+qd(1);
    VIcp2_312 = ORcp2_312+ORcp2_37+ORcp2_38+ORcp2_39+qd(2);
    ACcp2_212 = qdd(1)-OMcp2_17*ORcp2_38-OMcp2_18*ORcp2_39-OMcp2_19*ORcp2_312-OPcp2_17*RLcp2_38-OPcp2_18*RLcp2_39-OPcp2_19*RLcp2_312-ORcp2_37*qd(3)...
 -RLcp2_37*qdd(3);
    ACcp2_312 = qdd(2)+OMcp2_17*ORcp2_28+OMcp2_18*ORcp2_29+OMcp2_19*ORcp2_212+OPcp2_17*RLcp2_28+OPcp2_18*RLcp2_29+OPcp2_19*RLcp2_212+ORcp2_27*qd(3)...
 +RLcp2_27*qdd(3);

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,2);
    sens.P(2) = POcp2_212;
    sens.P(3) = POcp2_312;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C9p8p3p7;
    sens.R(2,3) = S9p8p3p7;
    sens.R(3,2) = -S9p8p3p7;
    sens.R(3,3) = C9p8p3p7;
    sens.V(2) = VIcp2_212;
    sens.V(3) = VIcp2_312;
    sens.OM(1) = OMcp2_19;
    sens.J(2,1) = (1.0);
    sens.J(2,3) = JTcp2_212_3;
    sens.J(2,7) = JTcp2_212_4;
    sens.J(2,8) = JTcp2_212_5;
    sens.J(2,9) = -RLcp2_312;
    sens.J(3,2) = (1.0);
    sens.J(3,3) = JTcp2_312_3;
    sens.J(3,7) = JTcp2_312_4;
    sens.J(3,8) = JTcp2_312_5;
    sens.J(3,9) = RLcp2_212;
    sens.J(4,3) = (1.0);
    sens.J(4,7) = (1.0);
    sens.J(4,8) = (1.0);
    sens.J(4,9) = (1.0);
    sens.A(2) = ACcp2_212;
    sens.A(3) = ACcp2_312;
    sens.OMP(1) = OPcp2_19;
 
% 
case 4, 


% = = Block_1_0_0_4_0_3 = = 
 
% Sensor Kinematics 


    RLcp3_27 = -s.dpt(3,2)*S3;
    RLcp3_37 = s.dpt(3,2)*C3;
    OMcp3_17 = qd(3)+qd(7);
    ORcp3_27 = -RLcp3_37*qd(3);
    ORcp3_37 = RLcp3_27*qd(3);
    OPcp3_17 = qdd(3)+qdd(7);
    RLcp3_28 = -s.dpt(3,8)*S3p7;
    RLcp3_38 = s.dpt(3,8)*C3p7;
    OMcp3_18 = OMcp3_17+qd(8);
    ORcp3_28 = -OMcp3_17*RLcp3_38;
    ORcp3_38 = OMcp3_17*RLcp3_28;
    OPcp3_18 = OPcp3_17+qdd(8);
    RLcp3_29 = -s.dpt(3,9)*S8p3p7;
    RLcp3_39 = s.dpt(3,9)*C8p3p7;
    OMcp3_19 = OMcp3_18+qd(9);
    ORcp3_29 = -OMcp3_18*RLcp3_39;
    ORcp3_39 = OMcp3_18*RLcp3_29;
    OPcp3_19 = OPcp3_18+qdd(9);
    RLcp3_213 = s.dpt(2,11)*C9p8p3p7-s.dpt(3,11)*S9p8p3p7;
    RLcp3_313 = s.dpt(2,11)*S9p8p3p7+s.dpt(3,11)*C9p8p3p7;
    POcp3_213 = RLcp3_213+RLcp3_27+RLcp3_28+RLcp3_29+q(1);
    POcp3_313 = RLcp3_313+RLcp3_37+RLcp3_38+RLcp3_39+q(2);
    JTcp3_213_3 = -(RLcp3_313+RLcp3_37+RLcp3_38+RLcp3_39);
    JTcp3_313_3 = RLcp3_213+RLcp3_27+RLcp3_28+RLcp3_29;
    JTcp3_213_4 = -(RLcp3_313+RLcp3_38+RLcp3_39);
    JTcp3_313_4 = RLcp3_213+RLcp3_28+RLcp3_29;
    JTcp3_213_5 = -(RLcp3_313+RLcp3_39);
    JTcp3_313_5 = RLcp3_213+RLcp3_29;
    ORcp3_213 = -OMcp3_19*RLcp3_313;
    ORcp3_313 = OMcp3_19*RLcp3_213;
    VIcp3_213 = ORcp3_213+ORcp3_27+ORcp3_28+ORcp3_29+qd(1);
    VIcp3_313 = ORcp3_313+ORcp3_37+ORcp3_38+ORcp3_39+qd(2);
    ACcp3_213 = qdd(1)-OMcp3_17*ORcp3_38-OMcp3_18*ORcp3_39-OMcp3_19*ORcp3_313-OPcp3_17*RLcp3_38-OPcp3_18*RLcp3_39-OPcp3_19*RLcp3_313-ORcp3_37*qd(3)...
 -RLcp3_37*qdd(3);
    ACcp3_313 = qdd(2)+OMcp3_17*ORcp3_28+OMcp3_18*ORcp3_29+OMcp3_19*ORcp3_213+OPcp3_17*RLcp3_28+OPcp3_18*RLcp3_29+OPcp3_19*RLcp3_213+ORcp3_27*qd(3)...
 +RLcp3_27*qdd(3);

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,2);
    sens.P(2) = POcp3_213;
    sens.P(3) = POcp3_313;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C9p8p3p7;
    sens.R(2,3) = S9p8p3p7;
    sens.R(3,2) = -S9p8p3p7;
    sens.R(3,3) = C9p8p3p7;
    sens.V(2) = VIcp3_213;
    sens.V(3) = VIcp3_313;
    sens.OM(1) = OMcp3_19;
    sens.J(2,1) = (1.0);
    sens.J(2,3) = JTcp3_213_3;
    sens.J(2,7) = JTcp3_213_4;
    sens.J(2,8) = JTcp3_213_5;
    sens.J(2,9) = -RLcp3_313;
    sens.J(3,2) = (1.0);
    sens.J(3,3) = JTcp3_313_3;
    sens.J(3,7) = JTcp3_313_4;
    sens.J(3,8) = JTcp3_313_5;
    sens.J(3,9) = RLcp3_213;
    sens.J(4,3) = (1.0);
    sens.J(4,7) = (1.0);
    sens.J(4,8) = (1.0);
    sens.J(4,9) = (1.0);
    sens.A(2) = ACcp3_213;
    sens.A(3) = ACcp3_313;
    sens.OMP(1) = OPcp3_19;
 
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
    OMcp4_15 = OMcp4_14+qd(5);
    ORcp4_25 = -OMcp4_14*RLcp4_35;
    ORcp4_35 = OMcp4_14*RLcp4_25;
    OPcp4_15 = OPcp4_14+qdd(5);
    RLcp4_26 = -s.dpt(3,4)*S5p3p4;
    RLcp4_36 = s.dpt(3,4)*C5p3p4;
    OMcp4_16 = OMcp4_15+qd(6);
    ORcp4_26 = -OMcp4_15*RLcp4_36;
    ORcp4_36 = OMcp4_15*RLcp4_26;
    OPcp4_16 = OPcp4_15+qdd(6);
    RLcp4_214 = -s.dpt(3,5)*S6p5p3p4;
    RLcp4_314 = s.dpt(3,5)*C6p5p3p4;
    POcp4_214 = RLcp4_214+RLcp4_24+RLcp4_25+RLcp4_26+q(1);
    POcp4_314 = RLcp4_314+RLcp4_34+RLcp4_35+RLcp4_36+q(2);
    JTcp4_214_3 = -(RLcp4_314+RLcp4_34+RLcp4_35+RLcp4_36);
    JTcp4_314_3 = RLcp4_214+RLcp4_24+RLcp4_25+RLcp4_26;
    JTcp4_214_4 = -(RLcp4_314+RLcp4_35+RLcp4_36);
    JTcp4_314_4 = RLcp4_214+RLcp4_25+RLcp4_26;
    JTcp4_214_5 = -(RLcp4_314+RLcp4_36);
    JTcp4_314_5 = RLcp4_214+RLcp4_26;
    ORcp4_214 = -OMcp4_16*RLcp4_314;
    ORcp4_314 = OMcp4_16*RLcp4_214;
    VIcp4_214 = ORcp4_214+ORcp4_24+ORcp4_25+ORcp4_26+qd(1);
    VIcp4_314 = ORcp4_314+ORcp4_34+ORcp4_35+ORcp4_36+qd(2);
    ACcp4_214 = qdd(1)-OMcp4_14*ORcp4_35-OMcp4_15*ORcp4_36-OMcp4_16*ORcp4_314-OPcp4_14*RLcp4_35-OPcp4_15*RLcp4_36-OPcp4_16*RLcp4_314-ORcp4_34*qd(3)...
 -RLcp4_34*qdd(3);
    ACcp4_314 = qdd(2)+OMcp4_14*ORcp4_25+OMcp4_15*ORcp4_26+OMcp4_16*ORcp4_214+OPcp4_14*RLcp4_25+OPcp4_15*RLcp4_26+OPcp4_16*RLcp4_214+ORcp4_24*qd(3)...
 +RLcp4_24*qdd(3);

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,1);
    sens.P(2) = POcp4_214;
    sens.P(3) = POcp4_314;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C6p5p3p4;
    sens.R(2,3) = S6p5p3p4;
    sens.R(3,2) = -S6p5p3p4;
    sens.R(3,3) = C6p5p3p4;
    sens.V(2) = VIcp4_214;
    sens.V(3) = VIcp4_314;
    sens.OM(1) = OMcp4_16;
    sens.J(2,1) = (1.0);
    sens.J(2,3) = JTcp4_214_3;
    sens.J(2,4) = JTcp4_214_4;
    sens.J(2,5) = JTcp4_214_5;
    sens.J(2,6) = -RLcp4_314;
    sens.J(3,2) = (1.0);
    sens.J(3,3) = JTcp4_314_3;
    sens.J(3,4) = JTcp4_314_4;
    sens.J(3,5) = JTcp4_314_5;
    sens.J(3,6) = RLcp4_214;
    sens.J(4,3) = (1.0);
    sens.J(4,4) = (1.0);
    sens.J(4,5) = (1.0);
    sens.J(4,6) = (1.0);
    sens.A(2) = ACcp4_214;
    sens.A(3) = ACcp4_314;
    sens.OMP(1) = OPcp4_16;
 
% 
case 6, 


% = = Block_1_0_0_6_0_3 = = 
 
% Sensor Kinematics 


    RLcp5_27 = -s.dpt(3,2)*S3;
    RLcp5_37 = s.dpt(3,2)*C3;
    OMcp5_17 = qd(3)+qd(7);
    ORcp5_27 = -RLcp5_37*qd(3);
    ORcp5_37 = RLcp5_27*qd(3);
    OPcp5_17 = qdd(3)+qdd(7);
    RLcp5_28 = -s.dpt(3,8)*S3p7;
    RLcp5_38 = s.dpt(3,8)*C3p7;
    OMcp5_18 = OMcp5_17+qd(8);
    ORcp5_28 = -OMcp5_17*RLcp5_38;
    ORcp5_38 = OMcp5_17*RLcp5_28;
    OPcp5_18 = OPcp5_17+qdd(8);
    RLcp5_29 = -s.dpt(3,9)*S8p3p7;
    RLcp5_39 = s.dpt(3,9)*C8p3p7;
    OMcp5_19 = OMcp5_18+qd(9);
    ORcp5_29 = -OMcp5_18*RLcp5_39;
    ORcp5_39 = OMcp5_18*RLcp5_29;
    OPcp5_19 = OPcp5_18+qdd(9);
    RLcp5_215 = -s.dpt(3,12)*S9p8p3p7;
    RLcp5_315 = s.dpt(3,12)*C9p8p3p7;
    POcp5_215 = RLcp5_215+RLcp5_27+RLcp5_28+RLcp5_29+q(1);
    POcp5_315 = RLcp5_315+RLcp5_37+RLcp5_38+RLcp5_39+q(2);
    JTcp5_215_3 = -(RLcp5_315+RLcp5_37+RLcp5_38+RLcp5_39);
    JTcp5_315_3 = RLcp5_215+RLcp5_27+RLcp5_28+RLcp5_29;
    JTcp5_215_4 = -(RLcp5_315+RLcp5_38+RLcp5_39);
    JTcp5_315_4 = RLcp5_215+RLcp5_28+RLcp5_29;
    JTcp5_215_5 = -(RLcp5_315+RLcp5_39);
    JTcp5_315_5 = RLcp5_215+RLcp5_29;
    ORcp5_215 = -OMcp5_19*RLcp5_315;
    ORcp5_315 = OMcp5_19*RLcp5_215;
    VIcp5_215 = ORcp5_215+ORcp5_27+ORcp5_28+ORcp5_29+qd(1);
    VIcp5_315 = ORcp5_315+ORcp5_37+ORcp5_38+ORcp5_39+qd(2);
    ACcp5_215 = qdd(1)-OMcp5_17*ORcp5_38-OMcp5_18*ORcp5_39-OMcp5_19*ORcp5_315-OPcp5_17*RLcp5_38-OPcp5_18*RLcp5_39-OPcp5_19*RLcp5_315-ORcp5_37*qd(3)...
 -RLcp5_37*qdd(3);
    ACcp5_315 = qdd(2)+OMcp5_17*ORcp5_28+OMcp5_18*ORcp5_29+OMcp5_19*ORcp5_215+OPcp5_17*RLcp5_28+OPcp5_18*RLcp5_29+OPcp5_19*RLcp5_215+ORcp5_27*qd(3)...
 +RLcp5_27*qdd(3);

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,2);
    sens.P(2) = POcp5_215;
    sens.P(3) = POcp5_315;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C9p8p3p7;
    sens.R(2,3) = S9p8p3p7;
    sens.R(3,2) = -S9p8p3p7;
    sens.R(3,3) = C9p8p3p7;
    sens.V(2) = VIcp5_215;
    sens.V(3) = VIcp5_315;
    sens.OM(1) = OMcp5_19;
    sens.J(2,1) = (1.0);
    sens.J(2,3) = JTcp5_215_3;
    sens.J(2,7) = JTcp5_215_4;
    sens.J(2,8) = JTcp5_215_5;
    sens.J(2,9) = -RLcp5_315;
    sens.J(3,2) = (1.0);
    sens.J(3,3) = JTcp5_315_3;
    sens.J(3,7) = JTcp5_315_4;
    sens.J(3,8) = JTcp5_315_5;
    sens.J(3,9) = RLcp5_215;
    sens.J(4,3) = (1.0);
    sens.J(4,7) = (1.0);
    sens.J(4,8) = (1.0);
    sens.J(4,9) = (1.0);
    sens.A(2) = ACcp5_215;
    sens.A(3) = ACcp5_315;
    sens.OMP(1) = OPcp5_19;

end


% ====== END Task 1 ====== 

  

