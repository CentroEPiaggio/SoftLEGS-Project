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
%	==> Function : F19 : External Forces
%	==> Flops complexity : 238
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,9);
 trq = zeros(3,9);

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

% = = Block_0_0_1_1_0_2 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;
  S5p3p4 = C5*S3p4+S5*C3p4;
  C5p3p4 = C5*C3p4-S5*S3p4;
  S6p5p3p4 = C6*S5p3p4+S6*C5p3p4;
  C6p5p3p4 = C6*C5p3p4-S6*S5p3p4;
 
% Sensor Kinematics 


  RLcp4_24 = -s.dpt(3,1)*S3;
  RLcp4_34 = s.dpt(3,1)*C3;
  OMcp4_14 = qd(3)+qd(4);
  ORcp4_24 = -qd(3)*RLcp4_34;
  ORcp4_34 = qd(3)*RLcp4_24;
  OPcp4_14 = qdd(3)+qdd(4);
  RLcp4_25 = -s.dpt(3,3)*S3p4;
  RLcp4_35 = s.dpt(3,3)*C3p4;
  OMcp4_15 = qd(5)+OMcp4_14;
  ORcp4_25 = -OMcp4_14*RLcp4_35;
  ORcp4_35 = OMcp4_14*RLcp4_25;
  OPcp4_15 = qdd(5)+OPcp4_14;
  RLcp4_26 = -s.dpt(3,4)*S5p3p4;
  RLcp4_36 = s.dpt(3,4)*C5p3p4;
  OMcp4_16 = qd(6)+OMcp4_15;
  ORcp4_26 = -OMcp4_15*RLcp4_36;
  ORcp4_36 = OMcp4_15*RLcp4_26;
  OPcp4_16 = qdd(6)+OPcp4_15;
  RLcp4_214 = -s.dpt(3,5)*S6p5p3p4;
  RLcp4_314 = s.dpt(3,5)*C6p5p3p4;
  ORcp4_214 = -OMcp4_16*RLcp4_314;
  ORcp4_314 = OMcp4_16*RLcp4_214;
  PxF1(1) = s.dpt(1,1);
  PxF1(2) = q(1)+RLcp4_214+RLcp4_24+RLcp4_25+RLcp4_26;
  PxF1(3) = q(2)+RLcp4_314+RLcp4_34+RLcp4_35+RLcp4_36;
  RxF1(1,1) = (1.0);
  RxF1(1,2) = 0;
  RxF1(1,3) = 0;
  RxF1(2,1) = 0;
  RxF1(2,2) = C6p5p3p4;
  RxF1(2,3) = S6p5p3p4;
  RxF1(3,1) = 0;
  RxF1(3,2) = -S6p5p3p4;
  RxF1(3,3) = C6p5p3p4;
  VxF1(1) = 0;
  VxF1(2) = qd(1)+ORcp4_214+ORcp4_24+ORcp4_25+ORcp4_26;
  VxF1(3) = qd(2)+ORcp4_314+ORcp4_34+ORcp4_35+ORcp4_36;
  OMxF1(1) = OMcp4_16;
  OMxF1(2) = 0;
  OMxF1(3) = 0;
  AxF1(1) = 0;
  AxF1(2) = qdd(1)-qd(3)*ORcp4_34-qdd(3)*RLcp4_34-OMcp4_14*ORcp4_35-OMcp4_15*ORcp4_36-OMcp4_16*ORcp4_314-OPcp4_14*RLcp4_35-OPcp4_15*RLcp4_36-...
 OPcp4_16*RLcp4_314;
  AxF1(3) = qdd(2)+qd(3)*ORcp4_24+qdd(3)*RLcp4_24+OMcp4_14*ORcp4_25+OMcp4_15*ORcp4_26+OMcp4_16*ORcp4_214+OPcp4_14*RLcp4_25+OPcp4_15*RLcp4_26+...
 OPcp4_16*RLcp4_214;
  OMPxF1(1) = OPcp4_16;
  OMPxF1(2) = 0;
  OMPxF1(3) = 0;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = SWr1(1);
  xfrc25 = SWr1(2)*C6p5p3p4+SWr1(3)*S6p5p3p4;
  xfrc35 = -(SWr1(2)*S6p5p3p4-SWr1(3)*C6p5p3p4);
  frc(1,6) = s.frc(1,6)+SWr1(1);
  frc(2,6) = s.frc(2,6)+xfrc25;
  frc(3,6) = s.frc(3,6)+xfrc35;
  xtrq15 = SWr1(4);
  xtrq25 = SWr1(5)*C6p5p3p4+SWr1(6)*S6p5p3p4;
  xtrq35 = -(SWr1(5)*S6p5p3p4-SWr1(6)*C6p5p3p4);
  trq(1,6) = s.trq(1,6)+xtrq15-xfrc25*(SWr1(9)-s.l(3,6))+xfrc35*SWr1(8);
  trq(2,6) = s.trq(2,6)+xtrq25+xfrc15*(SWr1(9)-s.l(3,6))-xfrc35*SWr1(7);
  trq(3,6) = s.trq(3,6)+xtrq35-xfrc15*SWr1(8)+xfrc25*SWr1(7);

% = = Block_0_0_1_2_0_3 = = 
 
% Trigonometric Variables  

  S3p7 = C3*S7+S3*C7;
  C3p7 = C3*C7-S3*S7;
  S8p3p7 = C8*S3p7+S8*C3p7;
  C8p3p7 = C8*C3p7-S8*S3p7;
  S9p8p3p7 = C9*S8p3p7+S9*C8p3p7;
  C9p8p3p7 = C9*C8p3p7-S9*S8p3p7;
 
% Sensor Kinematics 


  RLcp5_27 = -s.dpt(3,2)*S3;
  RLcp5_37 = s.dpt(3,2)*C3;
  OMcp5_17 = qd(3)+qd(7);
  ORcp5_27 = -qd(3)*RLcp5_37;
  ORcp5_37 = qd(3)*RLcp5_27;
  OPcp5_17 = qdd(3)+qdd(7);
  RLcp5_28 = -s.dpt(3,8)*S3p7;
  RLcp5_38 = s.dpt(3,8)*C3p7;
  OMcp5_18 = qd(8)+OMcp5_17;
  ORcp5_28 = -OMcp5_17*RLcp5_38;
  ORcp5_38 = OMcp5_17*RLcp5_28;
  OPcp5_18 = qdd(8)+OPcp5_17;
  RLcp5_29 = -s.dpt(3,9)*S8p3p7;
  RLcp5_39 = s.dpt(3,9)*C8p3p7;
  OMcp5_19 = qd(9)+OMcp5_18;
  ORcp5_29 = -OMcp5_18*RLcp5_39;
  ORcp5_39 = OMcp5_18*RLcp5_29;
  OPcp5_19 = qdd(9)+OPcp5_18;
  RLcp5_215 = -s.dpt(3,12)*S9p8p3p7;
  RLcp5_315 = s.dpt(3,12)*C9p8p3p7;
  ORcp5_215 = -OMcp5_19*RLcp5_315;
  ORcp5_315 = OMcp5_19*RLcp5_215;
  PxF2(1) = s.dpt(1,2);
  PxF2(2) = q(1)+RLcp5_215+RLcp5_27+RLcp5_28+RLcp5_29;
  PxF2(3) = q(2)+RLcp5_315+RLcp5_37+RLcp5_38+RLcp5_39;
  RxF2(1,1) = (1.0);
  RxF2(1,2) = 0;
  RxF2(1,3) = 0;
  RxF2(2,1) = 0;
  RxF2(2,2) = C9p8p3p7;
  RxF2(2,3) = S9p8p3p7;
  RxF2(3,1) = 0;
  RxF2(3,2) = -S9p8p3p7;
  RxF2(3,3) = C9p8p3p7;
  VxF2(1) = 0;
  VxF2(2) = qd(1)+ORcp5_215+ORcp5_27+ORcp5_28+ORcp5_29;
  VxF2(3) = qd(2)+ORcp5_315+ORcp5_37+ORcp5_38+ORcp5_39;
  OMxF2(1) = OMcp5_19;
  OMxF2(2) = 0;
  OMxF2(3) = 0;
  AxF2(1) = 0;
  AxF2(2) = qdd(1)-qd(3)*ORcp5_37-qdd(3)*RLcp5_37-OMcp5_17*ORcp5_38-OMcp5_18*ORcp5_39-OMcp5_19*ORcp5_315-OPcp5_17*RLcp5_38-OPcp5_18*RLcp5_39-...
 OPcp5_19*RLcp5_315;
  AxF2(3) = qdd(2)+qd(3)*ORcp5_27+qdd(3)*RLcp5_27+OMcp5_17*ORcp5_28+OMcp5_18*ORcp5_29+OMcp5_19*ORcp5_215+OPcp5_17*RLcp5_28+OPcp5_18*RLcp5_29+...
 OPcp5_19*RLcp5_215;
  OMPxF2(1) = OPcp5_19;
  OMPxF2(2) = 0;
  OMPxF2(3) = 0;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = SWr2(1);
  xfrc26 = SWr2(2)*C9p8p3p7+SWr2(3)*S9p8p3p7;
  xfrc36 = -(SWr2(2)*S9p8p3p7-SWr2(3)*C9p8p3p7);
  frc(1,9) = s.frc(1,9)+SWr2(1);
  frc(2,9) = s.frc(2,9)+xfrc26;
  frc(3,9) = s.frc(3,9)+xfrc36;
  xtrq16 = SWr2(4);
  xtrq26 = SWr2(5)*C9p8p3p7+SWr2(6)*S9p8p3p7;
  xtrq36 = -(SWr2(5)*S9p8p3p7-SWr2(6)*C9p8p3p7);
  trq(1,9) = s.trq(1,9)+xtrq16-xfrc26*(SWr2(9)-s.l(3,9))+xfrc36*SWr2(8);
  trq(2,9) = s.trq(2,9)+xtrq26+xfrc16*(SWr2(9)-s.l(3,9))-xfrc36*SWr2(7);
  trq(3,9) = s.trq(3,9)+xtrq36-xfrc16*SWr2(8)+xfrc26*SWr2(7);

% = = Block_0_0_1_2_1_0 = = 
 
% Symbolic Outputs  

  frc(1,3) = s.frc(1,3);
  frc(2,3) = s.frc(2,3);
  frc(3,3) = s.frc(3,3);
  frc(1,4) = s.frc(1,4);
  frc(2,4) = s.frc(2,4);
  frc(3,4) = s.frc(3,4);
  frc(1,5) = s.frc(1,5);
  frc(2,5) = s.frc(2,5);
  frc(3,5) = s.frc(3,5);
  frc(1,7) = s.frc(1,7);
  frc(2,7) = s.frc(2,7);
  frc(3,7) = s.frc(3,7);
  frc(1,8) = s.frc(1,8);
  frc(2,8) = s.frc(2,8);
  frc(3,8) = s.frc(3,8);
  trq(1,3) = s.trq(1,3);
  trq(2,3) = s.trq(2,3);
  trq(3,3) = s.trq(3,3);
  trq(1,4) = s.trq(1,4);
  trq(2,4) = s.trq(2,4);
  trq(3,4) = s.trq(3,4);
  trq(1,5) = s.trq(1,5);
  trq(2,5) = s.trq(2,5);
  trq(3,5) = s.trq(3,5);
  trq(1,7) = s.trq(1,7);
  trq(2,7) = s.trq(2,7);
  trq(3,7) = s.trq(3,7);
  trq(1,8) = s.trq(1,8);
  trq(2,8) = s.trq(2,8);
  trq(3,8) = s.trq(3,8);

% ====== END Task 0 ====== 

  

