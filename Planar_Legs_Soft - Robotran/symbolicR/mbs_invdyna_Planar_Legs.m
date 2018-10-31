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
%	==> Function : F 2 : Inverse Dynamics : RNEA
%	==> Flops complexity : 325
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [Qq] = invdyna(s,tsim,usrfun)

 Qq = zeros(9,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

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

% = = Block_0_1_0_0_0_0 = = 
 
% Forward Kinematics 

  ALPHA32 = qdd(2)-s.g(3);
  BS93 = -qd(3)*qd(3);
  ALPHA23 = qdd(1)*C3+ALPHA32*S3;
  ALPHA33 = -(qdd(1)*S3-ALPHA32*C3);
  OM14 = qd(3)+qd(4);
  OMp14 = qdd(3)+qdd(4);
  BS94 = -OM14*OM14;
  ALPHA24 = C4*(ALPHA23-qdd(3)*s.dpt(3,1))+S4*(ALPHA33+BS93*s.dpt(3,1));
  ALPHA34 = C4*(ALPHA33+BS93*s.dpt(3,1))-S4*(ALPHA23-qdd(3)*s.dpt(3,1));
  OM15 = qd(5)+OM14;
  OMp15 = qdd(5)+OMp14;
  BS95 = -OM15*OM15;
  ALPHA25 = C5*(ALPHA24-OMp14*s.dpt(3,3))+S5*(ALPHA34+BS94*s.dpt(3,3));
  ALPHA35 = C5*(ALPHA34+BS94*s.dpt(3,3))-S5*(ALPHA24-OMp14*s.dpt(3,3));
  OM16 = qd(6)+OM15;
  OMp16 = qdd(6)+OMp15;
  OM17 = qd(3)+qd(7);
  OMp17 = qdd(3)+qdd(7);
  BS97 = -OM17*OM17;
  ALPHA27 = C7*(ALPHA23-qdd(3)*s.dpt(3,2))+S7*(ALPHA33+BS93*s.dpt(3,2));
  ALPHA37 = C7*(ALPHA33+BS93*s.dpt(3,2))-S7*(ALPHA23-qdd(3)*s.dpt(3,2));
  OM18 = qd(8)+OM17;
  OMp18 = qdd(8)+OMp17;
  BS98 = -OM18*OM18;
  ALPHA28 = C8*(ALPHA27-OMp17*s.dpt(3,8))+S8*(ALPHA37+BS97*s.dpt(3,8));
  ALPHA38 = C8*(ALPHA37+BS97*s.dpt(3,8))-S8*(ALPHA27-OMp17*s.dpt(3,8));
  OM19 = qd(9)+OM18;
  OMp19 = qdd(9)+OMp18;
 
% Backward Dynamics 

  Fs29 = -(s.frc(2,9)+s.m(9)*(OMp19*s.l(3,9)-C9*(ALPHA28-OMp18*s.dpt(3,9))-S9*(ALPHA38+BS98*s.dpt(3,9))));
  Fs39 = -(s.frc(3,9)+s.m(9)*(OM19*OM19*s.l(3,9)-C9*(ALPHA38+BS98*s.dpt(3,9))+S9*(ALPHA28-OMp18*s.dpt(3,9))));
  Cq19 = -(s.trq(1,9)-s.In(1,9)*OMp19+Fs29*s.l(3,9));
  Fs28 = -(s.frc(2,8)-s.m(8)*(ALPHA28-OMp18*s.l(3,8)));
  Fq28 = Fs28+Fs29*C9-Fs39*S9;
  Fq38 = -(s.frc(3,8)-s.m(8)*(ALPHA38+BS98*s.l(3,8))-Fs29*S9-Fs39*C9);
  Cq18 = -(s.trq(1,8)-Cq19-s.In(1,8)*OMp18+Fs28*s.l(3,8)+s.dpt(3,9)*(Fs29*C9-Fs39*S9));
  Fs27 = -(s.frc(2,7)-s.m(7)*(ALPHA27-OMp17*s.l(3,7)));
  Fq27 = Fs27+Fq28*C8-Fq38*S8;
  Fq37 = -(s.frc(3,7)-s.m(7)*(ALPHA37+BS97*s.l(3,7))-Fq28*S8-Fq38*C8);
  Cq17 = -(s.trq(1,7)-Cq18-s.In(1,7)*OMp17+Fs27*s.l(3,7)+s.dpt(3,8)*(Fq28*C8-Fq38*S8));
  Fs26 = -(s.frc(2,6)+s.m(6)*(OMp16*s.l(3,6)-C6*(ALPHA25-OMp15*s.dpt(3,4))-S6*(ALPHA35+BS95*s.dpt(3,4))));
  Fs36 = -(s.frc(3,6)+s.m(6)*(OM16*OM16*s.l(3,6)-C6*(ALPHA35+BS95*s.dpt(3,4))+S6*(ALPHA25-OMp15*s.dpt(3,4))));
  Cq16 = -(s.trq(1,6)-s.In(1,6)*OMp16+Fs26*s.l(3,6));
  Fs25 = -(s.frc(2,5)-s.m(5)*(ALPHA25-OMp15*s.l(3,5)));
  Fq25 = Fs25+Fs26*C6-Fs36*S6;
  Fq35 = -(s.frc(3,5)-s.m(5)*(ALPHA35+BS95*s.l(3,5))-Fs26*S6-Fs36*C6);
  Cq15 = -(s.trq(1,5)-Cq16-s.In(1,5)*OMp15+Fs25*s.l(3,5)+s.dpt(3,4)*(Fs26*C6-Fs36*S6));
  Fs24 = -(s.frc(2,4)-s.m(4)*(ALPHA24-OMp14*s.l(3,4)));
  Fq24 = Fs24+Fq25*C5-Fq35*S5;
  Fq34 = -(s.frc(3,4)-s.m(4)*(ALPHA34+BS94*s.l(3,4))-Fq25*S5-Fq35*C5);
  Cq14 = -(s.trq(1,4)-Cq15-s.In(1,4)*OMp14+Fs24*s.l(3,4)+s.dpt(3,3)*(Fq25*C5-Fq35*S5));
  Fs23 = -(s.frc(2,3)-s.m(3)*(ALPHA23-qdd(3)*s.l(3,3)));
  Fq23 = Fs23+Fq24*C4+Fq27*C7-Fq34*S4-Fq37*S7;
  Fq33 = -(s.frc(3,3)-s.m(3)*(ALPHA33+BS93*s.l(3,3))-Fq24*S4-Fq27*S7-Fq34*C4-Fq37*C7);
  Cq13 = -(s.trq(1,3)-Cq14-Cq17-qdd(3)*s.In(1,3)+Fs23*s.l(3,3)+s.dpt(3,1)*(Fq24*C4-Fq34*S4)+s.dpt(3,2)*(Fq27*C7-Fq37*S7));
  Fq22 = Fq23*C3-Fq33*S3;
  Fq32 = Fq23*S3+Fq33*C3;

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  Qq(1) = Fq22;
  Qq(2) = Fq32;
  Qq(3) = Cq13;
  Qq(4) = Cq14;
  Qq(5) = Cq15;
  Qq(6) = Cq16;
  Qq(7) = Cq17;
  Qq(8) = Cq18;
  Qq(9) = Cq19;

% ====== END Task 0 ====== 

  

