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
%	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
%	==> Flops complexity : 763
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.020 seconds
%
%-------------------------------------------------------------
%
function [M,c] = dirdyna(s,tsim,usrfun)

 M = zeros(9,9);
 c = zeros(9,1);

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

% = = Block_0_1_0_0_0_1 = = 
 
% Forward Kinematics 

  BS93 = -qd(3)*qd(3);
  AlF23 = -s.g(3)*S3;
  AlF33 = -s.g(3)*C3;

% = = Block_0_1_0_1_0_2 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;
  S5p3p4 = C5*S3p4+S5*C3p4;
  C5p3p4 = C5*C3p4-S5*S3p4;
  S6p5p3p4 = C6*S5p3p4+S6*C5p3p4;
  C6p5p3p4 = C6*C5p3p4-S6*S5p3p4;
 
% Forward Kinematics 

  OM14 = qd(3)+qd(4);
  BS94 = -OM14*OM14;
  AlF24 = AlF23*C4+S4*(AlF33+BS93*s.dpt(3,1));
  AlF34 = -(AlF23*S4-C4*(AlF33+BS93*s.dpt(3,1)));
  AlM24_3 = -s.dpt(3,1)*C4;
  AlM34_3 = s.dpt(3,1)*S4;
  OM15 = qd(5)+OM14;
  BS95 = -OM15*OM15;
  AlF25 = AlF24*C5+S5*(AlF34+BS94*s.dpt(3,3));
  AlF35 = -(AlF24*S5-C5*(AlF34+BS94*s.dpt(3,3)));
  AlM25_3 = AlM34_3*S5+C5*(AlM24_3-s.dpt(3,3));
  AlM35_3 = AlM34_3*C5-S5*(AlM24_3-s.dpt(3,3));
  AlM25_4 = -s.dpt(3,3)*C5;
  AlM35_4 = s.dpt(3,3)*S5;
  OM16 = qd(6)+OM15;

% = = Block_0_1_0_1_0_3 = = 
 
% Trigonometric Variables  

  S3p7 = C3*S7+S3*C7;
  C3p7 = C3*C7-S3*S7;
  S8p3p7 = C8*S3p7+S8*C3p7;
  C8p3p7 = C8*C3p7-S8*S3p7;
  S9p8p3p7 = C9*S8p3p7+S9*C8p3p7;
  C9p8p3p7 = C9*C8p3p7-S9*S8p3p7;
 
% Forward Kinematics 

  OM17 = qd(3)+qd(7);
  BS97 = -OM17*OM17;
  AlF27 = AlF23*C7+S7*(AlF33+BS93*s.dpt(3,2));
  AlF37 = -(AlF23*S7-C7*(AlF33+BS93*s.dpt(3,2)));
  AlM27_3 = -s.dpt(3,2)*C7;
  AlM37_3 = s.dpt(3,2)*S7;
  OM18 = qd(8)+OM17;
  BS98 = -OM18*OM18;
  AlF28 = AlF27*C8+S8*(AlF37+BS97*s.dpt(3,8));
  AlF38 = -(AlF27*S8-C8*(AlF37+BS97*s.dpt(3,8)));
  AlM28_3 = AlM37_3*S8+C8*(AlM27_3-s.dpt(3,8));
  AlM38_3 = AlM37_3*C8-S8*(AlM27_3-s.dpt(3,8));
  AlM28_7 = -s.dpt(3,8)*C8;
  AlM38_7 = s.dpt(3,8)*S8;
  OM19 = qd(9)+OM18;

% = = Block_0_2_0_1_0_2 = = 
 
% Backward Dynamics 

  FA26 = -(s.frc(2,6)-s.m(6)*(AlF25*C6+S6*(AlF35+BS95*s.dpt(3,4))));
  FA36 = -(s.frc(3,6)+s.m(6)*(AlF25*S6+OM16*OM16*s.l(3,6)-C6*(AlF35+BS95*s.dpt(3,4))));
  CF16 = -(s.trq(1,6)+FA26*s.l(3,6));
  CM16_1 = -s.m(6)*s.l(3,6)*C6p5p3p4;
  CM16_2 = -s.m(6)*s.l(3,6)*S6p5p3p4;
  FB26_3 = s.m(6)*(AlM35_3*S6+C6*(AlM25_3-s.dpt(3,4))-s.l(3,6));
  FB36_3 = s.m(6)*(AlM35_3*C6-S6*(AlM25_3-s.dpt(3,4)));
  CM16_3 = s.In(1,6)-FB26_3*s.l(3,6);
  FB26_4 = s.m(6)*(AlM35_4*S6+C6*(AlM25_4-s.dpt(3,4))-s.l(3,6));
  FB36_4 = s.m(6)*(AlM35_4*C6-S6*(AlM25_4-s.dpt(3,4)));
  CM16_4 = s.In(1,6)-FB26_4*s.l(3,6);
  FB26_5 = -s.m(6)*(s.l(3,6)+s.dpt(3,4)*C6);
  CM16_5 = s.In(1,6)-FB26_5*s.l(3,6);
  CM16_6 = s.In(1,6)+s.m(6)*s.l(3,6)*s.l(3,6);
  FA25 = -(s.frc(2,5)-s.m(5)*AlF25);
  FF25 = FA25+FA26*C6-FA36*S6;
  FF35 = -(s.frc(3,5)-s.m(5)*(AlF35+BS95*s.l(3,5))-FA26*S6-FA36*C6);
  CF15 = -(s.trq(1,5)-CF16+FA25*s.l(3,5)+s.dpt(3,4)*(FA26*C6-FA36*S6));
  FB25_1 = s.m(5)*C5p3p4;
  FM25_1 = FB25_1+s.m(6)*C5p3p4;
  FM35_1 = -S5p3p4*(s.m(5)+s.m(6));
  CM15_1 = -(s.m(6)*(s.dpt(3,4)*C5p3p4+s.l(3,6)*C6p5p3p4)+FB25_1*s.l(3,5));
  FB25_2 = s.m(5)*S5p3p4;
  CM15_2 = -(s.m(6)*(s.dpt(3,4)*S5p3p4+s.l(3,6)*S6p5p3p4)+FB25_2*s.l(3,5));
  FB25_3 = s.m(5)*(AlM25_3-s.l(3,5));
  FM25_3 = FB25_3+FB26_3*C6-FB36_3*S6;
  FM35_3 = s.m(5)*AlM35_3+FB26_3*S6+FB36_3*C6;
  CM15_3 = s.In(1,5)+CM16_3-FB25_3*s.l(3,5)-s.dpt(3,4)*(FB26_3*C6-FB36_3*S6);
  FB25_4 = s.m(5)*(AlM25_4-s.l(3,5));
  CM15_4 = s.In(1,5)+CM16_4-FB25_4*s.l(3,5)-s.dpt(3,4)*(FB26_4*C6-FB36_4*S6);
  CM15_5 = s.In(1,5)+CM16_5+s.m(5)*s.l(3,5)*s.l(3,5)+s.dpt(3,4)*(s.m(6)*s.dpt(3,4)*S6*S6-FB26_5*C6);
  FA24 = -(s.frc(2,4)-s.m(4)*AlF24);
  FF24 = FA24+FF25*C5-FF35*S5;
  FF34 = -(s.frc(3,4)-s.m(4)*(AlF34+BS94*s.l(3,4))-FF25*S5-FF35*C5);
  CF14 = -(s.trq(1,4)-CF15+FA24*s.l(3,4)+s.dpt(3,3)*(FF25*C5-FF35*S5));
  FB24_1 = s.m(4)*C3p4;
  FM24_1 = FB24_1+FM25_1*C5-FM35_1*S5;
  FM34_1 = FM25_1*S5+FM35_1*C5-s.m(4)*S3p4;
  CM14_1 = CM15_1-FB24_1*s.l(3,4)-s.dpt(3,3)*(FM25_1*C5-FM35_1*S5);
  FB24_2 = s.m(4)*S3p4;
  CM14_2 = CM15_2-FB24_2*s.l(3,4)-s.dpt(3,3)*(C5*(FB25_2+s.m(6)*S5p3p4)-S5*C5p3p4*(s.m(5)+s.m(6)));
  FB24_3 = s.m(4)*(AlM24_3-s.l(3,4));
  CM14_3 = s.In(1,4)+CM15_3-FB24_3*s.l(3,4)-s.dpt(3,3)*(FM25_3*C5-FM35_3*S5);
  CM14_4 = s.In(1,4)+CM15_4+s.m(4)*s.l(3,4)*s.l(3,4)-s.dpt(3,3)*(C5*(FB25_4+FB26_4*C6-FB36_4*S6)-S5*(s.m(5)*AlM35_4+FB26_4*S6+FB36_4*C6));

% = = Block_0_2_0_1_0_3 = = 
 
% Backward Dynamics 

  FA29 = -(s.frc(2,9)-s.m(9)*(AlF28*C9+S9*(AlF38+BS98*s.dpt(3,9))));
  FA39 = -(s.frc(3,9)+s.m(9)*(AlF28*S9+OM19*OM19*s.l(3,9)-C9*(AlF38+BS98*s.dpt(3,9))));
  CF19 = -(s.trq(1,9)+FA29*s.l(3,9));
  CM19_1 = -s.m(9)*s.l(3,9)*C9p8p3p7;
  CM19_2 = -s.m(9)*s.l(3,9)*S9p8p3p7;
  FB29_3 = s.m(9)*(AlM38_3*S9+C9*(AlM28_3-s.dpt(3,9))-s.l(3,9));
  FB39_3 = s.m(9)*(AlM38_3*C9-S9*(AlM28_3-s.dpt(3,9)));
  CM19_3 = s.In(1,9)-FB29_3*s.l(3,9);
  FB29_7 = s.m(9)*(AlM38_7*S9+C9*(AlM28_7-s.dpt(3,9))-s.l(3,9));
  FB39_7 = s.m(9)*(AlM38_7*C9-S9*(AlM28_7-s.dpt(3,9)));
  CM19_7 = s.In(1,9)-FB29_7*s.l(3,9);
  FB29_8 = -s.m(9)*(s.l(3,9)+s.dpt(3,9)*C9);
  CM19_8 = s.In(1,9)-FB29_8*s.l(3,9);
  CM19_9 = s.In(1,9)+s.m(9)*s.l(3,9)*s.l(3,9);
  FA28 = -(s.frc(2,8)-s.m(8)*AlF28);
  FF28 = FA28+FA29*C9-FA39*S9;
  FF38 = -(s.frc(3,8)-s.m(8)*(AlF38+BS98*s.l(3,8))-FA29*S9-FA39*C9);
  CF18 = -(s.trq(1,8)-CF19+FA28*s.l(3,8)+s.dpt(3,9)*(FA29*C9-FA39*S9));
  FB28_1 = s.m(8)*C8p3p7;
  FM28_1 = FB28_1+s.m(9)*C8p3p7;
  FM38_1 = -S8p3p7*(s.m(8)+s.m(9));
  CM18_1 = -(s.m(9)*(s.dpt(3,9)*C8p3p7+s.l(3,9)*C9p8p3p7)+FB28_1*s.l(3,8));
  FB28_2 = s.m(8)*S8p3p7;
  CM18_2 = -(s.m(9)*(s.dpt(3,9)*S8p3p7+s.l(3,9)*S9p8p3p7)+FB28_2*s.l(3,8));
  FB28_3 = s.m(8)*(AlM28_3-s.l(3,8));
  FM28_3 = FB28_3+FB29_3*C9-FB39_3*S9;
  FM38_3 = s.m(8)*AlM38_3+FB29_3*S9+FB39_3*C9;
  CM18_3 = s.In(1,8)+CM19_3-FB28_3*s.l(3,8)-s.dpt(3,9)*(FB29_3*C9-FB39_3*S9);
  FB28_7 = s.m(8)*(AlM28_7-s.l(3,8));
  CM18_7 = s.In(1,8)+CM19_7-FB28_7*s.l(3,8)-s.dpt(3,9)*(FB29_7*C9-FB39_7*S9);
  CM18_8 = s.In(1,8)+CM19_8+s.m(8)*s.l(3,8)*s.l(3,8)+s.dpt(3,9)*(s.m(9)*s.dpt(3,9)*S9*S9-FB29_8*C9);
  FA27 = -(s.frc(2,7)-s.m(7)*AlF27);
  FF27 = FA27+FF28*C8-FF38*S8;
  FF37 = -(s.frc(3,7)-s.m(7)*(AlF37+BS97*s.l(3,7))-FF28*S8-FF38*C8);
  CF17 = -(s.trq(1,7)-CF18+FA27*s.l(3,7)+s.dpt(3,8)*(FF28*C8-FF38*S8));
  FB27_1 = s.m(7)*C3p7;
  FM27_1 = FB27_1+FM28_1*C8-FM38_1*S8;
  FM37_1 = FM28_1*S8+FM38_1*C8-s.m(7)*S3p7;
  CM17_1 = CM18_1-FB27_1*s.l(3,7)-s.dpt(3,8)*(FM28_1*C8-FM38_1*S8);
  FB27_2 = s.m(7)*S3p7;
  CM17_2 = CM18_2-FB27_2*s.l(3,7)-s.dpt(3,8)*(C8*(FB28_2+s.m(9)*S8p3p7)-S8*C8p3p7*(s.m(8)+s.m(9)));
  FB27_3 = s.m(7)*(AlM27_3-s.l(3,7));
  CM17_3 = s.In(1,7)+CM18_3-FB27_3*s.l(3,7)-s.dpt(3,8)*(FM28_3*C8-FM38_3*S8);
  CM17_7 = s.In(1,7)+CM18_7+s.m(7)*s.l(3,7)*s.l(3,7)-s.dpt(3,8)*(C8*(FB28_7+FB29_7*C9-FB39_7*S9)-S8*(s.m(8)*AlM38_7+FB29_7*S9+FB39_7*C9));

% = = Block_0_2_0_2_0_1 = = 
 
% Backward Dynamics 

  FA23 = -(s.frc(2,3)-s.m(3)*AlF23);
  FF23 = FA23+FF24*C4+FF27*C7-FF34*S4-FF37*S7;
  FF33 = -(s.frc(3,3)-s.m(3)*(AlF33+BS93*s.l(3,3))-FF24*S4-FF27*S7-FF34*C4-FF37*C7);
  CF13 = -(s.trq(1,3)-CF14-CF17+FA23*s.l(3,3)+s.dpt(3,1)*(FF24*C4-FF34*S4)+s.dpt(3,2)*(FF27*C7-FF37*S7));
  FB23_1 = s.m(3)*C3;
  FM23_1 = FB23_1+FM24_1*C4+FM27_1*C7-FM34_1*S4-FM37_1*S7;
  FM33_1 = FM27_1*S7+FM37_1*C7-s.m(3)*S3+FM24_1*S4+FM34_1*C4;
  CM13_1 = CM14_1+CM17_1-FB23_1*s.l(3,3)-s.dpt(3,1)*(FM24_1*C4-FM34_1*S4)-s.dpt(3,2)*(FM27_1*C7-FM37_1*S7);
  FB23_2 = s.m(3)*S3;
  CM13_2 = CM14_2+CM17_2-FB23_2*s.l(3,3)-s.dpt(3,1)*(C4*(FB24_2+S3p4*(s.m(5)+s.m(6)))-S4*C3p4*(s.m(4)+s.m(5)+s.m(6)))-s.dpt(3,2)*(C7*(FB27_2+S3p7...
 *(s.m(8)+s.m(9)))-S7*C3p7*(s.m(7)+s.m(8)+s.m(9)));
  CM13_3 = s.In(1,3)+CM14_3+CM17_3+s.m(3)*s.l(3,3)*s.l(3,3)-s.dpt(3,1)*(C4*(FB24_3+FM25_3*C5-FM35_3*S5)-S4*(s.m(4)*AlM34_3+FM25_3*S5+FM35_3*C5))-...
 s.dpt(3,2)*(C7*(FB27_3+FM28_3*C8-FM38_3*S8)-S7*(s.m(7)*AlM37_3+FM28_3*S8+FM38_3*C8));
  FF2_23 = FF23*C3-FF33*S3;
  FF2_33 = FF23*S3+FF33*C3;
  FM21_23 = FM23_1*C3-FM33_1*S3;
  FM21_33 = FM23_1*S3+FM33_1*C3;
  FM22_33 = C3*C3*(s.m(3)+s.m(4)+s.m(5)+s.m(6)+s.m(7)+s.m(8)+s.m(9))+S3*(FB23_2+S3*(s.m(4)+s.m(5)+s.m(6))+S3*(s.m(7)+s.m(8)+s.m(9)));

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  M(1,1) = FM21_23;
  M(1,2) = FM21_33;
  M(1,3) = CM13_1;
  M(1,4) = CM14_1;
  M(1,5) = CM15_1;
  M(1,6) = CM16_1;
  M(1,7) = CM17_1;
  M(1,8) = CM18_1;
  M(1,9) = CM19_1;
  M(2,1) = FM21_33;
  M(2,2) = FM22_33;
  M(2,3) = CM13_2;
  M(2,4) = CM14_2;
  M(2,5) = CM15_2;
  M(2,6) = CM16_2;
  M(2,7) = CM17_2;
  M(2,8) = CM18_2;
  M(2,9) = CM19_2;
  M(3,1) = CM13_1;
  M(3,2) = CM13_2;
  M(3,3) = CM13_3;
  M(3,4) = CM14_3;
  M(3,5) = CM15_3;
  M(3,6) = CM16_3;
  M(3,7) = CM17_3;
  M(3,8) = CM18_3;
  M(3,9) = CM19_3;
  M(4,1) = CM14_1;
  M(4,2) = CM14_2;
  M(4,3) = CM14_3;
  M(4,4) = CM14_4;
  M(4,5) = CM15_4;
  M(4,6) = CM16_4;
  M(5,1) = CM15_1;
  M(5,2) = CM15_2;
  M(5,3) = CM15_3;
  M(5,4) = CM15_4;
  M(5,5) = CM15_5;
  M(5,6) = CM16_5;
  M(6,1) = CM16_1;
  M(6,2) = CM16_2;
  M(6,3) = CM16_3;
  M(6,4) = CM16_4;
  M(6,5) = CM16_5;
  M(6,6) = CM16_6;
  M(7,1) = CM17_1;
  M(7,2) = CM17_2;
  M(7,3) = CM17_3;
  M(7,7) = CM17_7;
  M(7,8) = CM18_7;
  M(7,9) = CM19_7;
  M(8,1) = CM18_1;
  M(8,2) = CM18_2;
  M(8,3) = CM18_3;
  M(8,7) = CM18_7;
  M(8,8) = CM18_8;
  M(8,9) = CM19_8;
  M(9,1) = CM19_1;
  M(9,2) = CM19_2;
  M(9,3) = CM19_3;
  M(9,7) = CM19_7;
  M(9,8) = CM19_8;
  M(9,9) = CM19_9;
  c(1) = FF2_23;
  c(2) = FF2_33;
  c(3) = CF13;
  c(4) = CF14;
  c(5) = CF15;
  c(6) = CF16;
  c(7) = CF17;
  c(8) = CF18;
  c(9) = CF19;

% ====== END Task 0 ====== 

  

