% Create the optimal control problem 

import casadi.*


% declaration of the global user structure
global MBS_user;

% initialization of the user field "process"
MBS_user.process = '';

% loading of walkman_robotran.mbs with default options
[MBS_data, MBS_info] = mbs_load('Planar_Legs_Soft','default');

% conversion of symbolic files to casadi syntax
converter_to_casadi_syntax(MBS_data,'mbs_dirdyna_Planar_Legs');
converter_to_casadi_syntax(MBS_data,'mbs_extforces_Planar_Legs');
converter_to_casadi_syntax(MBS_data,'mbs_invdyna_Planar_Legs');
converter_to_casadi_syntax(MBS_data,'mbs_sensor_Planar_Legs');

% Total mass
m_robot = sum(MBS_data.m);
grav_acc = 9.81;

% System states
q = SX.sym('q',   MBS_data.Njoint);
qd = SX.sym('qd',  MBS_data.Njoint);
qdd = SX.sym('qdd', MBS_data.Njoint);

MBS_data.q = q;
MBS_data.qd = qd; 
MBS_data.qdd = qdd;

% actuators stiffness
K_el = SX.sym('K', MBS_data.nqa);

% input torques
MBS_data.QqRHS = cell(MBS_data.Njoint,1);
MBS_data.el_term = cell(MBS_data.nqa,1);
MBS_data.theta = cell(MBS_data.nqa,1);
MBS_data.thetad = cell(MBS_data.nqa,1);
MBS_data.thetadd = cell(MBS_data.nqa,1);
MBS_data.inputs = cell(MBS_data.nqa,1);
for i=MBS_data.qa
    MBS_data.inputs{find(MBS_data.qa==i)} = SX.sym(strcat('u_',num2str(i))); 
    MBS_data.theta{find(MBS_data.qa==i)} = SX.sym(strcat('theta_',num2str(i)));
    MBS_data.thetad{find(MBS_data.qa==i)} = SX.sym(strcat('thetad_',num2str(i)));
    MBS_data.thetadd{find(MBS_data.qa==i)} = SX.sym(strcat('thetadd_',num2str(i)));
    MBS_data.el_term{find(MBS_data.qa==i)} = K_el(find(MBS_data.qa==i))*(MBS_data.theta{find(MBS_data.qa==i)} - MBS_data.q(i));
    MBS_data.QqRHS{i} = K_el(find(MBS_data.qa==i))*(MBS_data.theta{find(MBS_data.qa==i)} - MBS_data.q(i));
end
QqRHS = cell2mat_casadi(MBS_data.QqRHS);
el_term = cell2mat_casadi(MBS_data.el_term);

% motor states
theta = cell2mat_casadi(MBS_data.theta);
thetad = cell2mat_casadi(MBS_data.thetad);
thetadd = cell2mat_casadi(MBS_data.thetadd);
inputs = cell2mat_casadi(MBS_data.inputs);

% Motor dynamics
J_mot = 5.55e-7; % kg m^2
red = 205; % reduction ratio
b = 9.18e-7; % Nm/(rad/s)      damping
dynAct = J_mot*red^2*thetadd + damping*thetad + el_term - inputs;

% contact forces
MBS_data.SFor = cell(MBS_data.Nxfrc,1);
for i=1:MBS_data.Nxfrc
    force_i = SX.sym(strcat('f_',MBS_info.allextforces{i,4}), 2,1);
    mom_i = SX.sym(strcat('M_',MBS_info.allextforces{i,4}));
    MBS_data.SFor{i} = vertcat(force_i, mom_i);
end
SFor = cell2mat_casadi(MBS_data.SFor);

% resultant forces and torques
[frc,trq] = mbs_extforces_Planar_Legs_casadi(MBS_data,'void','void');
MBS_data.frc = cell2mat_casadi(frc);
MBS_data.trq = cell2mat_casadi(trq);

% Mass matrix
[M, c] = mbs_dirdyna_Planar_Legs_casadi(MBS_data,'void','void');
MBS_data.M = cell2mat_casadi(M);

Mfun = Function('Mfun',{q},{MBS_data.M});

% invdyn
[QqLHS] = mbs_invdyna_Planar_Legs_casadi(MBS_data,'void','void');
QqLHS = cell2mat_casadi(QqLHS);

% Lagrange mechanics
dynSys = QqLHS - QqRHS + damping_L*qd;
dynRes = vertcat(dynSys,dynAct);
dynFun = Function('dynFun',{q, qd, qdd, SFor, theta, thetad, thetadd, inputs},{dynRes});

% N = 30; % number of control intervals
% N_trans = 9*N/10; % discontinuity in the dynamics

% sensor of the contact point
sens_id_left = 5;
sens_left = mbs_sensor_Planar_Legs_casadi(MBS_data,'void','void',sens_id_left);
c_0 = cell2mat_casadi(sens_left.P);
c0fun = Function('c0fun', {q}, {c_0});
cd_0 = cell2mat_casadi(sens_left.V);
cdd_0 = cell2mat_casadi(sens_left.A);
Jc_0 = cell2mat_casadi(sens_left.J);

alpha_0 = q(3) + q(4) + q(5) + q(6);  % from the kinematic chain
alphad_0 = cell2mat_casadi(sens_left.OM(1));
alphadd_0 = cell2mat_casadi(sens_left.OMP(1));

sens_id_right = 6;
sens_right = mbs_sensor_Planar_Legs_casadi(MBS_data,'void','void',sens_id_right);
c_1 = cell2mat_casadi(sens_right.P);
c1fun = Function('c1fun', {q}, {c_1});
dist = SX.sym('dist');
c_1_diff = c_1 - vertcat(0,dist,0);
cd_1 = cell2mat_casadi(sens_right.V);
cdd_1 = cell2mat_casadi(sens_right.A);
Jc_1 = cell2mat_casadi(sens_right.J);

alpha_1 = q(3) + q(7) + q(8) + q(9);  % from the kinematic chain
alphad_1 = cell2mat_casadi(sens_right.OM(1));
alphadd_1 = cell2mat_casadi(sens_right.OMP(1));

sens_id_toe = mbs_get_S_sensor_id(MBS_info, 'Sensor_RightToe');
sens_toe = mbs_sensor_Planar_Legs_casadi(MBS_data,'void','void',sens_id_toe);
c_toe = cell2mat_casadi(sens_toe.P);
ctoefun = Function('c1fun', {q}, {c_toe});

sens_id_heel = mbs_get_S_sensor_id(MBS_info, 'Sensor_RightHeel');
sens_heel = mbs_sensor_Planar_Legs_casadi(MBS_data,'void','void',sens_id_heel);
c_heel = cell2mat_casadi(sens_heel.P);
cheelfun = Function('c1fun', {q}, {c_heel});

% just because of 2D model!!!

pC_left_fun = Function('pC_left_fun', {vertcat(q,qd,theta,thetad)}, {c_0(2:3)});
pCd_left_fun = Function('pCd_left_fun', {vertcat(q,qd,theta,thetad)}, {cd_0(2:3)});

alpha_left_fun = Function('alpha_left_fun', {vertcat(q,qd,theta,thetad)}, {alpha_0});
alphad_left_fun = Function('alphad_left_fun', {vertcat(q,qd,theta,thetad)}, {alphad_0});

pC_right_fun = Function('pC_right_fun', {vertcat(q,qd,theta,thetad), dist}, {c_1_diff(2:3)});
pCd_right_fun = Function('pCd_right_fun', {vertcat(q,qd,theta,thetad)}, {cd_1(2:3)});

alpha_right_fun = Function('alpha_right_fun', {vertcat(q,qd,theta,thetad)}, {alpha_1});
alphad_right_fun = Function('alphad_right_fun', {vertcat(q,qd,theta,thetad)}, {alphad_1});

% invariants + index reduction
% left foot
cdd_left_hor = cdd_0(2) + 2*zita*gamma_x*cd_0(2) + gamma_x^2*c_0(2);
cdd_left_vert = cdd_0(3) + 2*zita*gamma_y*cd_0(3) + gamma_y^2*c_0(3);
alphadd_left = alphadd_0 + 2*zita*gamma_alpha*alphad_0 + gamma_alpha^2*alpha_0;
cdd_left_planar = vertcat(cdd_left_hor, cdd_left_vert, alphadd_left);
% right foot
cdd_right_hor = cdd_1(2) + 2*zita*gamma_x*cd_1(2) + gamma_x^2*c_1_diff(2);
cdd_right_vert = cdd_1(3) + 2*zita*gamma_y*cd_1(3) + gamma_y^2*c_1_diff(3);
alphadd_right = alphadd_1 + 2*zita*gamma_alpha*alphad_1 + gamma_alpha^2*alpha_1;
cdd_right_planar = vertcat(cdd_right_hor, cdd_right_vert, alphadd_right);

% collisionFun = Function('collisionFun', {vertcat(q,qd,theta,thetad)}, {vertcat(c_heel(3),alpha_1)});
collisionFun = Function('collisionFun', {vertcat(q,qd,theta,thetad)}, {vertcat(c_toe(3), c_heel(3))});

Jc_r = Jc_1(2:4,:);

Fc_left = SFor(1:2);
M_left = SFor(3);
Fc_right = SFor(4:5);
M_right = SFor(6);

% Model variables
dae_x = struct('q', q, 'qd', qd, 'theta', theta, 'thetad', thetad);
dae_z = struct('qdd', qdd, 'Fc', SFor, 'thetadd', thetadd);

% time scaling coefficients
t_s = SX.sym('t_s');
t_d = SX.sym('t_b');

% Left foot in contact  Stance leg q_456
dae_s = struct;
dae_s.x = casadi_struct2vec(dae_x);
dae_s.z = casadi_struct2vec(dae_z);
dae_s.p = vertcat(K_el, inputs, t_s);
dae_s.ode = casadi_vec(dae_x, 'q', t_s*qd, 'qd', t_s*qdd, 'theta', t_s*thetad, 'thetad', t_s*thetadd);
dae_s.alg = [ dynRes; cdd_left_planar; Fc_right; M_right ];

nx = size(dae_s.x,1);
nu = size(dae_s.p,1);
nz = size(dae_s.z,1);

% Both feet in contact
dae_d = struct;
dae_d.x = casadi_struct2vec(dae_x);
dae_d.z = casadi_struct2vec(dae_z);
dae_d.p = vertcat(K_el, inputs, t_d, dist);
dae_d.ode = casadi_vec(dae_x, 'q', t_d*qd, 'qd', t_d*qdd, 'theta', t_d*thetad, 'thetad', t_d*thetadd);
dae_d.alg = [ dynRes; cdd_left_planar; cdd_right_planar];

% State discontinuity function   - right foot impact (swing foot q_789)
q_p = SX.sym('q_p',   MBS_data.Njoint);
qd_p = SX.sym('qd_p',  MBS_data.Njoint);
theta_p = SX.sym('theta_p',   MBS_data.nqa);
thetad_p = SX.sym('thetad_p',  MBS_data.nqa);
lam = SX.sym('lam',3);
mom_disc = mtimes(Mfun(q), qd_p-qd) - mtimes(Jc_r',lam);
mom_disc = vertcat(q_p-q, mom_disc, theta_p-theta, thetad_p-thetad);
inel_imp = mtimes(Jc_r, qd_p);
discfun = Function('discfun',{vertcat(q,qd,theta,thetad), vertcat(q_p,qd_p,theta_p,thetad_p), lam},{vertcat(mom_disc)});

% Formulate discrete time dynamics
% Degree of interpolating polynomial
d = 2;

% Get collocation points
tau_root = collocation_points(d, 'radau');
tau_root = [0 tau_root];

collfun_s = simpleColl(dae_s, tau_root, T/N);
collfun_d = simpleColl(dae_d, tau_root, T/N);

% Constraint functions
% mu = 0.7;   % friction coefficient
% inequality constraints due to friction
fT_left_LBfun = Function('fT_left_LBfun', {SFor}, {mu*Fc_left(2) + Fc_left(1)});
fT_left_UBfun = Function('fT_left_UBfun', {SFor}, {mu*Fc_left(2) - Fc_left(1)});

fT_right_LBfun = Function('fT_right_LBfun', {SFor}, {mu*Fc_right(2) + Fc_right(1)});
fT_right_UBfun = Function('fT_right_UBfun', {SFor}, {mu*Fc_right(2) - Fc_right(1)});

% inequality constraints on the CoP position
% a_r = 0.05;  % TODO: conservative epsilon
% % a_f = 0.05;
% a_f = 0.06;
Mx_left_LBfun = Function('Mx_left_LBfun', {SFor}, {M_left + Fc_left(2)*a_r});
Mx_left_UBfun = Function('Mx_left_UBfun', {SFor}, {Fc_left(2)*a_f - M_left});

Mx_right_LBfun = Function('Mx_right_LBfun', {SFor}, {M_right + Fc_right(2)*a_r});
Mx_right_UBfun = Function('Mx_right_UBfun', {SFor}, {Fc_right(2)*a_f - M_right});

Mx_double_LBfun = Function('Mx_double_LBfun', {SFor}, {M_left + M_right + Fc_right(2)*walk_dist + (Fc_left(2) + Fc_right(2))*a_r});
Mx_double_UBfun = Function('Mx_double_UBfun', {SFor}, {(Fc_left(2) + Fc_right(2))*(a_f+walk_dist) - M_left - M_right - Fc_right(2)*walk_dist});

%Periodicity Constraint
q_rev = vertcat(q(1), q(2), q(3), q(7), q(8), q(9), q(4), q(5), q(6));
qd_rev = vertcat(qd(1), qd(2), qd(3), qd(7), qd(8), qd(9), qd(4), qd(5), qd(6));
theta_rev = vertcat(theta(4), theta(5), theta(6), theta(1), theta(2), theta(3));
thetad_rev = vertcat(thetad(4), thetad(5), thetad(6), thetad(1), thetad(2), thetad(3));
rev_fun = Function('rev_fun', {vertcat(q,qd,theta,thetad)},{vertcat(q_rev,qd_rev,theta_rev,thetad_rev)});
Fc_rev = vertcat(SFor(4), SFor(5), SFor(6),SFor(1), SFor(2), SFor(3));
Fc_rev_fun = Function('Fc_rev_fun',{SFor},{Fc_rev});
Tauc_rev = vertcat(inputs(4), inputs(5), inputs(6),inputs(1), inputs(2), inputs(3));
Tauc_rev_fun = Function('Tauc_rev_fun',{inputs},{Tauc_rev});

% Start with an empty NLP
w = {};

Xs = {};
for i=1:N+1
   Xs{i} = MX.sym(['X_' num2str(i)],nx);
end
XCs = {};
Zs = {};
Us = {};
for i=1:N
   XCs{i} = MX.sym(['XC_' num2str(i)],nx,d);
   Zs{i}  = MX.sym(['Z_' num2str(i)],nz,d);
   Us{i}  = MX.sym(['U_' num2str(i)],nu);
end

Imp = MX.sym('lam', 3);

w_block = struct();
w_block.X  = Sparsity.dense(nx,1);
w_block.XC = Sparsity.dense(nx,d);
w_block.Z  = Sparsity.dense(nz,d);
w_block.U  = Sparsity.dense(nu,1);

w_block_t = struct();
w_block_t.X  = Sparsity.dense(nx,1);
w_block_t.XC = Sparsity.dense(nx,d);
w_block_t.Z  = Sparsity.dense(nz,d);
w_block_t.U  = Sparsity.dense(nu,1);
w_block_t.Imp = Sparsity.dense(3,1);

% Bounds on decision variables
lbw = {};
ubw = {};

q_lb = [-inf; 0.18; -inf; -pi/4; -pi/2; -pi/4; -pi/4; -pi/2; -pi/4 ];
q_ub = [ inf;  inf;  inf;  pi/2;     0;  pi/4;  pi/2;     0;  pi/4 ];

% Speed Limit Motor
thetad_lb = [ones(6,1)*min_speed]; 
thetad_ub = [ones(6,1)*max_speed]; 

x_lb = casadi_vec(dae_x,-inf,'q',q_lb,'thetad',thetad_lb);
x_ub = casadi_vec(dae_x, inf,'q',q_ub,'thetad',thetad_ub);

ts_min = 0.1;
ts_max = 2;

% Torque Limit Single Phase
u_lb = [ K_el_min*ones(6,1); min_torque*ones(6,1); ts_min];
u_ub = [ K_el_max*ones(6,1); max_torque*ones(6,1); ts_max];

Z_lb = -inf(nz,2);
fz_lb = 0;
Z_lb(q.size1()+2, 1)= fz_lb;
Z_lb(q.size1()+5, 1)= fz_lb;
Z_lb(q.size1()+2, d)= fz_lb;
Z_lb(q.size1()+5, d)= fz_lb;

Imp_lb = [-inf; 0; -inf];

% initial guess
w0 = {};

% List of constraints
g = {};

% Bounds on constraints
lbg = {};
ubg = {};

% Objective function
J = 0;

W_tau = DM.eye(MBS_data.nqa);
tausq = t_s*T/N*mtimes( inputs', mtimes( W_tau, inputs ) );
tausqFun = Function('tausqFun', {vertcat(inputs,t_s)}, {tausq});

% initial guess
r_body_low = 0.05;
leg = 0.3 + 0.074;
ql_guess = asin(walk_dist/(2*leg));
q2_guess = pi/6;

vert = r_body_low + leg; 
hor = leg*sin(ql_guess);

ts_guess = 1.;
K_guess = [K_el_guess*ones(6,1)];
u_guess = [0; 0; 0; 0; 0; 0; ts_guess];
u_guess = vertcat(K_guess, u_guess);

% Formulate the NLP
for k=1:N
    % initial guess
    if k < N_trans
        q1l_guess = -ql_guess + k/N_trans*(2*ql_guess);
        q1r_guess = ql_guess - k/(N_trans)*(2*ql_guess);
        hor_guess = -hor + k/N_trans*(2*hor);
        vert_guess = vert;
    else
        q1r_guess = -ql_guess;
        q1l_guess = ql_guess;
        hor_guess = hor;
        vert_guess = vert;
    end
        
    q_guess = [hor_guess; vert_guess; 0; q1r_guess; 0; -q1r_guess; q1l_guess; 0; -q1l_guess];
    theta_guess = [q1r_guess; 0; -q1r_guess; q1l_guess; 0; -q1l_guess];
    x_guess = casadi_vec(dae_x,zeros,'q',q_guess,'theta',theta_guess);
    
        % Add NLP variables
    if k < N_trans
        w = {w{:} casadi_vec(w_block,'X',Xs{k},'XC',XCs{k},'Z',Zs{k},'U',Us{k})};
        
        % add initial guess
        w0 = {w0{:} casadi_vec(w_block,zeros,'X',x_guess,'U',u_guess)};
        
        % add bounds on variables
        lbw = {lbw{:} casadi_vec(w_block,-inf,'X',x_lb,'U',u_lb,'Z',Z_lb)};
        ubw = {ubw{:} casadi_vec(w_block, inf,'X',x_ub,'U',u_ub)};
    elseif k==N_trans
        w = {w{:} casadi_vec(w_block_t,'X',Xs{k},'XC',XCs{k},'Z',Zs{k},'U',Us{k},'Imp',Imp)};
        
        % add initial guess
        w0 = {w0{:} casadi_vec(w_block_t,zeros,'X',x_guess,'U',u_guess)};
        
        % add bounds on variables
        lbw = {lbw{:} casadi_vec(w_block_t,-inf,'X',x_lb,'U',u_lb,'Z',Z_lb,'Imp',Imp_lb)};
        ubw = {ubw{:} casadi_vec(w_block_t, inf,'X',x_ub,'U',u_ub)};
    else
        w = {w{:} casadi_vec(w_block,'X',Xs{k},'XC',XCs{k},'Z',Zs{k},'U',Us{k})};
        
        % add initial guess
        w0 = {w0{:} casadi_vec(w_block,zeros,'X',x_guess,'U',u_guess)};
        
        % add bounds on variables
        lbw = {lbw{:} casadi_vec(w_block,-inf,'X',x_lb,'U',u_lb,'Z',Z_lb)};
        ubw = {ubw{:} casadi_vec(w_block, inf,'X',x_ub,'U',u_ub)};
    end
    
    if k==1
        % initial consistency conditions
        XC_init = XCs{k}(:, 1);
        
        pC_left_0 = pC_left_fun( XC_init );
        pCd_left_0 = pCd_left_fun( XC_init );
        alphad_left_0 = alphad_left_fun( XC_init );
        
        g = {g{:} pC_left_0(1)};     % horizontal position of the left foot
        lbg = {lbg{:} 0};
        ubg = {ubg{:} 0};
        
        g = {g{:} pCd_left_0};       % left foot velocity
        lbg = {lbg{:} zeros(pCd_left_0.size1(),1)};
        ubg = {ubg{:} zeros(pCd_left_0.size1(),1)};
        
        g = {g{:} alphad_left_0};    % angular velocity left foot
        lbg = {lbg{:} 0};
        ubg = {ubg{:} 0};
        
        % time scaling parameter
        dt_constr = Us{k}(end) - Us{k+1}(end);
        g = {g{:} dt_constr};
        lbg = {lbg{:} 0};
        ubg = {ubg{:} 0};
        
        % stiffness
        Usplit = vertsplit(Us{k},[0,MBS_data.nqa,nu]);
        K_k = Usplit{1};
        Usplit = vertsplit(Us{k+1},[0,MBS_data.nqa,nu]);
        K_kp = Usplit{1};
        K_constr = K_k - K_kp;
        g = {g{:} K_constr};
        lbg = {lbg{:} zeros(MBS_data.nqa,1)};
        ubg = {ubg{:} zeros(MBS_data.nqa,1)};
        
    elseif k==N_trans
        % transition consistency conditions
        XC_trans = XCs{k+1}(:, 1);
        
        pC_right_trans = pC_right_fun( XC_trans, walk_dist );
        alpha_right_trans = alpha_right_fun( XC_trans );
        
        pCd_left_trans = pCd_left_fun( XC_trans );
        alphad_left_trans = alphad_left_fun( XC_trans );
        
        g = {g{:} pC_right_trans};      % position right foot
        lbg = {lbg{:} zeros(pC_right_trans.size1(),1)};
        ubg = {ubg{:} zeros(pC_right_trans.size1(),1)};
        
        g = {g{:} alpha_right_trans};   % orientation right foot
        lbg = {lbg{:} 0};
        ubg = {ubg{:} 0};
               
        g = {g{:} pCd_left_trans};      % velocity left foot
        lbg = {lbg{:} zeros(pCd_left_trans.size1(),1)};
        ubg = {ubg{:} zeros(pCd_left_trans.size1(),1)};
        
        g = {g{:} alphad_left_trans};   % angular velocity left foot
        lbg = {lbg{:} 0};
        ubg = {ubg{:} 0};
        
        % time scaling parameter
        dt_constr = Us{k}(end)*N_trans + Us{k+1}(end)*(N - N_trans) - N;
        g = {g{:} dt_constr};
        lbg = {lbg{:} 0};
        ubg = {ubg{:} 0};
        
        % stiffness
        Usplit = vertsplit(Us{k},[0,MBS_data.nqa,nu]);
        K_k = Usplit{1};
        Usplit = vertsplit(Us{k+1},[0,MBS_data.nqa,nu]);
        K_kp = Usplit{1};
        K_constr = K_k - K_kp;
        g = {g{:} K_constr};
        lbg = {lbg{:} zeros(MBS_data.nqa,1)};
        ubg = {ubg{:} zeros(MBS_data.nqa,1)};
    else
        
        if k~=N
            % time scaling parameter
            dt_constr = Us{k}(end) - Us{k+1}(end);
            g = {g{:} dt_constr};
            lbg = {lbg{:} 0};
            ubg = {ubg{:} 0};
            % stiffness
            Usplit = vertsplit(Us{k},[0,MBS_data.nqa,nu]);
            K_k = Usplit{1};
            Usplit = vertsplit(Us{k+1},[0,MBS_data.nqa,nu]);
            K_kp = Usplit{1};
            K_constr = K_k - K_kp;
            g = {g{:} K_constr};
            lbg = {lbg{:} zeros(MBS_data.nqa,1)};
            ubg = {ubg{:} zeros(MBS_data.nqa,1)};
        end
    end
    
    
    if k < N_trans
        [quad, coll_vect] = collfun_s(Xs{k},XCs{k},Zs{k},Us{k});
        
        g = {g{:} Xs{k+1}-quad}; % gap closing
        lbg = {lbg{:} zeros(nx,1)};
        ubg = {ubg{:} zeros(nx,1)};
        
        % no right foot collision
        Xkp = Xs{k+1};
        g_z = collisionFun(Xkp);

        if k==1      
        elseif (k <= n_step_h + 1) 
            g = {g{:} g_z};
            
            coeff = (k-1)/n_step_h;
            lbg = {lbg{:} vertcat(h_foot*coeff, h_foot*coeff)}; %zeros(g_z.size1(),1)};
            ubg = {ubg{:} inf(g_z.size1(),1)};
        elseif (k >= (N_trans - n_step_h))
            g = {g{:} g_z};
            coeff = 1 - (k - (N_trans - n_step_h))/n_step_h;
            lbg = {lbg{:} vertcat(h_foot*coeff, h_foot*coeff)}; %zeros(g_z.size1(),1)};
            ubg = {ubg{:} inf(g_z.size1(),1)};
        else
            g = {g{:} g_z};
            lbg = {lbg{:} vertcat(h_foot, h_foot)}; %zeros(g_z.size1(),1)}; 
            ubg = {ubg{:} inf(g_z.size1(),1)};
        end      
        
        Zsplit = vertsplit(Zs{k},[0,qdd.size1(),nz]);
        Zsplit = Zsplit{2};
        Zsplit_1 = vertsplit(Zsplit,[0,SFor.size1(),Zsplit.size1()]);
        Fc_k = Zsplit_1{1};
        Fc_end = Fc_k(:, d);
        % friction
        g_ft = vertcat(fT_left_LBfun(Fc_end), fT_left_UBfun(Fc_end));
        g = {g{:} g_ft};
        
        lbg = {lbg{:} zeros(2,1)};
        ubg = {ubg{:} inf(2,1)};
        
        % CoP
        if flag_COP == 1
            
            g_CoP = vertcat(Mx_left_LBfun(Fc_end), Mx_left_UBfun(Fc_end));
            g = {g{:} g_CoP};

            lbg = {lbg{:} zeros(2,1)};
            ubg = {ubg{:} inf(2,1)};
        end
        
    elseif k==N_trans
        [quad, coll_vect] = collfun_s(Xs{k},XCs{k},Zs{k},Us{k});
        
        g = {g{:} discfun(quad,Xs{k+1},Imp)}; % jump
        lbg = {lbg{:} zeros(nx,1)};
        ubg = {ubg{:} zeros(nx,1)};
        
        Zsplit = vertsplit(Zs{k},[0,qdd.size1(),nz]);
        Zsplit = Zsplit{2};
        Zsplit_1 = vertsplit(Zsplit,[0,SFor.size1(),Zsplit.size1()]);
        Fc_k = Zsplit_1{1};
        Fc_end = Fc_k(:, d);
        
        % friction
        g_ft = vertcat(fT_left_LBfun(Fc_end), fT_left_UBfun(Fc_end), fT_right_LBfun(vertcat(0, 0, 0, Imp)), fT_right_UBfun(vertcat(0, 0, 0, Imp)));
        g = {g{:} g_ft};
        
        lbg = {lbg{:} zeros(4,1)};
        ubg = {ubg{:} inf(4,1)};
        
%         % CoP
%         if flag_COP == 1
%             g_CoP = vertcat(Mx_left_LBfun(Fc_end), Mx_left_UBfun(Fc_end), Mx_right_LBfun(vertcat(0, 0, 0, Imp)), Mx_right_UBfun(vertcat(0, 0, 0, Imp)));
%             g = {g{:} g_CoP};
% 
%             lbg = {lbg{:} zeros(4,1)};
%             ubg = {ubg{:} inf(4,1)};
%         end
        
    else
        [quad, coll_vect] = collfun_d(Xs{k},XCs{k},Zs{k},vertcat(Us{k},walk_dist));
        
        g = {g{:} Xs{k+1}-quad}; % gap closing
        lbg = {lbg{:} zeros(nx,1)};
        ubg = {ubg{:} zeros(nx,1)};
        
        Zsplit = vertsplit(Zs{k},[0,qdd.size1(),nz]);
        Zsplit = Zsplit{2};
        Zsplit_1 = vertsplit(Zsplit,[0,SFor.size1(),Zsplit.size1()]);
        Fc_k = Zsplit_1{1};
        Fc_end = Fc_k(:, d);
        
        % friction
        g_ft = vertcat(fT_left_LBfun(Fc_end), fT_left_UBfun(Fc_end), fT_right_LBfun(Fc_end), fT_right_UBfun(Fc_end));
        g = {g{:} g_ft};
        
        lbg = {lbg{:} zeros(4,1)};
        ubg = {ubg{:} inf(4,1)};
        
        % CoP
        if flag_COP == 1
            g_CoP = vertcat(Mx_double_LBfun(Fc_end), Mx_double_UBfun(Fc_end));
            g = {g{:} g_CoP};

            lbg = {lbg{:} zeros(2,1)};
            ubg = {ubg{:} inf(2,1)};
        end
    end
    
    g = {g{:} coll_vect};         % collocation constraints
    lbg = {lbg{:} zeros((nx+nz)*d,1)};
    ubg = {ubg{:} zeros((nx+nz)*d,1)};
    
    Usplit = vertsplit(Us{k},[0,MBS_data.nqa,nu]);
    U_k = Usplit{2};
    J = J + tausqFun(U_k);
    
end

w = {w{:} Xs{end}};
w0 = {w0{:} x_guess};

lbw = {lbw{:} x_lb};
ubw = {ubw{:} x_ub};

% Periodicity constraint on states
X_red_init = Xs{1}(2:nx);
X_end = rev_fun(Xs{end});
X_red_end = X_end(2:nx);

period = X_red_init - X_red_end;

g = {g{:} period};
lbg = {lbg{:} zeros(period.size1(),1)};
ubg = {ubg{:} zeros(period.size1(),1)};

% Periodicity constraint on contact forces
Z0split = vertsplit(Zs{1},[0,qdd.size1(),nz]);
Z0split = Z0split{2};
Z0split_1 = vertsplit(Z0split,[0,SFor.size1(),Z0split.size1()]);
Fc_0 = Z0split_1{1};
Fc_0 = Fc_0(:, 1);

ZNsplit = vertsplit(Zs{end},[0,qdd.size1(),nz]);
ZNsplit = ZNsplit{2};
ZNsplit_1 = vertsplit(ZNsplit,[0,SFor.size1(),ZNsplit.size1()]);
Fc_end = ZNsplit_1{1};
Fc_end = Fc_end(:, d);

Fc_end = Fc_rev_fun(Fc_end);

period_fc = Fc_0 - Fc_end;

g = {g{:} period_fc};
lbg = {lbg{:} -fc_per_tol*ones(period_fc.size1(),1)};
ubg = {ubg{:}  fc_per_tol*ones(period_fc.size1(),1)};

% Periodicity constraints on torques
Tauc_end = Tauc_rev_fun(Us{end}(MBS_data.nqa+1:end-1));
period_tauc = Us{1}(MBS_data.nqa+1:end-1) - Tauc_end;
                                         
g = {g{:} period_tauc};
lbg = {lbg{:} -tau_per_tol*ones(period_tauc.size1(),1)};
ubg = {ubg{:}  tau_per_tol*ones(period_tauc.size1(),1)};

% COT
J = J/(m_robot*grav_acc*walk_dist);

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
options = struct;
options.ipopt.max_iter = maxiter;
options.ipopt.tol = tol;
if flag_dual
options.ipopt.dual_inf_tol=dual_tol;
end
solver = nlpsol('solver', 'ipopt', prob, options);

% Solve the NLP
sol = solver('x0',vertcat(w0{:}),'lbx', vertcat(lbw{:}), 'ubx', vertcat(ubw{:}), 'lbg', vertcat(lbg{:}), 'ubg', vertcat(ubg{:}));
display(sol.f)
w_opt = full(sol.x);
