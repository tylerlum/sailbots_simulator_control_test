% Non-Lenear model of Autonomous Sailing Vessel
% UBC Sailbot
% Nicolas Navarre
% Feb 25 2019
%close all;
%clear all;
%% State Equation Setup

% Define constants
m = 1000; %kg
I_xx = 100; %kg m^2
I_zz = 100; %kg m^2
A_s = 10; %m^2
A_r = 1; %m^2
A_h = 1; %m^2
x_m = 0.5; %m x-coord of mast in b-frame
rCoE_x = 0.1; %m
rCoE_y = 0.1; %m
rCoE_z = 0.3; %m
sCoE_x = 1; %m
sCoE_y = 1; %m
sCoE_z = 3; %m
hCoE_x = 1; %m
hCoE_y = 1; %m
hCoE_z = 1; %m
kCoE_x = 1; %m
kCoE_y = 1; %m
kCoE_z = 1; %m
sCoE = [sCoE_x;sCoE_y;sCoE_z];
rCoE = [rCoE_x;rCoE_y;rCoE_z];
hCoE = [hCoE_x;hCoE_y;rCoE_z];
x_sm = norm(sCoE);
heel_const = 1; %testing?
a_right = 1; % Quatratic righting coeficient
b_right = 1; % Linear righting coefficient

rho_w = 997; %kg/m^3
rho_a = 1.225; %kg/m^3
k1 = 1; %lift/drag const (This can be tuned)
waterline_vol = 2; %m^3  This might have to be a function of roll
added_mass = waterline_vol * rho_w /2; %

%State space variables
syms x y psi phi surge sway roll yaw dx dy sangle rangle

%Variable Constants
syms v_tw_b v_tw_n v_aw_b v_ar_b v_ak_b v_ah_n alpha_ah v_ah alpha_ak v_ak alpha_ar v_ar alpha_tw alpha_aw v_tw v_aw lift drag T T_s T_r D_heel D_h D_yaw M_righting A rho alpha v

% Mass Matrix
M_rb = [m 0 0 0;
        0 m 0 0;
        0 0 I_xx 0;
        0 0 0 I_zz];
M_add = diag(added_mass*ones(1,4));
M = M_rb + M_add;
%Motion Matrix
C_rb = [0 -m*yaw 0 0;
        m*yaw 0 0 0;
        0 0 0 0;
        0 0 0 0];
C_add = added_mass*[0 0 0 1;
                    0 0 0 -1;
                    0 0 0 0;
                    -1 1 0 0];
C = C_add + C_rb;

%% Collect Input Data
% run('generateData.m');
% OR (Uncomment desired data option)
%run('setData.m');
%load('inputData.mat');

%% Necessary Motion functions

vss = [surge;sway;roll;yaw];
J = [cos(psi) -sin(psi)*cos(phi) 0 0;
     sin(psi) cos(psi)*cos(phi) 0 0;
     0 0 1 0;
     0 0 0 cos(phi)];
 
%Lift and Drag Equations
lift(rho,A,v,alpha) = 0.5*rho*A*(v^2)*k1*sin(2*alpha);
drag(rho,A,v,alpha) = 0.5*rho*A*(v^2)*k1*(1-cos(2*alpha));

%% Relative Velocities (VECTORS BABY)

%Sail (n-frame), (b-frame)
R1 = [cos(-psi), -sin(-psi), 0;
    sin(-psi), cos(-psi), 0;
    0, 0, 1];
R2 = [1, 0, 0;
    0, cos(-phi), -sin(-phi);
    0, sin(-phi), cos(-phi)];

v_tw_n = [v_tw*cos(alpha_tw); v_tw*sin(alpha_tw); 0];
v_tw_b = R2*R1*v_tw_n;

v_aw_b = [v_tw*cos(alpha_tw-psi)-surge+yaw*sCoE_y;
    v_tw*sin(alpha_tw-psi)*cos(phi)-sway+yaw*sCoE_x+roll*sCoE_z;
    0];
v_aw = norm(v_aw_b);
alpha_aw = atan2(v_aw_b(2),-v_aw_b(1));
alpha_s = alpha_aw - sangle;

%Rudder (b-frame)
v_ar_b = [-surge + yaw*rCoE_y; -sway - yaw*rCoE_x + roll*rCoE_z; 0];
v_ar = norm(v_ar_b);
alpha_ar = atan2(v_ar_b(2),-v_ar_b(1));
alpha_r = alpha_ar - rangle;

%Keel (b-frame)
v_ak_b = [-surge + yaw*kCoE_y; -sway - yaw*kCoE_x + roll*kCoE_z; 0];
v_ak = norm(v_ar_b);
alpha_ak = atan2(v_ak_b(2),-v_ak_b(1));

%Hull (n-frame)
v_ah_n = [-surge + yaw*hCoE_y; sec(phi)*(-sway - yaw*hCoE_x + roll*hCoE_z); 0];
v_ah = norm(v_ah_n);
alpha_ah = atan2(v_ah_n(2),-v_ah_n(1));



%% Force Equations

% Sail Forces
T_s = [lift(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw)-drag(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw);
                                lift(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw)+drag(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw);
                                (lift(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw)+drag(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw))*abs(sCoE_z);
                                -(lift(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw)-drag(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw))*x_sm*sin(sangle) + (lift(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw)+drag(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw))*(x_m-x_sm*cos(sangle))];
% Rudder Forces
T_r = [-drag(rho_w,A_r,v_ah,alpha_r);
                     lift(rho_w,A_r,v_ar,alpha_r);
                     lift(rho_w,A_r,v_ar,alpha_r)*abs(rCoE_z);
                     -lift(rho_w,A_r,v_ar,alpha_r)*abs(rCoE_x)];
                 
T = T_s + T_r;

% Righting Moment
M_righting(phi) = [0;
    0;
    0;
    a_right*phi^2+b_right*phi];

%% Damping Forces
D_h = [drag(rho_w,A_h,v_ah,alpha_ah)*cos(alpha_ah);
                                -drag(rho_w,A_h,v_ah,alpha_ah)*sin(alpha_ah)*cos(phi);
                                (-drag(rho_w,A_h,v_ah,alpha_ah)*sin(alpha_ah)*cos(phi))*abs(hCoE_z);
                                (drag(rho_w,A_h,v_ah,alpha_ah)*sin(alpha_ah)*cos(phi))*abs(hCoE_x)];

D_heel = [0;
             0;
             heel_const*roll*abs(roll);
             0];
D_yaw = [0;
    0;
    0;
    yaw*abs(yaw)*cos(phi)];

D = D_heel + D_h + D_yaw;

%% Non-linear equation
v_dot = M^(-1)*(C*vss - D - M_righting(phi) + T);
n_dot = J*vss;

%State Space and Linearization
ss_eq = [n_dot;v_dot];
ss_vars = [x y phi psi surge sway roll yaw];
ss_in = [sangle, rangle];

n_in = [x_in,y_in,phi_in,psi_in];
v_in = transpose(eval(subs(J,[psi,phi],[psi_in,phi_in])\[dx_in;dy_in;roll_in;yaw_in]));
ang_in = [sangle_in,rangle_in];

linearized_A = jacobian(ss_eq,ss_vars);
linearized_B = jacobian(ss_eq,ss_in);

linearized_A = subs(linearized_A,[ss_vars, ss_in], [n_in, v_in, ang_in]);
linearized_B = subs(linearized_B,[ss_vars, ss_in], [n_in, v_in, ang_in]);

%% TYLER ADDITION
%T = subs(T, [ss_vars, ss_in, v_tw, alpha_tw], [n_in, v_in, ang_in, v_tw_in, alpha_tw_in]);