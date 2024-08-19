%% Clear Section
clc;
clearvars;
close all;
%% Read Excel datasheet of Aircraft type and Flight Condition

filename_density_L = 'B_747_FC_5'; 

aircraft_data=xlsread(filename_density_L,'B2:B61');

%% Time vector parameters
dt = aircraft_data(1);  
tfinal = aircraft_data(2); 

%% Initial conditions
states_0 = aircraft_data(4:15);
states_dot_0 = zeros(12,1);
Vtotal_0 = sqrt(states_0(1)^2 + states_0(2)^2 + states_0(3)^2);    

S_0.x = states_0(10);
S_0.y = states_0(11);
S_0.z = states_0(12);
S_0.phi = states_0(7);
S_0.theta = states_0(8);
S_0.psi = states_0(9);
S_0.u = states_0(1);
S_0.v = states_0(2);
S_0.w = states_0(3);
S_0.p = states_0(4);
S_0.q = states_0(5);
S_0.r = states_0(6);
S_0.alpha = aircraft_data(18);
S_0.beta = aircraft_data(20);
S_0.V_total = Vtotal_0;
S_0.wdot = 0;

%% Control actions values
% da = aircraft_data(57);
% dr = aircraft_data(58);
% de = aircraft_data(59);
% dth = aircraft_data(60);
Delta_vector = [ aircraft_data(57:59)  ; aircraft_data(60)];

%% Gravity, Mass & Inertia
m = aircraft_data(51);
g = aircraft_data(52);
Ixx = aircraft_data(53);
Iyy = aircraft_data(54);
Izz = aircraft_data(55);
Ixz = aircraft_data(56);    Ixy=0;  Iyz=0;
I = [Ixx , -Ixy , -Ixz ;...
    -Ixy , Iyy , -Iyz ;...
    -Ixz , -Iyz , Izz];

%% Stability derivatives Longitudinal motion

SD_Long = aircraft_data(21:36);
SD_Long_final = SD_Long;

tempvarLong = num2cell(SD_Long_final);

[Xu,Zu,Mu,Xw,Zw,Mw,Zwd,Zq,Mwd,Mq,Xde,Zde,Mde,Xdth,Zdth,Mdth] = deal(tempvarLong{:});
clear tempvarLong;

%% Stability derivatives Lateral motion

SD_Lat_dash = aircraft_data(37:50);

Lb_dash =  SD_Lat_dash(3);
Nb_dash =  SD_Lat_dash(4);
Lp_dash =  SD_Lat_dash(5);
Np_dash =  SD_Lat_dash(6);
Lr_dash =  SD_Lat_dash(7);
Nr_dash =  SD_Lat_dash(8);
Yda_star = SD_Lat_dash(9);
Ydr_star = SD_Lat_dash(10);
Lda_dash = SD_Lat_dash(11);
Nda_dash = SD_Lat_dash(12);
Ldr_dash = SD_Lat_dash(13);
Ndr_dash = SD_Lat_dash(14);

SD_Lat_final = Lateral_correction(SD_Lat_dash, Vtotal_0, Ixx, Izz, Ixz);

tempvarLat = num2cell(SD_Lat_final);    

[Yv,Yb,Lb,Nb,Lp,Np,Lr,Nr,Yda,Ydr,Lda,Nda,Ldr,Ndr] = deal(tempvarLat{:});
clear tempvarLat;

Lv = Lb/Vtotal_0;
Nv = Nb/Vtotal_0;
Yp=0;
Yr=0;

%% Initial gravity force

Fg_0 = m * g * [ sin(states_0(8)) ; -cos(states_0(8))*sin(states_0(7)) ; -cos(states_0(8))*cos(states_0(7)) ];

%% Matrices

States_Matrix = [Xu  0  Xw  0  0  0  0;...
                 0   Yv 0   Yp 0  Yr 0;...
                 Zu  0  Zw  0  Zq 0  Zwd;...
                 0   Lv 0   Lp 0  Lr 0;...
                 Mu  0  Mw  0  Mq 0  Mwd;...
                 0   Nv 0   Np 0  Nr 0];

Control_Matrix = [0   Xde Xdth 0;...
                  0   0   0    Ydr;...
                  0   Zde Zdth 0;...
                  Lda 0   0    Ldr;...
                  0   Mde Mdth 0;...
                  Nda 0   0    Ndr];
 
%% longitudinal
%% define A matrix  
A_full_long=[Xu Xw -S_0.w -g*cos(S_0.theta);Zu/(1-Zwd) Zw/(1-Zwd) (Zq+S_0.u)/(1-Zwd) -g*sin(S_0.theta)/(1-Zwd);...
    Mu+Mwd*Zu/(1-Zwd) Mw+Mwd*Zw/(1-Zwd) Mq+Mwd*(Zq+S_0.u)/(1-Zwd) -Mwd*g*sin(S_0.theta)/(1-Zwd);...
    0 0 1 0];%full linearized longitudinal model

%% define B matrix
B_full_long=[Xde Xdth;Zde/(1-Zwd) Zdth/(1-Zwd);Mde+Zde*Mwd/(1-Zwd) Mdth+Zdth*Mwd/(1-Zwd);0 0];%full linearized longitudinal model

%% define C matrix
C_full_long=eye(4);%full linearized longitudinal model

%% define state space for each model
full_linarized_model=ss(A_full_long,B_full_long,C_full_long,0,'StateName',{'u' 'w' 'q' 'theta'},'InputName',{'delta_elev' 'delta_thrust'});

%% define the transfer functions of each system
tf_full=tf(full_linarized_model);

%% useful_TFs
servo = tf(10,[1 10]);
Servo = tf(10,[1 10]);
Engine_Time_lag = tf(0.1,[1 0.1]);
Differentiator = tf([1 0],1);
Integrator = tf(1,[1 0]);

%% longitudinal Autopilot
long_ss=ss(A_full_long,B_full_long,C_full_long,0);
long_tf=tf(long_ss);

%% lateral              
%% define A matrix

A_full_long = [Yv (Yp+S_0.w)/Vtotal_0 (Yr-S_0.u)/Vtotal_0 g*cos(S_0.theta)/Vtotal_0 0;...
          Lb_dash Lp_dash Lr_dash 0 0;...
          Nb_dash Np_dash Nr_dash 0 0;...
          0 1 tan(S_0.theta) 0 0;...
          0 0 1/cos(S_0.theta) 0 0];

%% define B matrix

B_full = [Yda_star Ydr_star;Lda_dash Ldr_dash;Nda_dash Ndr_dash;0 0 ;0 0];

%% define C matrix

C_full_long = eye(5);

%% define state space for each model

Lat_ss = ss(A_full_long,B_full,C_full_long,0,'StateName',{'beta' 'p' 'r' 'phi' 'psi'},'InputName',{'d_a' 'd_r'});

%% define the transfer functions of each system

Lat_tf = tf(Lat_ss);

%% define transfer functions for full linearized lateral model
beta_da = Lat_tf(1,1);
p_da = Lat_tf(2,1);
r_da = Lat_tf(3,1);
phi_da = Lat_tf(4,1);
psi_da = Lat_tf(5,1);

beta_dr = Lat_tf(1,2);
p_dr = Lat_tf(2,2);
r_dr = Lat_tf(3,2);
phi_dr = Lat_tf(4,2);
psi_dr = Lat_tf(5,2);

%% Lateral Autopilot

OL_r_rcom = servo*r_dr;

load('HPF_YawDamper.mat');

Lat_YawDamped_ss = feedback(servo*Lat_ss,HPF_YawDamper,2,3,1);
Lat_YawDamped_tf = tf(Lat_YawDamped_ss);
phi_da_YawDamped = Lat_YawDamped_tf(4,1);

OL_phi_phicom = minreal(phi_da_YawDamped);
