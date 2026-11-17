%% Rocket TVC + Multi-phase Flight — Initialization

clear; clc;

%% --- Rocket parameters (from OpenRocket) ---
m0       = 0.792;         % Initial mass with motor [kg]
m_dry    = 0.767;         % Dry mass [kg]
t_burn   = 0.99;          % Burn duration [s]
T_avg    = 56;            % Average thrust [N]
I_pitch  = 0.0695;        % Pitch moment of inertia [kg*m^2]
L_gimbal = 0.357;         % Distance CG -> gimbal point [m]
A_ref    = 22.9e-4;       % Body reference area [m^2]
Cd_asc   = 0.75;          % Drag coefficient during ascent
D_para   = 0.70;          % Parachute diameter [m]
A_para   = pi*(D_para/2)^2;
Cd_para  = 0.80;

% Environment
g   = 9.81;
rho = 1.225;

% Plant gain for pitch dynamics (from project 1)
K = T_avg * L_gimbal / I_pitch;

%% --- PID Controller (from project 1, already tuned) ---
Kp = 2.5;
Ki = 0;
Kd = 0.12;
Tf = 0.05;

%% --- Simulation ---
t_sim = 40;               % Total simulation time [s]

fprintf('Rocket plant gain K = %.1f rad/s^2 per rad\n', K);
fprintf('Init done. Ready for Simulink.\n');