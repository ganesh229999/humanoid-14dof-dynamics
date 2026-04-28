% MAIN_RNE_BOTTLE_OPENING  RNE dynamics for dual-arm bottle-opening task.
%
% Scenario: Arm A holds a bottle (radius=0.04 m, height=0.15 m, filled with water)
% while Arm B applies a rotational opening motion to the cap.
% Object inertia computed analytically from solid-cylinder formula.
%
% Prerequisites: Run trajectoryDualArmNew1610 with bottle_opening trajectory
%               to generate 'bottle_opening_data.xlsx' first.

clc; tic;
filename = 'bottle_opening_data.xlsx';

%% Load trajectory
th_A    = readmatrix(filename,'Sheet',1).';
thdot_A = readmatrix(filename,'Sheet',2).';
thddot_A= readmatrix(filename,'Sheet',3).';
th_B    = readmatrix(filename,'Sheet',4).';
thdot_B = readmatrix(filename,'Sheet',5).';
thddot_B= readmatrix(filename,'Sheet',6).';

%% Object: water-filled bottle (solid-cylinder inertia)
rho    = 1000;              % water density [kg/m³]
r      = 0.04;              % bottle radius [m]
h      = 0.15;              % bottle height [m]
m_obj  = rho * pi * r^2 * h;

% Centroidal inertia of solid cylinder
Ixx = (1/12)*m_obj*(3*r^2 + h^2);   % perpendicular to axis
Iyy = Ixx;
Izz = (1/2)*m_obj*r^2;              % along axis

objParams.mass  = m_obj;
objParams.I     = diag([Ixx, Iyy, Izz]);
objParams.g_obj = [-9.81; 0; 0];

fprintf('Object mass: %.4f kg | Ixx=%.6f  Izz=%.6f kg·m²\n', m_obj, Ixx, Izz);

%% RNE
fprintf('Running RNE – Arm A (holding)...\n');
[tau_A, M_A, C_A, G_A] = rnedynamicFixedBase1610(th_A,thdot_A,thddot_A,'A',objParams);
fprintf('Running RNE – Arm B (opening)...\n');
[tau_B, M_B, C_B, G_B] = rnedynamicFixedBase1610(th_B,thdot_B,thddot_B,'B',objParams);

%% Save & plot
save('RNE_BottleOpening.mat','tau_A','M_A','C_A','G_A','tau_B','M_B','C_B','G_B');
exportAndPlotTau(tau_A, 0.01, 'TauA_BottleOpening.xlsx', 'all', 'ArmA_Open');
exportAndPlotTau(tau_B, 0.01, 'TauB_BottleOpening.xlsx', 'all', 'ArmB_Open');
fprintf('Elapsed: %.4f s\n', toc);
