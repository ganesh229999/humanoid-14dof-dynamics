% CASE5_LIFT_UPWARD  Dual-arm coordinated upward lift (0.205 m in 5 s).
%
% Scenario: Both arms grasp an object at the initial pose and lift it
% 0.205 m upward (along the robot's X-axis) over 5 seconds at constant
% Cartesian velocity (0.041 m/s). Joint trajectories are generated via
% DLS inverse kinematics in trajectoryDualArmNew1610.
%
% OUTPUT: Saves trajectories to 'Testoctlifting.xlsx' (7 sheets).
%         Run main_rne_fixedbase.m next to compute torques.
%
% ── Task Parameters ───────────────────────────────────────────────────────
%   Velocity:  vx = 0.041 m/s  (all other components = 0)
%   Duration:  5 s  |  dt = 0.01 s  |  500 timesteps
%   Object initial position: [-0.23; 0.4885; 0] m
%   Arm A initial config: pre-grasp pose (DRDO calibrated)
%   Arm B initial config: symmetric mirror of Arm A

clc; clear;

%% Desired end-effector velocity and acceleration
Vc_des       = [0.041; 0; 0; 0; 0; 0];   % [m/s; m/s; m/s; rad/s; rad/s; rad/s]
Vc_dot_d_des = zeros(6,1);               % Constant velocity → zero acceleration

%% Object parameters
objectParams.Pc0 = [-0.23; 0.4885; 0];   % Initial object position [m]

%% Initial joint configurations [rad]  (DRDO calibrated pre-grasp pose)
q_A_init = [-0.4652;  0.0081;  0.0934; -0.2084; -0.0972; -0.8611;  0.0153];
q_B_init = [-0.4669; -0.0148; -0.1741; -0.2084;  0.1919; -0.8626; -0.0284];

%% Simulation parameters
t_start  = 0;
t_end    = 5;
dt       = 0.01;
filename = 'Testoctlifting.xlsx';

%% Compute trajectories
fprintf('Computing trajectories for Case 5 (upward lift)...\n');
[q_A_history, q_A_dot_history, q_A_ddot_history, ...
 q_B_history, q_B_dot_history, q_B_ddot_history, ...
 Pc_history, Vc_history, x_dot_d_history, elapsed_time] = ...
    trajectoryDualArmNew1610(Vc_des, Vc_dot_d_des, objectParams, ...
                              q_A_init, q_B_init, t_start, t_end, dt, filename);

fprintf('Done. Elapsed: %.4f s\n', elapsed_time);
fprintf('Trajectory saved to: %s\n', filename);
fprintf('Next step: run main_rne_fixedbase.m to compute joint torques.\n');
