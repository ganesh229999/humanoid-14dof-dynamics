function [q_A_history, q_A_dot_history, q_A_ddot_history, ...
          q_B_history, q_B_dot_history, q_B_ddot_history, ...
          Pc_history, Vc_history, x_dot_d_history, elapsed_time] = ...
    trajectoryDualArmNew1610(Vc_des, Vc_dot_d_des, objectParams, ...
                              q_A_init, q_B_init, t_start, t_end, dt, FileName)
% TRAJECTORYDUALARM  Joint trajectory generation for a fixed-base dual-arm robot.
%
% Generates smooth joint trajectories for both arms to achieve a desired
% Cartesian end-effector velocity using damped least-squares (DLS) inverse
% kinematics at each timestep (Euler integration).
%
%   q̇ = J†_dls · Jo · Vc_d      (DLS: J†_dls = (JᵀJ + λ²I)⁻¹Jᵀ)
%
% ═══════════════════════════════════════════════════════════════════════════
% INPUTS:
%   Vc_des       – 6×1  desired EE velocity    [vx;vy;vz;ωx;ωy;ωz]  [m/s, rad/s]
%   Vc_dot_d_des – 6×1  desired EE acceleration [m/s², rad/s²]
%   objectParams – struct: .Pc0 (3×1 initial object position)
%   q_A_init     – 7×1  initial joint angles, Arm A [rad]
%   q_B_init     – 7×1  initial joint angles, Arm B [rad]
%   t_start      – scalar start time [s]
%   t_end        – scalar end   time [s]
%   dt           – scalar timestep   [s]
%   FileName     – char  output Excel file path
%
% OUTPUTS:
%   q_A_history  – 7×(N+1)  Arm A joint angle trajectory
%   q_B_history  – 7×(N+1)  Arm B joint angle trajectory
%   q_A_dot_history  – 7×N  Arm A joint velocity trajectory
%   q_B_dot_history  – 7×N  Arm B joint velocity trajectory
%   q_A_ddot_history – 7×N  Arm A joint acceleration trajectory
%   q_B_ddot_history – 7×N  Arm B joint acceleration trajectory
%   Pc_history   – 3×(N+1) object position history
%   Vc_history   – 6×(N+1) desired EE velocity history
%   x_dot_d_history – 1×(N+1) x-component velocity history
%   elapsed_time – scalar wall-clock simulation time [s]
%
% NOTE: Damping factor λ=1e-4 avoids singularity blow-up; increase for
%       more regularisation near degenerate configurations.
% ═══════════════════════════════════════════════════════════════════════════

% Object Jacobian (small offset skew)
rx=0.0001; ry=0.0001; rz=0.0001;
Jo = [eye(3), [0,-rz,ry;rz,0,-rx;-ry,rx,0]; zeros(3),eye(3)];

n_steps = floor((t_end-t_start)/dt);
Pc = objectParams.Pc0;

% Pre-allocate
q_A_history  = zeros(7,n_steps+1); q_B_history  = zeros(7,n_steps+1);
q_A_dot_history  = zeros(7,n_steps);  q_B_dot_history  = zeros(7,n_steps);
q_A_ddot_history = zeros(7,n_steps);  q_B_ddot_history = zeros(7,n_steps);
Pc_history   = zeros(3,n_steps+1);
Vc_history   = zeros(6,n_steps+1);
x_dot_d_history = zeros(1,n_steps+1);

q_A_history(:,1) = q_A_init;
q_B_history(:,1) = q_B_init;
Pc_history(:,1)  = Pc;
lambda = 1e-4;   % DLS damping factor

tic;
for i = 2:n_steps+1
    Vc_d    = Vc_des;
    Vc_dot_d = Vc_dot_d_des;

    q_A = q_A_history(:,i-1);
    q_B = q_B_history(:,i-1);

    J_A = jacobianOfArmA(q_A);   % 6×7
    J_B = jacobianOfArmB(q_B);   % 6×7

    % DLS pseudo-inverse
    Jp_A = (J_A'*J_A + lambda^2*eye(7)) \ J_A';
    Jp_B = (J_B'*J_B + lambda^2*eye(7)) \ J_B';

    q_A_dot  = Jp_A * Jo * Vc_d;
    q_B_dot  = Jp_B * Jo * Vc_d;
    q_A_ddot = Jp_A * Jo * Vc_dot_d;
    q_B_ddot = Jp_B * Jo * Vc_dot_d;

    q_A_history(:,i)     = q_A + q_A_dot*dt;
    q_B_history(:,i)     = q_B + q_B_dot*dt;
    q_A_dot_history(:,i-1)  = q_A_dot;
    q_B_dot_history(:,i-1)  = q_B_dot;
    q_A_ddot_history(:,i-1) = q_A_ddot;
    q_B_ddot_history(:,i-1) = q_B_ddot;

    Pc_history(:,i)   = Pc_history(:,i-1) + Vc_d(1:3)*dt;
    Vc_history(:,i)   = Vc_d;
    x_dot_d_history(i) = Vc_d(1);
end
elapsed_time = toc;
fprintf('Trajectory computed in %.4f s  (%d steps)\n', elapsed_time, n_steps);

% Export to Excel
writematrix(q_A_history,  FileName,'Sheet','ArmA_q');
writematrix(q_A_dot_history, FileName,'Sheet','ArmA_qdot');
writematrix(q_A_ddot_history,FileName,'Sheet','ArmA_qddot');
writematrix(q_B_history,  FileName,'Sheet','ArmB_q');
writematrix(q_B_dot_history, FileName,'Sheet','ArmB_qdot');
writematrix(q_B_ddot_history,FileName,'Sheet','ArmB_qddot');
writematrix(Pc_history,   FileName,'Sheet','ObjectPc');
end
