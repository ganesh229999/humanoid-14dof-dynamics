% STARTUP.M  — Run this first to configure MATLAB path for the entire repo.
%
% Usage:
%   >> cd path/to/humanoid-14dof-dynamics
%   >> startup
%
% After running, all functions (kinematics, dynamics, utils, tasks)
% are immediately available from any working directory.

fprintf('\n=========================================================\n');
fprintf('  Humanoid 14-DOF Dual-Arm Dynamics — MATLAB Environment\n');
fprintf('=========================================================\n');

repoRoot = fileparts(mfilename('fullpath'));

% Add all source subdirectories
addpath(fullfile(repoRoot, 'src', 'kinematics'));
addpath(fullfile(repoRoot, 'src', 'dynamics'));
addpath(fullfile(repoRoot, 'src', 'trajectory'));
addpath(fullfile(repoRoot, 'src', 'animation'));
addpath(fullfile(repoRoot, 'src', 'utils'));
addpath(fullfile(repoRoot, 'tasks', 'lifting'));
addpath(fullfile(repoRoot, 'tasks', 'bottle_opening'));

fprintf('✓ Path configured. %d modules loaded.\n\n', 8);
fprintf('Quick-start commands:\n');
fprintf('  run(''tasks/lifting/case5_lift_upward.m'')     %% Generate trajectory\n');
fprintf('  run(''tasks/lifting/main_rne_fixedbase.m'')    %% Compute torques\n');
fprintf('  run(''src/animation/animate_dualarm_3d.m'')    %% Animate result\n');
fprintf('\nType ''help rnedynamicFixedBase1610'' for core solver documentation.\n');
fprintf('=========================================================\n\n');
