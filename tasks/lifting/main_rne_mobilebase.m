% MAIN_RNE_MOBILEBASE  Mobile-base RNE torque computation for lifting task.
%
% Extends the fixed-base simulation by accounting for dynamic coupling
% between the robotic arms and the mobile platform. The platform acceleration
% is solved from Newton-Euler coupling at each timestep.

clc; tic;
filename = 'Testoctlifting.xlsx';

%% Locate and verify file
fileFullPath = which(filename);
if isempty(fileFullPath), fileFullPath = fullfile(pwd,filename); end
if ~exist(fileFullPath,'file'), error('File not found: %s',fileFullPath); end
fprintf('Using: %s\n', fileFullPath);

%% Load trajectory
th_A    = readmatrix(fileFullPath,'Sheet',1).';
thdot_A = readmatrix(fileFullPath,'Sheet',2).';
thddot_A= readmatrix(fileFullPath,'Sheet',3).';
th_B    = readmatrix(fileFullPath,'Sheet',4).';
thdot_B = readmatrix(fileFullPath,'Sheet',5).';
thddot_B= readmatrix(fileFullPath,'Sheet',6).';

%% Parameters
objParams.mass  = 1.056;
objParams.I     = diag([0.00107, 0.05632, 0.05738]);
objParams.g_obj = [-9.81; 0; 0];

baseState.T_w0 = eye(4);
baseState.v0   = [0; 4; 0];
baseState.w0   = [0; 3; 0];

%% Run dual-arm mobile-base RNE
[tauA,tauB,M_A,M_B,C_A,C_B,G_A,G_B,a_base_all] = rnemobilebasec(...
    th_A,th_B,thdot_A,thdot_B,thddot_A,thddot_B,baseState,objParams);

%% Save
[outFolder,~,~] = fileparts(fileFullPath);
matOut = fullfile(outFolder,'RNE_DualArm_Mobile.mat');
save(matOut,'tauA','M_A','C_A','G_A','tauB','M_B','C_B','G_B','a_base_all');
fprintf('Saved: %s\n', matOut);

%% Display & export
fprintf('\nBase acceleration (first 5 steps):\n');
disp(a_base_all(:,1:min(5,end)).');
exportAndPlotTau(tauA, 0.02, 'TestoctLTauA.xlsx', 'all', 'ArmA');
exportAndPlotTau(tauB, 0.02, 'TestoctLTauB.xlsx', 'all', 'ArmB');
fprintf('Elapsed: %.3f s\n', toc);
