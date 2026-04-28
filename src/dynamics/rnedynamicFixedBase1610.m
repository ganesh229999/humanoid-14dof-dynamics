function [tau, M, C, G] = rnedynamicFixedBase1610(th, thdot, thddot, armType, objParams)
% RNEDYNAMICFIXEDBASE1610  Recursive Newton-Euler dynamics for fixed-base arm.
%
% Computes joint torques τ and dynamic matrices (M, C, G) for every
% timestep in the input trajectory using the Recursive Newton-Euler (RNE)
% algorithm (Luh, Walker & Paul, 1980).
%
%   τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)  +  τ_ext
%
% ═══════════════════════════════════════════════════════════════════════════
% ALGORITHM OVERVIEW:
%   Forward pass  – propagate angular/linear velocity & acceleration
%                   from base (link 0) to EE (link 7), then to COM.
%   Backward pass – accumulate forces & moments from EE inward to base.
%   Decomposition – extract M by perturbing q̈ with unit vectors;
%                   extract C = τ_total − M·q̈ − G.
%
% INPUTS:
%   th      – N×7  joint positions    [rad]
%   thdot   – N×7  joint velocities   [rad/s]
%   thddot  – N×7  joint accelerations[rad/s²]
%   armType – 'A' | 'B'
%   objParams – struct: .mass [kg], .I [3×3 kg·m²], .g_obj [3×1 m/s²]
%
% OUTPUTS:
%   tau  – N×7   joint torques            [N·m]
%   M    – 7×7×N inertia matrices
%   C    – N×7   Coriolis/centrifugal torques
%   G    – N×7   gravity torques
%
% REFERENCE: Luh, J.Y.S., Walker, M.W. & Paul, R.P.C. (1980).
%   "On-line Computational Scheme for Mechanical Manipulators."
%   Trans. ASME J. Dynamic Systems, Measurement & Control, 102(2), 69–76.
% ═══════════════════════════════════════════════════════════════════════════

if nargin < 4, armType = 'A'; end
if nargin < 5, error('objParams must be provided.'); end

numLinks = 8;
if size(th,1) == 1, N_samples = 1; else, N_samples = size(th,1)-1; end

tau   = zeros(N_samples, 7);
M_mat = zeros(7,7,N_samples);
G_vec = zeros(N_samples, 7);
C_vec = zeros(N_samples, 7);

%% ── External Force Input ──────────────────────────────────────────────────
prompt = sprintf('External forces on arm %s? (1=Yes, 0=No): ', upper(armType));
ext_force_flag = input(prompt);
if ext_force_flag == 1
    method = input('Input method: (1) Manual  (2) Excel: ');
    extEntries = struct('joint',{},'wrench',{});
    if method == 1
        n = input('Number of wrench entries: ');
        for idx = 1:n
            jn = input(sprintf('Joint number (1-8) for entry %d: ', idx));
            fprintf('Wrench at joint %d [fx fy fz mx my mz]:\n', jn);
            extEntries(idx).joint  = jn;
            extEntries(idx).wrench = [input('fx='); input('fy='); input('fz=');
                                      input('mx='); input('my='); input('mz=')];
        end
    else
        fname = input('Excel filename: ','s');
        ext   = readmatrix(fname);
        for idx = 1:size(ext,1)
            extEntries(idx).joint  = ext(idx,2);
            extEntries(idx).wrench = ext(idx,3:8)';
        end
    end
else
    extEntries = [];
    disp('No external forces.');
end

%% ── Main Loop ─────────────────────────────────────────────────────────────
for i = 1:N_samples
    th_i     = th(i,:)';
    thdot_i  = thdot(i,:)';
    thddot_i = thddot(i,:)';

    robot = get_robot_params(armType, th_i);

    % Full torque
    tau_total = compute_tau_corrected(thdot_i, thddot_i, robot, numLinks, objParams);

    % External torque contribution
    tau_ext = zeros(7,1);
    for idx = 1:numel(extEntries)
        jn  = extEntries(idx).joint;
        w   = extEntries(idx).wrench;
        Jp  = computePartialJacobiaN(armType, th_i, jn);
        Rp  = get_partial_rotation(armType, th_i, jn);
        nj  = size(Jp,2);
        wr  = [Rp*w(1:3); Rp*w(4:6)];
        tau_ext(1:nj) = tau_ext(1:nj) + Jp'*wr;
    end

    % Gravity torque (zero vel/acc)
    tau_grav = compute_tau_corrected(zeros(7,1), zeros(7,1), robot, numLinks, objParams);

    % Inertia matrix (column-by-column unit perturbation)
    M_i = zeros(7,7);
    for j = 1:7
        e = zeros(7,1); e(j)=1;
        M_i(:,j) = compute_tau_corrected(zeros(7,1), e, robot, numLinks, objParams) - tau_grav;
    end

    C_i = tau_total - (M_i*thddot_i + tau_grav);

    tau(i,:)      = (tau_total + tau_ext)';
    M_mat(:,:,i)  = M_i;
    G_vec(i,:)    = tau_grav';
    C_vec(i,:)    = C_i';
end

M = M_mat; C = C_vec; G = G_vec;
end

% ═══════════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
% ═══════════════════════════════════════════════════════════════════════════

function tau_s = compute_tau_corrected(thdot_i, thddot_i, robot, numLinks, objParams)
% RNE core: forward pass then backward pass → joint torques.
m=robot.m; Pc=robot.Pc; I=robot.I; g0=robot.g0;
A=robot.A; Ri=robot.Ri; T=robot.T;
numJ=7;

% Forward recursion
w=cell(1,numLinks+1); aa=cell(1,numLinks+1); al=cell(1,numLinks+1);
w{1}=[0;0;0]; aa{1}=[0;0;0]; al{1}=[g0;0;0];
for j=1:numJ
    w{j+1}  = Ri{j}'*(w{j}+[0;0;thdot_i(j)]);
    aa{j+1} = Ri{j}'*(aa{j}+cross(w{j},[0;0;thdot_i(j)])+[0;0;thddot_i(j)]);
    p=A{j}(1:3,4);
    al{j+1} = Ri{j}'*(al{j}+cross(aa{j},p)+cross(w{j},cross(w{j},p)));
end

% COM accelerations
ac=cell(1,numJ);
for j=1:numJ
    ac{j}=al{j+1}+cross(aa{j+1},Pc(:,j))+cross(w{j+1},cross(w{j+1},Pc(:,j)));
end

% Link forces & moments
Fl=cell(1,numJ); Nl=cell(1,numJ);
for j=1:numJ
    Fl{j}=m(j)*ac{j};
    Nl{j}=I{j}*aa{j+1}+cross(w{j+1},I{j}*w{j+1});
end

% Backward recursion
f=cell(1,numLinks+1); n=cell(1,numLinks+1);
for k=1:numLinks+1, f{k}=zeros(3,1); n{k}=zeros(3,1); end
[f{numJ+1}, n{numJ+1}] = compute_object_dynamics(T, objParams);
for j=numJ:-1:1
    p=A{j+1}(1:3,4);
    f{j}=Ri{j+1}*f{j+1}+Fl{j};
    n{j}=Ri{j+1}*n{j+1}+cross(Pc(:,j),Fl{j})+cross(p,Ri{j+1}*f{j+1})+Nl{j};
end

tau_s=zeros(7,1);
for j=1:7, tau_s(j)=n{j}(3); end
end

function [f8,n8] = compute_object_dynamics(T, objParams)
% Object wrench at EE: gravitational + inertial load split between arms.
m_o=objParams.mass; Ic=objParams.I;
g_obj=objParams.g_obj;
IB=Ic; wc=[0;0;0];
rx=0.1; ry=0.1; rz=0.1;
S=[0,-rz,ry; rz,0,-rx; -ry,rx,0];
Jo=[eye(3),S; zeros(3),eye(3)];
acc=[0.0082;0;0;0;0;0];
FI=(m_o*9.81)/2;
NE=[m_o*eye(3), zeros(3); zeros(3),IB]*acc + [-m_o*eye(3)*g_obj; cross(wc,IB*wc)];
F=(pinv(Jo)'*NE)+FI;
R0E=T{end}(1:3,1:3);
FM=-[R0E',zeros(3);zeros(3),R0E']*F(1:6);
f8=FM(1:3); n8=FM(4:6);
end
