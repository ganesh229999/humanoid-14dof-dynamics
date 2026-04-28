function robot = get_robot_params(armType, th_i)
% GET_ROBOT_PARAMS  Kinematic and dynamic parameters for one 7-DOF arm of a
%                   fixed-base dual-arm humanoid robot, plus full DH transform chain.
%
% ═══════════════════════════════════════════════════════════════════════════
%  Robot Architecture
% ═══════════════════════════════════════════════════════════════════════════
%  • 7 revolute joints per arm  →  14 DOF total (dual-arm)
%  • Modified DH convention (Craig, 2005)
%  • Link 8: rigid end-effector (EE) offset — no joint variable
%  • Arm A (right): d1 > 0  |  Arm B (left): d1 < 0
%  • Gravity: −X direction of base frame
%
%  Joint layout (human-arm analogy):
%    J1 – Shoulder flexion/extension       (lateral offset d1)
%    J2 – Shoulder abduction/adduction
%    J3 – Shoulder internal/external rot.  (upper-arm d2)
%    J4 – Elbow flexion/extension
%    J5 – Forearm pronation/supination     (forearm d3)
%    J6 – Wrist flexion/extension
%    J7 – Wrist radial/ulnar deviation     (wrist d4)
%    EE – Tool flange (fixed, d5)
%
% ───────────────────────────────────────────────────────────────────────────
% INPUTS:
%   armType  – char  : 'A' (right arm) | 'B' (left arm)
%   th_i     – 7×1  : joint angles [rad]
%
% OUTPUTS:  robot struct
%   .DH      – 8×4  : DH table [alpha, a, theta_offset, d]
%   .m       – 1×7  : link masses [kg]
%   .Pc      – 3×7  : COM positions in link frames [m]
%   .I       – {7}  : 3×3 inertia tensors about COMs [kg·m²]
%   .g0      – scalar: gravity magnitude [m/s²]
%   .T       – {9}  : absolute transforms  T{1}=eye(4), T{k+1}=frame k
%   .A       – {8}  : relative DH transforms
%   .Ri      – {8}  : 3×3 rotation components of A{k}
%   .P       – {9}  : 3×1 joint origin positions in base frame
%
% ───────────────────────────────────────────────────────────────────────────
% USAGE:
%   q      = zeros(7,1);
%   robot  = get_robot_params('A', q);
%   T_ee   = robot.T{end};       % 4×4 EE transform wrt base
%   p_ee   = T_ee(1:3,4);        % EE position [m]
%   R_ee   = T_ee(1:3,1:3);      % EE orientation
%
% CALLED BY: jacobianOfArmA, jacobianOfArmB, computePartialJacobiaN,
%            rnedynamicFixedBase1610, get_partial_rotation
% ═══════════════════════════════════════════════════════════════════════════

%% ── Link-Length Offsets [m] ───────────────────────────────────────────────
if strcmp(armType,'B')
    d1 = -0.4500;   % Left  arm lateral shoulder offset
else
    d1 =  0.4500;   % Right arm lateral shoulder offset
end
d2 = -0.3510;   % Upper-arm axial offset
d3 = -0.4340;   % Forearm axial offset
d4 = -0.2680;   % Wrist axial offset
d5 = -0.1090;   % Tool-flange axial offset

%% ── DH Parameter Table  [alpha | a | theta_offset | d] ──────────────────
robot.DH = [ 0,      0,   pi,     d1;   % Joint 1 – Shoulder flex
            -pi/2,   0,  -pi/2,   0;    % Joint 2 – Shoulder abd
             pi/2,   0,  -pi/2,   d2;   % Joint 3 – Shoulder rot
            -pi/2,   0,   0,      0;    % Joint 4 – Elbow
             pi/2,   0,   0,      d3;   % Joint 5 – Forearm
            -pi/2,   0,   0,      0;    % Joint 6 – Wrist flex
             pi/2,   0,   0,      d4;   % Joint 7 – Wrist dev
             0,      0,   0,      d5];  % EE frame  (fixed)

%% ── Link Masses [kg] ──────────────────────────────────────────────────────
robot.m = [3.0412, 2.8830, 1.1260, 2.100, 1.9780, 0.1220, 3.5100];

%% ── Centre-of-Mass Positions in Link Frames [m] ──────────────────────────
%  Column k = COM of link k, expressed in frame k.
%  Obtained from SolidWorks mass-property export.
robot.Pc = 1e-3 * ...
    [  0.06,  -2.04,  -0.31,  -0.11,   0.29,   0.38,  -0.83;   % x
     -9.42,  -0.14,  -0.20,  -3.76,  10.08,  -1.31,   1.18;   % y
    -112.30, -51.17, 169.80,  -9.94, 125.60,   9.02, -78.44];  % z

%% ── Inertia Tensors About Each Link COM [kg·m²] ──────────────────────────
%  Full 3×3 symmetric matrix in body frame (products of inertia included).
I1 = [5031500,    2920,    4510;
         1720, 1458200,   65440;
         3840,   61840, 3742600] / 1e6;

I2 = [2864300,     -29,  226900;
          -29, 2987100,   -4010;
       226900,   -4010, 1284700] / 1e6;

I3 = [2431800,     388,   31480;
          388, 2334500,    2550;
        31480,    2550,  513200] / 1e6;

I4 = [941600,      59,   -4210;
          59,  678900,   44410;
       -4210,   44410,  684300] / 1e6;

I5 = [2041200,    1454,   -4220;
         1454, 1669800, -542400;
        -4220, -542400,  639100] / 1e6;

I6 = [538200,    -786,    5210;
        -786,  437800,  -33440;
        5210,  -33440,  384500] / 1e6;

I7 = [41893200,      -150,    22160;
           -150,  39847600, -2235200;
          22160, -2235200,  2680300] / 1e6;

robot.I  = {I1, I2, I3, I4, I5, I6, I7};

%% ── Gravity ───────────────────────────────────────────────────────────────
robot.g0 = -9.81;   % [m/s²]  direction applied in RNE forward pass

%% ── DH Transform Chain ────────────────────────────────────────────────────
numLinks = size(robot.DH, 1);          % 8 (7 joints + EE)
T  = cell(1, numLinks+1);
P  = cell(1, numLinks+1);
A  = cell(1, numLinks);
Ri = cell(1, numLinks);

T{1} = eye(4);
P{1} = zeros(3,1);

for j = 1:numLinks
    % Joint angle: active for j=1..7, zero for EE row (j=8)
    theta = robot.DH(j,3) + (j <= 7) * th_i(j);
    alp = robot.DH(j,1);
    a   = robot.DH(j,2);
    d   = robot.DH(j,4);

    ct = cos(theta); st = sin(theta);
    ca = cos(alp);   sa = sin(alp);

    % Standard DH matrix: Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α)
    A{j} = [ ct,      -st,      0,    a;
             st*ca,   ct*ca,  -sa,  -sa*d;
             st*sa,   ct*sa,   ca,   ca*d;
             0,        0,       0,    1  ];

    T{j+1} = T{j} * A{j};
    P{j+1} = T{j+1}(1:3, 4);
    Ri{j}  = A{j}(1:3, 1:3);
end

robot.T  = T;
robot.P  = P;
robot.A  = A;
robot.Ri = Ri;
end
