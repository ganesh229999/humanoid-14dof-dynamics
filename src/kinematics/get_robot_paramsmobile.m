function robot = get_robot_paramsmobile(armType, th_i, T_w0, v0, w0)
% GET_ROBOT_PARAMSMOBILE  Kinematic/dynamic parameters for a 7-DOF arm
%                         mounted on a mobile (floating) base platform.
%
% Extends get_robot_params by:
%   1. Incorporating the world-to-base transform T_w0
%   2. Storing base linear/angular velocity (v0, w0) for RNE propagation
%   3. Computing the Khalil–Dombre effective base inertia (Mbase),
%      reaction-coordinate Jacobian (Jrc), and velocity bias (betarc)
%      for solving base acceleration from Newton-Euler coupling.
%
% ═══════════════════════════════════════════════════════════════════════════
% INPUTS:
%   armType  – 'A' | 'B'
%   th_i     – 7×1  joint angles [rad]
%   T_w0     – 4×4  world-to-base homogeneous transform
%   v0       – 3×1  base linear  velocity in base frame [m/s]
%   w0       – 3×1  base angular velocity in base frame [rad/s]
%
% OUTPUTS: robot struct (all fields of get_robot_params, plus)
%   .v0      – 3×1  base linear velocity
%   .w0      – 3×1  base angular velocity
%   .Mbase   – 3×3  effective base inertia (planar: x, y, yaw)
%   .Jrc     – 3×7  reaction-coordinate Jacobian
%   .betarc  – 3×1  velocity-dependent base bias (zero if v0=0)
%
% REFERENCE:
%   Khalil & Dombre (2002), "Modeling, Identification and Control of Robots"
%   Chapter 9 – Dynamic modelling of mobile manipulators
% ═══════════════════════════════════════════════════════════════════════════

%% ── DH Offsets [m] ───────────────────────────────────────────────────────
if strcmp(armType,'B'), d1 = -0.3180; else, d1 = 0.3180; end
d2 = -0.2510; d3 = -0.2340; d4 = -0.1680; d5 = -0.2090;

%% ── DH Table ─────────────────────────────────────────────────────────────
robot.DH = [ 0,      0,   pi,     d1;
            -pi/2,   0,  -pi/2,   0;
             pi/2,   0,  -pi/2,   d2;
            -pi/2,   0,   0,      0;
             pi/2,   0,   0,      d3;
            -pi/2,   0,   0,      0;
             pi/2,   0,   0,      d4;
             0,      0,   0,      d5];

%% ── Dynamic Parameters ───────────────────────────────────────────────────
robot.m  = [2.0412, 1.8830, 1.2760, 1.1340, 0.9780, 0.7920, 3.2100, 0];

robot.Pc = [1e-3*[  0.06,  -2.94,  -0.21,  -0.11,   0.09,   0.18,  -0.03,  -0.03;
                  -13.42,  -0.08,  -0.05,  -4.76,  11.08,  -4.31,   3.18,   3.18;
                 -118.30, -52.17, 142.80,  -8.94, 125.60,   9.02, -90.44, -90.44]];

I1 = [4021500,    1920,    3510; 1920, 1548200,  67840;  3510,  67840, 3812600]/1e6;
I2 = [2684300,     -31,  229600;  -31, 2974100,  -4020; 229600, -4020, 1284700]/1e6;
I3 = [2341800,     398,   31840;  398, 2334500,   2660;  31840,  2660,  514200]/1e6;
I4 = [941600,       54,   -4280;   54,  678900,  44810;  -4280, 44810,  648300]/1e6;
I5 = [2014200,    1564,   -4250; 1564, 1669800,-551400;  -4250,-551400, 649100]/1e6;
I6 = [583200,     -768,    5160; -768,  437800, -33340;   5160,-33340,  385400]/1e6;
I7 = [41983200,   -161,   22460; -161,39847600,-2335200; 22460,-2335200,2608300]/1e6;
I8 = [2014200,    2601,   -4250; 2601, 2649800,-364200;  -4250,-364200, 463700]/1e6;
robot.I = {I1,I2,I3,I4,I5,I6,I7,I8};

robot.g0 = -9.81;
robot.v0 = v0;
robot.w0 = w0;

%% ── DH Transform Chain (base frame = T_w0) ──────────────────────────────
numLinks = size(robot.DH,1);
T = cell(1,numLinks+1); P = cell(1,numLinks+1);
A = cell(1,numLinks);   Ri= cell(1,numLinks);

T{1} = T_w0;
P{1} = T_w0(1:3,4);

for j = 1:numLinks
    theta = robot.DH(j,3) + (j<=7)*th_i(j);
    alp=robot.DH(j,1); a=robot.DH(j,2); d=robot.DH(j,4);
    ct=cos(theta); st=sin(theta); ca=cos(alp); sa=sin(alp);
    A{j} = [ct,    -st,    0,   a;
            st*ca, ct*ca, -sa, -sa*d;
            st*sa, ct*sa,  ca,  ca*d;
            0,     0,      0,   1];
    T{j+1}=T{j}*A{j}; P{j+1}=T{j+1}(1:3,4); Ri{j}=A{j}(1:3,1:3);
end
robot.T=T; robot.P=P; robot.A=A; robot.Ri=Ri;

%% ── Khalil–Dombre: Effective Base Inertia & Reaction Jacobian ────────────
% Compute COM positions and rotated inertias in base frame
m_vec = robot.m;
link_COM_base = zeros(3, numLinks);
link_Iz_base  = zeros(1, numLinks);

for i = 1:numLinks
    R_i  = robot.T{i+1}(1:3,1:3);
    p_i  = robot.T{i+1}(1:3,4);
    com_i_base = p_i + R_i * robot.Pc(:,i);
    link_COM_base(:,i) = com_i_base;
    I_rot = R_i * robot.I{i} * R_i';
    link_Iz_base(i) = I_rot(3,3);
end

% Aggregate inertial properties
M_links = sum(m_vec);
x = link_COM_base(1,:); y = link_COM_base(2,:);
mx = sum(m_vec.*x);  my = sum(m_vec.*y);
Izz = sum(link_Iz_base + m_vec.*(x.^2 + y.^2));

% Platform own inertia
m_base = 5.0; I_base_z = 0.1;
M_total = M_links + m_base;
Izz_sum = Izz + I_base_z;

% 3×3 effective base inertia (planar: x, y, yaw)
robot.Mbase = [ M_total,      0,   -my;
                   0,    M_total,    mx;
                 -my,        mx,  Izz_sum ];

% 3×7 Reaction-coordinate Jacobian
p_end = robot.T{end}(1:3,4);
Jrc = zeros(3,7);
for j = 1:7
    z0j = robot.T{j}(1:3,1:3)*[0;0;1];
    p_j = robot.T{j}(1:3,4);
    Jrc(1:2,j) = [-z0j(3)*(p_end(2)-p_j(2));  z0j(3)*(p_end(1)-p_j(1))];
    Jrc(3,j)   =  z0j(3);
end
robot.Jrc = Jrc;

% Velocity-dependent bias (zero when v0=0)
robot.betarc = zeros(3,1);
end
