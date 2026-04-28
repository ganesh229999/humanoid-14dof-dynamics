function [f8, n8] = compute_object_dynamics(T, objParams)
% COMPUTE_OBJECT_DYNAMICS  End-effector wrench from manipulated object (fixed base).
%
% Computes the reaction force and moment exerted by a manipulated object
% on the robot EE, accounting for gravity and inertial loads.
% The load is split equally between both arms (cooperative manipulation).
%
% INPUTS:
%   T         – cell array of transforms from get_robot_params
%   objParams – struct: .mass [kg], .I [3×3], .g_obj [3×1 m/s²]
%
% OUTPUTS:
%   f8 – 3×1 EE force  [N]  in base frame
%   n8 – 3×1 EE moment [N·m] in base frame

m_o=objParams.mass;
Ic=objParams.I;
g_obj = isfield(objParams,'g_obj')*objParams.g_obj + ~isfield(objParams,'g_obj')*[-9.81;0;0];

IB=Ic; wc=[0;0;0];
rx=0.1; ry=0.1; rz=0.1;
Jo=[eye(3),[0,-rz,ry;rz,0,-rx;-ry,rx,0]; zeros(3),eye(3)];
acc=[0.0082;0;0;0;0;0];
FI=(m_o*9.81)/2;
NE=[m_o*eye(3),zeros(3);zeros(3),IB]*acc+[-m_o*eye(3)*g_obj;cross(wc,IB*wc)];
F=(pinv(Jo)'*NE)+FI;
R0E=T{end}(1:3,1:3);
FM=-[R0E',zeros(3);zeros(3),R0E']*F(1:6);
f8=FM(1:3); n8=FM(4:6);
end
