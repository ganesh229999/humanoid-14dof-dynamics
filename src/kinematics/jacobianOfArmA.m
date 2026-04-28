function J = jacobianOfArmA(q)
% JACOBIANOFАРМА  Computes the 6×7 geometric Jacobian for Arm A.
%
%  J = [Jv; Jw] where:
%    Jv(:,i) = cross(z_i, p_E - p_i)   [linear velocity contribution]
%    Jw(:,i) = z_i                      [angular velocity contribution]
%
%  Uses the standard geometric Jacobian formulation for revolute joints.
%  Frame conventions follow get_robot_params (Modified DH, Craig 2005).
%
% INPUT:   q  – 7×1 joint angle vector [rad]
% OUTPUT:  J  – 6×7 Jacobian matrix
%               rows 1-3: linear velocity  (Jv)
%               rows 4-6: angular velocity (Jw)
%
% SINGULARITIES: det(J*J') → 0 at certain configurations (wrist/elbow lock).
%   Use damped pseudo-inverse for inversion near singularities.

q = q(:);
if numel(q) ~= 7, error('jacobianOfArmA: q must be 7×1.'); end

robot = get_robot_params('A', q);
T     = robot.T;
pE    = T{end}(1:3,4);   % EE position in base frame

Jv = zeros(3,7);
Jw = zeros(3,7);
for i = 1:7
    if i == 1
        z_i = T{1}(1:3,3);   % base z-axis = [0;0;1]
        p_i = T{1}(1:3,4);   % base origin  = [0;0;0]
    else
        z_i = T{i+1}(1:3,3);
        p_i = T{i+1}(1:3,4);
    end
    Jv(:,i) = cross(z_i, pE - p_i);
    Jw(:,i) = z_i;
end
J = [Jv; Jw];
end
