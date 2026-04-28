function J = jacobianOfArmB(q)
% JACOBIANOFARMB  Computes the 6×7 geometric Jacobian for Arm B.
%
%  Identical formulation to jacobianOfArmA; arm differentiation is handled
%  internally by get_robot_params('B', q) which sets d1 < 0.
%
% INPUT:   q  – 7×1 joint angle vector [rad]
% OUTPUT:  J  – 6×7 Jacobian  [Jv; Jw]

q = q(:);
if numel(q) ~= 7, error('jacobianOfArmB: q must be 7×1.'); end

robot = get_robot_params('B', q);
T     = robot.T;
pE    = T{end}(1:3,4);

Jv = zeros(3,7);
Jw = zeros(3,7);
for i = 1:7
    if i == 1
        z_i = T{1}(1:3,3);
        p_i = T{1}(1:3,4);
    else
        z_i = T{i+1}(1:3,3);
        p_i = T{i+1}(1:3,4);
    end
    Jv(:,i) = cross(z_i, pE - p_i);
    Jw(:,i) = z_i;
end
J = [Jv; Jw];
end
