function J_partial = computePartialJacobiaN(armType, th, joint_num)
% COMPUTEPARTIALJACOBIAN  Geometric Jacobian up to a specified joint.
%
%  Used to map an external wrench applied at joint_num back to joint torques.
%  J_partial is (6 × n_joints) where n_joints = min(joint_num, 7).
%
% INPUTS:
%   armType   – 'A' | 'B'
%   th        – 7×1 joint angle vector [rad]
%   joint_num – joint index where wrench acts (1..8;  8 = EE)
%
% OUTPUT:
%   J_partial – 6×n  partial Jacobian

th = th(:);
if numel(th) < 7, error('th must have 7 elements.'); end
joint_num = min(max(round(joint_num),1), 8);

robot = get_robot_params(armType, th);
T     = robot.T;
numT  = numel(T);

% EE position for this partial chain
idx_pE = min(joint_num+1, numT);
p_E    = T{idx_pE}(1:3,4);

n_joints = min(joint_num, 7);
Jv = zeros(3, n_joints);
Jw = zeros(3, n_joints);

for i = 1:n_joints
    if i == 1
        z_i = [0;0;1];  p_i = [0;0;0];
    else
        Ti  = T{min(i, numT)};
        z_i = Ti(1:3,3);  p_i = Ti(1:3,4);
    end
    Jv(:,i) = cross(z_i, p_E - p_i);
    Jw(:,i) = z_i;
end
J_partial = [Jv; Jw];
end
