function R_partial = get_partial_rotation(armType, th, joint_num)
% GET_PARTIAL_ROTATION  Rotation matrix from base frame to frame at joint_num.
%
%  Used for transforming external wrenches from world frame to joint frame
%  before computing the equivalent joint torques via J^T * wrench.
%
% INPUTS:
%   armType   – 'A' | 'B'
%   th        – 7×1 joint angles [rad]
%   joint_num – target joint index (1..7)
%
% OUTPUT:
%   R_partial – 3×3 rotation matrix R_0^{joint_num}

robot    = get_robot_params(armType, th);
R_partial = robot.T{joint_num+1}(1:3,1:3);
end
