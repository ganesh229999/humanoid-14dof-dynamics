function R_partial = get_partial_rotationm(armType, th, joint_num, baseState)
% GET_PARTIAL_ROTATIONM  Rotation matrix to joint_num for mobile-base robot.
%
%  Mobile-base version of get_partial_rotation. Incorporates world-to-base
%  transform T_w0 from baseState when building the kinematic chain.
%
% INPUTS:
%   armType   – 'A' | 'B'
%   th        – 7×1 joint angles [rad]
%   joint_num – target joint index (1..7)
%   baseState – struct: .T_w0 (4×4), .v0 (3×1), .w0 (3×1)

robot    = get_robot_paramsmobile(armType, th, baseState.T_w0, baseState.v0, baseState.w0);
R_partial = robot.T{joint_num+1}(1:3,1:3);
end
