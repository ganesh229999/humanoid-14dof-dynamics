function betarc_vel = compute_betarc_vel(robot, thdot)
% COMPUTE_BETARC_VEL  Velocity-dependent base bias β_rc (Khalil-Dombre).
%
% Computes the centripetal coupling between joint motions and base velocity:
%   β_rc = Σ_j  cross(z0j, cross(z0j, v0)) · θ̇_j
%
% This term is zero when the base is stationary (v0 = 0).
%
% INPUTS:
%   robot  – struct from get_robot_paramsmobile (must contain .T, .v0)
%   thdot  – n×1 joint velocity vector [rad/s]
%
% OUTPUT:
%   betarc_vel – 3×1 bias vector [ax; ay; α]

n=numel(thdot); betarc_vel=zeros(3,1); v0=robot.v0;
for j=1:n
    z0j=robot.T{j}(1:3,1:3)*[0;0;1];
    betarc_vel=betarc_vel+cross(z0j,cross(z0j,v0))*thdot(j);
end
end
