function a_base = compute_base_accel(thA,thdot_A,thddotA,thB,thdot_B,thddotB,baseState,extEntries)
% COMPUTE_BASE_ACCEL  Shared base acceleration via Newton-Euler arm coupling.
%
% Solves for the mobile platform acceleration by coupling both arms through
% the Khalil-Dombre reaction-coordinate formulation:
%
%   Mbase · a_base = −[Jrc_A·q̈_A + Jrc_B·q̈_B + β_rc_A + β_rc_B + β_ext]
%
% INPUTS:
%   thA/thB       – 7×1 joint angles     [rad]
%   thdot_A/B     – 7×1 joint velocities [rad/s]
%   thddotA/thddotB – 7×1 joint accelerations [rad/s²]
%   baseState     – struct: .T_w0, .v0, .w0
%   extEntries    – external wrench struct array (may be empty)
%
% OUTPUT:
%   a_base – 3×1 platform acceleration [ax; ay; α] in base frame

robA=get_robot_paramsmobile('A',thA,baseState.T_w0,baseState.v0,baseState.w0);
robB=get_robot_paramsmobile('B',thB,baseState.T_w0,baseState.v0,baseState.w0);

[Mbase,Jrc,~]=sum_base_coupling(robA,robB);
betarc = compute_betarc_vel(robA,thdot_A) + ...
         compute_betarc_vel(robB,thdot_B) + ...
         compute_betarc_ext(extEntries);

rhs = Jrc(:,1:7)*thddotA + Jrc(:,8:14)*thddotB + betarc + ...
      baseState.v0 + baseState.w0;
a_base = -Mbase \ rhs;
end
