function [Mbase, Jrc, betarc] = sum_base_coupling(robA, robB)
% SUM_BASE_COUPLING  Aggregate both arms' base-coupling inertial terms.
%
% Combines the effective base inertia matrices and reaction-coordinate
% Jacobians from Arm A and Arm B into a single unified system used to
% solve for the platform acceleration.
%
% INPUTS:
%   robA, robB – structs from get_robot_paramsmobile (each has .Mbase, .Jrc, .betarc)
%
% OUTPUTS:
%   Mbase  – 3×3  combined effective base inertia
%   Jrc    – 3×14 stacked reaction Jacobian  [Jrc_A | Jrc_B]
%   betarc – 3×1  combined velocity bias

Mbase  = robA.Mbase  + robB.Mbase;
Jrc    = [robA.Jrc,   robB.Jrc];
betarc = robA.betarc + robB.betarc;
end
