function betarc_ext = compute_betarc_ext(extEntries)
% COMPUTE_BETARC_EXT  External-wrench bias β_rc on the base.
%
% Accumulates force components (x, y) and moment (z) from all external
% wrenches applied to the system, contributing to the base acceleration RHS.
%
% INPUT:
%   extEntries – struct array with field .wrench = [fx;fy;fz;mx;my;mz]
%
% OUTPUT:
%   betarc_ext – 3×1 [Fx_total; Fy_total; Mz_total]

betarc_ext=zeros(3,1);
for k=1:numel(extEntries)
    w=extEntries(k).wrench;
    betarc_ext(1:2)=betarc_ext(1:2)+w(1:2);
    betarc_ext(3)  =betarc_ext(3)  +w(6);
end
end
