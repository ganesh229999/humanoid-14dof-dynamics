function [f8, n8] = compute_object_dynamicsm(T, objParams)
% COMPUTE_OBJECT_DYNAMICSM  EE wrench for mobile-base solver.
%
% Mobile-base version: EE wrench set to zero by design (object dynamics
% handled implicitly through base coupling terms). This is a valid
% simplification when base acceleration is solved via Khalil-Dombre.
%
% INPUTS / OUTPUTS: same signature as compute_object_dynamics

f8 = zeros(3,1);
n8 = zeros(3,1);
end
