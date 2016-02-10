% Estimate equilibrium position of simplified 1-joint, 2-muscle model.
% 
%   q0 = q0_kawato_1dof ( x, u, model )
%
% q0 is computed analytically as
%
%   q0 = inv(A'*K(u)*A)*A'*K(u)(l_m-l_0 + Ru)
%
% in:
%      u     - muscle activations
%      model - struct containing model parameters
%
% out:
%      q0    - equilibrium position
%
function q0 = q0_kawato_1dof ( u, model )

A     = model.A;
gr    = model.gr;
lm_l0 = model.lm_l0;

k  = km_kawato_1dof(u,model);
K  = diag(k);

q0 = pinv(A'*K*A)*A'*K*(lm_l0 + gr.*u);

