% Estimate joint stiffness of 1-DOF Kawato model for a given command u and state x.
%
% in:
%      u     - muscle activations
%      model - struct containing model parameters
%
% out:
%      k     - stiffness
%
% Calculation is based on derivation given in Katayama and Kawato, 1993.
%
function k = get_stiffness_kawato_1dof ( u, model )

A = model.A;
K = diag(km_kawato_1dof(u,model));
k = A'*K*A;

