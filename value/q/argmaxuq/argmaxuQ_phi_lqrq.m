% Function for calculating argmax_u (Q) from the weights of the Q-function for the LQR problem. 
%
%  [pi, L] = argmaxuQ_fn_basis_lqr_q ( theta )
%
% in: 
%     theta - basis function weights
%
% out:
%     pi    - optimal policy (function handle)
%     L     - policy gains (since this is a linear policy)
%
% Note: 
%      This function assumes that you are using fn_basis_lqr_q.m as basis
%      functions, and gives a closed-form solution.
%
function [pi, L] = argmaxuQ_phi_lqrq ( theta )

L=theta(5:6)'/(theta(3)*2);

% policy
pi=@(x)pi_linear(x,L);

