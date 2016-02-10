% Function for calculating argmax_u (Q) from the weights of the Q-function for the LQR problem. 
%
%  [pi, L] = argmaxuQ_fn_basis_lqr ( theta )
%
% in: 
%     theta - basis function weights
%
% out:
%     pi    - optimal policy (function handle)
%     L     - policy gains (since this is a linear policy)
%
% Note: 
% This function assumes that you are using
% fn_basis_lqr.m as basis functions, and gives a
% closed-form solution.
%
function pi = argmaxuQ_fn_basis_lqr ( theta )

theta = reshape(theta,16,5);
[dummy inds] = max(theta,[],2);
%inds
% policy
pi=@(x)(inds(x)');

