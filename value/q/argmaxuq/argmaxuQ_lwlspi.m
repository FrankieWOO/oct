% Function for calculating argmax_u (Q) from the weights of the Q-function represented with a quadratic basis. 
% 
%  [pi, L] = argmaxuQ_lwlspi ( model, dimX, dimU )
%
% in: 
%     theta - basis function weights
%     dimX  - dimensionality of state vector x
%     dimU  - dimensionality of command vector u
%
% out:
%     pi    - optimal policy (function handle)
%     L     - policy gains (since this is a linear policy)
%
% Note: 
% This function assumes that you are using
% fn_basis_quadratic.m as basis functions, and gives a
% closed-form solution.
%
function [pi, L] = argmaxuQ_lwlspi ( model, dimX, dimU, p )

Nc = size(model.w,3);
for nc=1:Nc
	[d1, d2, Lx(:,:,nc), Lu(:,:,nc), L1(:,:,nc)] = argmaxuQ_fn_basis_quadratic(model.w(:,:,nc),dimX,dimU);
end

pi=@(x)pi_lwlspi(x,Lx,Lu,L1,model.W,p);


% Function for calculating argmax_u (Q) from the weights of the Q-function represented with a quadratic basis. 
% 
%  [pi, L] = argmaxuQ_fn_basis_quadratic ( theta, dimX, dimU )
%
% in: 
%     theta - basis function weights
%     dimX  - dimensionality of state vector x
%     dimU  - dimensionality of command vector u
%
% out:
%     pi    - optimal policy (function handle)
%     L     - policy gains (since this is a linear policy)
%
% Note: 
% This function assumes that you are using
% fn_basis_quadratic.m as basis functions, and gives a
% closed-form solution.
%
function [pi, L, Wx, Wu, W1] = argmaxuQ_fn_basis_quadratic ( theta, dimX, dimU )

n = dimX; m = dimU; 

% convert theta to W
W = zeros(n+m+1,n+m+1);
inds = find([~tril(ones(n+m)),zeros(n+m,1);zeros(1,n+m+1)]); ninds = length(inds);
for i=1:length(inds), W(inds(i)) = .5*theta(n+m+i); end
W(end,1:n+m)=theta(1+ninds+n+m:ninds+2*(n+m));
W=W+W';
W=W+diag([theta(1:n+m);theta(end)]);

% set up linear system
Wx = W(n+1:n+m,  1:n  )+W(  1:n  ,n+1:n+m)';
Wu = W(n+1:n+m,n+1:n+m)+W(n+1:n+m,n+1:n+m)';
W1 = W(n+1:n+m,n+m+1);

% solve for gains
%L  = inv(Wu+1e-6*eye(m))*[Wx,W1];
%L  = Wu\[Wx,W1];
L  = (Wu+1e-6*eye(m))\[Wx,W1];

% policy
pi=@(x)fn_policy_linear([x;ones(1,size(x,2))],L);

