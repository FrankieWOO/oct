% Polynomial basis function for use on the 1-D LQR problem with linear function approximation fitting of Q.
%
%  phi = fn_basis_lqr_q ( x, u )
%
% Implements polynomial basis function of the form:
%
%  phi(x,u) = [x1^2, x2^2, u^2, x1*x2, x1*u, x2*u]'
%
% in: 
%     x,u - state, action
%
% out: 
%     phi - basis function prediction
%
% Note: This is a special case of fn_basis_quadratic
%       in which the linear terms are not computed.
%
function phi = phi_lqrq ( x, u )

phi(1,:)=x(1,:).^2;
phi(2,:)=x(2,:).^2;
phi(3,:)=u.^2;
phi(4,:)=x(1,:).*x(2,:);
phi(5,:)=x(1,:).*u;
phi(6,:)=x(2,:).*u;

