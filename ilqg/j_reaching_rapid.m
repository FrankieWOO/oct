
function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_reaching_rapid(x,u,t,p)
% Input:
% x - position sequence
% u - command sequence
% t - time sequence
% p - parameters: x_target, T, epsilon
% p.x_target - target point
% p.T - the maximium time allowed
% p.epsilon - a small constant, by default 10^-8 in experiments
% p.dt - time interval
% c = /int_0^T (x(t) - x_target)^2 + epsilon (u^T * u) dt


fl = @(x,u,t) l_rapid_movement(x,u,t,p);
l = fl(x,u,t);


% compute derivatives of cost
if nargout>1
 % analytical derivatives
 l_x = [2*(x(1) - p.x_target);0] ;
 l_u = 2*p.epsilon*u ;
 l_xx = [ 2 0;0 0];
 l_uu = [2*p.epsilon 0 0;0 2*p.epsilon 0;0 0 2*p.epsilon] ;
 l_ux = zeros(3,2);
% 
%  flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
%  [l_x ,l_u      ] = flJ ( x, u, t );
%  flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
%  [l_xx,l_uu,l_ux] = flH  ( x, u, t );
end
