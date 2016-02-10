% 'Reaching task' cost function for the 1-DOF plants (e.g., MACCEPA, swinger, etc.) for use with ILQR/G
%
% [l, l_x, l_xx, l_u, l_uu, l_ux] = fn_cost_contact_task_1dof_plants ( x, u, t, w, xt, fnTorque )
%
% Implements cost function of the form:
%
% J = w1 (q(T)-q*)^2 + w2 qdot(T)^2 + \int_0^T w3 tau*qdot dt
% 
% in:
%    x, u, t  - state, command, time
%    p        - parameter struct, containing
%     .w      - weight vector [w1,w2,w3]'
%     .qt     - target in joint space
%     .tau    - actuator torque (function handle)
%
% out:
%    l, l_x, l_xx, l_u, l_uu, l_ux - cost and derivatives
%
function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_contact_task_1dof_plants ( x, u, t, p )

fl = @( x, u, t ) l_contact_task_1dof_plants ( x, u, t, p );
l  = fl ( x, u, t );

% compute derivatives of cost
if nargout>1
	% TODO % find analytical derivatives 
	flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
	[l_x ,l_u      ] = flJ ( x, u, t );
	flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
	[l_xx,l_uu,l_ux] = flH  ( x, u, t );
end

