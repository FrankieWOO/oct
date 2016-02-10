% Dynamics function for the 1-DOF Kawato joint (one joint, two 'kawato model' muscles).
%
% in: 
%     x     - state [position; velocity]
%     u     - command [muscle activations]
%     model - struct containing model parameters
%
% out:
%     xdot,xdot_x,xdot_u - state change and derivatives
%
function [xdot, xdot_x, xdot_u] = g_kawato_1dof ( x, u, model )

xdot = f_kawato_1dof ( x, u, model );

% compute xdot_x, xdot_u
if nargout > 1
	% finite difference calculation
	dimQ = model.dimQ;
	dimX = 2*dimQ;
	dimU = model.dimU;

	fx=@(x)qddot_kawato_1dof(x(1),x(2),u,model);
	dqddotdx=J_fd(fx,x);
	fu=@(u)qddot_kawato_1dof(x(1),x(2),u,model);
	dqddotdu=J_fd(fu,u);

	xdot_x = [0 1; dqddotdx];
	xdot_u = [0 0; dqddotdu];

end

