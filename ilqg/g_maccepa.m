% Dynamics function for the MACCEPA.
%
% in: 
%     x     - state [position; velocity]
%     u     - command vector [desired motor positions in radiens; damping command]
%     model - struct containing model parameters
%
% out:
%     xdot,xdot_x,xdot_u - state change and derivatives
%
function [xdot, xdot_x, xdot_u] = g_maccepa ( x, u, model )

xdot = f_maccepa ( x, u, model );

% compute xdot_x, xdot_u
if nargout > 1
	% finite difference calculation
	dimQ = model.dimQ;
	dimX = 2*dimQ;
	dimU = model.dimU;

	fx=@(x)qddot_maccepa(x(1),x(2),u,model);
	dqddotdx=J_fd(fx,x);
	fu=@(u)qddot_maccepa(x(1),x(2),u,model);
	dqddotdu=J_fd(fu,u);

	xdot_x = [0 1; dqddotdx];
	xdot_u = [0 0; dqddotdu];

end

