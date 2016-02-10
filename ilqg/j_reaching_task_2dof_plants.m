% 'Reaching task' cost function for the 2-DOF plants (e.g., Kawato arm) for use with ILQR/G
%
% [l, l_x, l_xx, l_u, l_uu, l_ux] = j_reaching_task_2dof_plants ( x, u, t, w, qt, fnTorque )
%
% Implements cost function of the form:
%
% J = w1|q(T)-q*|^2 + w2|qdot(T)|^2 + \int_0^T w3|tau|^2 dt
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
function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_reaching_task_2dof_plants ( x, u, t, p )

fl = @( x, u, t ) l_reaching_task_2dof_plants ( x, u, t, p );
l  = fl ( x, u, t );

% compute derivatives of cost
if nargout>1
	if isnan(t)
		try
			w  = p.w;
			qt = p.qt;

			dimX     = size(x,1);
			dimU     = size(u,1);

			q        = x(1:2);
			qdot     = x(3:end);

			l_x(1:2) = 2*w(1)*(q-qt);
			l_x(3:4) = 2*w(2)*qdot;
			l_xx     = 2*diag([w(1)*ones(1,2) w(2)*ones(1,2)]);

			l_u      = zeros(dimU,1);
			l_uu     = zeros(dimU,dimU);
			l_ux     = zeros(dimU,dimX);
		catch % finite differences should always work  
			flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
			[l_x ,l_u      ] = flJ ( x, u, t );
			flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
			[l_xx,l_uu,l_ux] = flH  ( x, u, t );
		end
	else
		% TODO % find analytical derivatives 
		flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
		[l_x ,l_u      ] = flJ ( x, u, t );
		flH =@(x,u,t)H_cost_fd ( flJ, x, u, t );
		[l_xx,l_uu,l_ux] = flH  ( x, u, t );
	end
end

