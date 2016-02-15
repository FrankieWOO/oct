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
    I = model.I;
    
 	fx=@(x)qddot_maccepa(x(1),x(2),u,model);
 	dqddotdx=J_fd(fx,x);
 	fu=@(u)qddot_maccepa(x(1),x(2),u,model);
 	dqddotdu=J_fd(fu,u);
    xdot_x = [0 1; dqddotdx  ];
 	xdot_u = [zeros(1,dimU); dqddotdu ];
    
    
%     dtaukdx1 = model_maccepa( 'maccepa_model_get_dtaukdx1', x, u, model);
%     dtaubdx2 = model_maccepa( 'maccepa_model_get_dtaubdx2', x, u, model);
%     dtaufdx2 = model_maccepa( 'maccepa_model_get_dtaufdx2', x, u, model);
%     dtaukdu1 = model_maccepa( 'maccepa_model_get_dtaukdu1', x, u, model);
%     dtaukdu2 = model_maccepa( 'maccepa_model_get_dtaukdu2', x, u, model);
%     dtaubdu3 = model_maccepa( 'maccepa_model_get_dtaubdu3', x, u, model);
    
% 	xdot_x = [0 1; dtaukdx1/I ( dtaubdx2 + dtaufdx2)/I  ];
% 	xdot_u = [zeros(1,dimU); dtaukdu1/I dtaukdu2/I dtaubdu3/I];

end

