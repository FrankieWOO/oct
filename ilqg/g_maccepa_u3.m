
function [ xdot, xdot_x, xdot_u ] = g_maccepa_u3( x, u3, model, u1, u2 )
%   G_MACCEPA_U3 for fixed u1 and u2
%   Detailed explanation goes here
    
    u = [u1; u2; u3];
    xdot = f_maccepa ( x, u, model );
    
    
% compute xdot_x, xdot_u
if nargout > 1
	
    I = model.I;
    

    
    
     dtaukdx1 = model_maccepa( 'maccepa_model_get_dtaukdx1', x, u, model);
     dtaubdx2 = model_maccepa( 'maccepa_model_get_dtaubdx2', x, u, model);
     dtaufdx2 = model_maccepa( 'maccepa_model_get_dtaufdx2', x, u, model);
     
     dtaubdu3 = model_maccepa( 'maccepa_model_get_dtaubdu3', x, u, model);
    
 	xdot_x = [0 1; dtaukdx1/I ( dtaubdx2 + dtaufdx2)/I  ];
 	xdot_u = [0; dtaubdu3/I];


end

end