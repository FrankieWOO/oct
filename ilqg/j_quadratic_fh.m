% Finite horizon quadratic cost function.
%
% 	J = x^T QT x + \int_0^T x^T Q x + u^T R u dt
% 
% in: 
%    x - state
%    u - command
%    p - parameter struct, containing
%     .Q,R,QT - cost parameters
%
% out:
%    l,l_x,l_xx,l_u,l_uu,l_ux - cost and derivatives
% 
% NOTE: For use with ILQG, the function should be called with t=NaN to access the final cost (and derivatives).
%
function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_quadratic_fh ( x, u, t, p )

l = l_quadratic_fh ( x, u, t, p );

% compute derivatives of cost
if(nargout>1)
	dimX = size(x,1);
	dimU = size(u,1);

	if isnan(t)
		QT   = p.QT;

		l_x  = 2*QT*x;
		l_xx = 2*QT;

		l_u  = zeros(dimU,1);
		l_uu = zeros(dimU,dimU);

		l_ux = zeros(dimU,dimX);
	else
		Q    = p.Q;
		R    = p.R;

		l_x  = 2*Q*x;
		l_xx = 2*Q;

		l_u  = 2*R*u;
		l_uu = 2*R;

		l_ux = zeros(dimU,dimX);
	end
end
