% Finite horizon quadratic cost function.
%
% 	J = x^T QT x + \int_0^T x^T Q x + u^T R u dt
% 
% in: 
%    x      - state
%    u      - command
%    p - parameter struct, containing
%     .Q,R,QT - cost parameters
%
% out:
%    l      - cost
% 
% NOTE: For use with ILQG, the function should be called with t=NaN to access the final cost.
%
function l = l_quadratic_fh ( x, u, t, p )

% compute cost
if isnan(t)
	QT = p.QT;
	l  = x'*QT*x;
else

	Q  = p.Q;
	R  = p.R;

	N = size(u,2);
	for n=1:N
	l(n)  = x(:,n)'*Q*x(:,n) + u(:,n)'*R*u(:,n);
	end
end

