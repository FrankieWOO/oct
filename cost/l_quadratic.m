% Finite horizon quadratic cost function.
%
% 	J = \int_0^\infty x^T Q x + u^T R u dt
% 
% in: 
%    x - state
%    u - command
%    p - parameter struct, containing
%     .Q,R  - cost parameters
%
% out:
%    l - cost
% 
function l = l_quadratic ( x, u, p )

Q  = p.Q;
R  = p.R;
l=sum(x.*(Q*x))+sum(u.*(R*u),1);

