function [ l, l_x, l_xx, l_u, l_uu, l_ux ] = j_reaching_rapid_u3( x,u3,t,p, u1, u2 )
%J_REACHING_RAPID_U3 Summary of this function goes here
%   Detailed explanation goes here
    
    n_u = size(u3,2);
    if size(u1,2)~= n_u
        u = [ u1*ones(1,n_u) ; u2*ones(1, n_u); u3]; 
    else
        u = [u1;u2;u3];
    end
    fl = @(x,u,t) l_rapid_movement(x,u,t,p);
    l = fl(x,u,t);


% compute derivatives of cost
    if nargout>1
 % analytical derivatives
    l_x = [2*(x(1) - p.x_target);0] ;
    l_u = 2*p.epsilon*u3 ;
    l_xx = [ 2 0;0 0];
    l_uu = 2*p.epsilon ;
    l_ux = 0;
    end

end

