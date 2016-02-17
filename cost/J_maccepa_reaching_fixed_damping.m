function [ cost ] = J_maccepa_reaching_fixed_damping( u3, x0,f,p,u1,u2 )
%J_MACCEPA_REACHING_FIXED_DAMPING Summary of this function goes here
%   Detailed explanation goes here
    
    %t = (0:p.N-1)*p.dt;
    % u is 3*(N-1)
    u = repmat([u1;u2;u3],1,p.N-1);
    
    ps = [];
    ps.dt = p.dt;
    ps.solver = p.solver ;
    x = simulate_feedforward(x0,f,u,ps);
    
    pl = [] ;
    pl.x_target = p.x_target ;
    pl.epsilon = p.epsilon;
    pl.dt = p.dt;
    l = @(x,u,t) l_rapid_movement(x,u,t,pl);
    
     
    cost = evaluate_trajectory_cost_fh(x,u,l,ps);

end

