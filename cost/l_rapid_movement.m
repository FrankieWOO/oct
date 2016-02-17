function [ c ] = l_rapid_movement( x, u, t, p )
% The cost function to encode the object of rapid, accurate reaching to
% some target.
% Input:
% x - position sequence
% u - command sequence
% t - time sequence
% p - parameters: x_target, T, epsilon
% p.x_target - target point
% p.T - the maximium time allowed
% p.epsilon - a small constant, by default 10^-8 in experiments
% p.dt - time interval
% c = /int_0^T (x(t) - x_target)^2 + epsilon (u^T * u) dt

    c1 = (x(1,:) - p.x_target).^2;
    if isnan(t)
        %final cost
        c = c1 *p.dt;
    else
        % running cost
        c =  c1 + sum(u.^2,1) * p.epsilon ;
    end
end
