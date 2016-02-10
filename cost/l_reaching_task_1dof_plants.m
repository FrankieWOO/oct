% Function for calculating the cost in the reaching task for the 1-DOF plants (e.g., MACCEPA, Edinburgh VSA, etc.)
%
%   l = l_reaching_task_1dof_plants ( x, u, t, p )
%
% Implements cost function of the form:
%
%   J = w1 (q(T)-q*)^2 + w2 qdot(T)^2 + \int_0^T w3 tau^2 dt
% 
% in:
%    x, u, t  - state, command, time
%    p        - parameter struct, containing
%     .w      - weight vector [w1,w2,w3]'
%     .qt     - target in joint space
%     .tau    - actuator torque (function handle)
%
% out:
%    l        - cost
%
function l = l_reaching_task_1dof_plants ( x, u, t, p )

w = p.w;
qt = p.qt;

% compute cost
if isnan(t)
	% final cost
	l = w(1)*(x(1)-qt)^2 + w(2)*x(2)^2;
else
	% running cost
	l = w(3)*p.tau(x,u).^2;
end

