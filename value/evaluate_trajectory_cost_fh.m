% Function for evaluating the cost along a trajectory under a given cost function
% 
% in:
%     x     - matrix containing trajectory through state space
%     u     - matrix containing trajectory of commands
%     l     - function handle to cost function
%     p     - simulation struct
%
% out:
%     cost   - cost incurred
%
function cost = evaluate_trajectory_cost_fh ( x, u, l, p )

if nargin<4
   error 'Too few parameters given'
end

[n ,N ] = size(u);
[n2,N1] = size(x);

if N1~=N+1
   error 'Bad dimensionality of x and u'
end

dt = p.dt;
t  = (0:N)*dt;

% integrate running cost & add final cost
cost = dt*sum(l(x(:,1:end-1),u,t)) + l(x(:,end),nan,nan);

