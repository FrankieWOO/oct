% Function for evaluating the cost along a trajectory under a given cost function in discrete time
% 
% in:
%     x     - matrix containing trajectory through state space
%     u     - matrix containing trajectory of commands
%     l     - function handle to cost function (in discrete time)
%     gamma - discount factor
%
% out:
%     cost   - (discounted) cost incurred
%
function cost = evaluate_trajectory_cost_ih_d ( x, u, l, gamma )

if nargin<4
   error 'Too few parameters given'
end

[n ,N ] = size(u);
[n2,N1] = size(x);

if N1~=N+1
   error 'Bad dimensionality of x and u'
end

cost = sum((gamma.^(0:N-1)).*l(x(:,1:N),u));

