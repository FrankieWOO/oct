% Cost function for the 4x4 gridworld
%
% in: 
%    x   - state (integer in [1,2,...,16])
%    u   - command (integer in [1,2,...,5])
%
% out:
%    l   - cost 
% 
function l = fn_cost_gridworld(x,u)

j = [-10,ones(1,4);ones(15,5)];
l = diag(j(x,u))'; % diag, to return costs for multiple x, u combinations
