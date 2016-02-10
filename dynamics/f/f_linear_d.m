% Linear dynamics function in discrete time
%
% 	x_{n+1} = A x_n + B u_n
% 
% in: 
%    x   - state
%    u   - command
%    A,B - model parameters
%
% out:
%    x   - next state
% 
function x = f_linear_d ( x, u, model )

A = model.A;
B = model.B;

x = A*x + B*u;

