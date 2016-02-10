% Linear dynamics function.
%
% 	xdot = A x + B u
% 
% in: 
%    x     - state
%    u     - command
%    model - model struct, containing
%         .A,B  - model parameters
%
% out:
%    xdot - state change and derivatives
% 
function xdot = f_linear ( x, u, model )

A = model.A;
B = model.B;
xdot = A*x + B*u;


