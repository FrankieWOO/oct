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
%    xdot,xdot_x,xdot_u - state change and derivatives
% 
function [ xdot, xdot_x, xdot_u ] = g_linear ( x, u, model )

xdot = f_linear ( x, u, model );

A = model.A;
B = model.B;
if nargout>1
xdot_x = A;
xdot_u = B;
end

