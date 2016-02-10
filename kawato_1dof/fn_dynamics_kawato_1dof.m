% State space dynamics function for the  1-DOF Kawato joint (one joint, two 'kawato model' muscles).
%
% in: 
%     x     = state [position; velocity]
%     u     = command [muscle activations]
%     model = model struct
%
% out:
%     xdot = state change
%
function xdot = f_kawato_1dof ( x, u, model )

qddot = qddot_kawato_1dof ( x(1), x(2), u, model );
xdot  = [x(2); qddot];

