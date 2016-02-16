%  State space dynamics function for the MACCEPA.
%
% in: 
%     x     - state [position; velocity]
%     u     - command [desired motor positions in radiens]
%     model - model struct
%
% out:
%     xdot - state change
%
function xdot = f_maccepa ( x, u, model )

qddot = qddot_maccepa ( x(1,:), x(2,:), u, model );
xdot  = [x(2,:); qddot];
end

