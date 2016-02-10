% Calculate acceleration for Kawato 1-DOF model given current state x, motor command u and model parameters (model struct).
%
%  in:
%     q     - joint angles
%     qdot  - joint velocities
%     u     - muscle activations
%     model - model struct
%
% out:  
%      qddot - acceleration
% 
function qddot = qddot_kawato_1dof ( q, qdot, u, model )

I    = model.I; % inertia
fv   = model.viscous_friction;
tau  = tau_kawato_1dof ( q, qdot, u, model ); % actuator torque

qddot=(1/I)*(tau - fv*qdot);
