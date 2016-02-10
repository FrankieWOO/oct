% Calculate torque for Kawato 1-DOF model given given current joint angles, velocities and muscle activations.
% 
%  in:
%      q     - joint angles
%      qdot  - joint velocities
%      u     - muscle activations
%      model - model struct
%
%  out:
%      tau   - torque
%
function tau = tau_kawato_1dof ( q, qdot, u, model )

tau = tau_kawato_1dof_mx ( q, qdot, u, model );

