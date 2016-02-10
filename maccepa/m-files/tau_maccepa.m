% Calculate torque for MACCEPA model given state x and command u.
% (This is a wrapper for MaccepaModel.mex).
% 
%  in:
%      q     - joint angles
%      qdot  - joint velocities
%      u     - motor commands
%      model - model struct
%
%  out:
%      tau   - torque
%
function tau = tau_maccepa ( q, qdot, u, model )

tau = model_maccepa('maccepa_model_get_actuator_torque',[q;qdot],u,model);

%b = get_damping_maccepa([q;qdot],u,model);
%tau=tau-b*qdot;
