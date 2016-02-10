% Estimate equilibrium position of MACCEPA model
%
% in:
%      u     - command vector
%      model - struct containing model parameters
%
% out:
%      q0    - equilibrium position
%
function q0 = q0_maccepa ( u, model )

% libmaccepa calculation:
q0 = model_maccepa('maccepa_model_get_equilibrium_position',nan(2,size(u,2)),u,model);


