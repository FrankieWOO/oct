% Estimate joint stiffness of MACCEPA model
%
% in:
%      x     - state vector 
%      u     - command vector
%      model - struct containing model parameters
%
% out:
%      k     - stiffness
%
function k = k_maccepa ( q, u, model )

k = model_maccepa('maccepa_model_get_stiffness',[q;nan(size(q))],u,model);

%% analytic calculation:
%B     = model.lever_length;
%C     = model.pin_displacement;
%kappa = model.spring_constant;
%r     = model.drum_radius;   
%a     = u(1)-x(1); % we control psi, so calculate alpha from this and current joint angle
%k     = ...
%        kappa*B*C*cos(a)*(1+(r*u(2)-(C-B))/sqrt(B^2+C^2-2*B*C*cos(a)))...
%       -kappa*((B*C*sin(a))^2)*((r*u(2)-(C-B))/((B^2+C^2-2*B*C*cos(a))^(3/2)));
%
