% Calculate torque for Kawato 1-DOF model given state x, motor command u and model parameters (model struct).
% 
%  in:
%      x     - state (pos,vel)
%      u     - (normalised) motor commands 
%      model - model structure 
%  out:
%      tau   - torque
%
function tau = get_torque_kawato_1dof_mx ( q, qdot, u, model ) %#eml

A  = model.A  ;  % moment arm matrix
gr = model.gr;  % muscle length parameters (activation constant)
lm_l0 = model.lm_l0; % lm-l0

K = km_kawato_1dof(u,model);
B = bm_kawato_1dof(u,model);

N   = size(u,2);
tau = zeros(1,N);

for n=1:N
	ldot     =    - A*qdot(:,n);
	T        = K(:,n).*(lm_l0 - A*q(:,n) + gr.*u(:,n))+B(:,n).*ldot;
	tau(:,n) = A'*T;
end

