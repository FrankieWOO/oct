% Function for calculating the cost in the reaching task for the 1-DOF plants (e.g., MACCEPA, Edinburgh VSA, etc.)
%
%   l = l_contact_task_1dof_plants ( x, u, t, p )
%
% Implements cost function of the form:
%
%   J = w1 (q(T)-q*)^2 + w2 qdot(T)^2 + \int_0^T w3 tau^2 dt
% 
% in:
%    x, u, t  - state, command, time
%    q        - joint angle
%    qdot     - joint velocity 
%    p        - parameter struct, containing
%     .w      - weight vector [w1,w2,w3]'
%     .qt     - target in joint space
%     .m      - mass
%     .g      - gravitational field strength
%     .tau    - actuator torque (function handle)
%     .k1     - spring constant of damping surface
%     .rho0   - intial distance between fixed point and unprobed surface 
%    rho    - distance between fixed point on Maccepa and probed surface
% out:
%    l        - cost
%

function l = l_contact_task_1dof_plants ( x, u, t, p )

Fx_desired = repmat(p.Fx_desired,1,size(x,2));
w          = p.w;
model      = p.model;

q    = x(1,:);
qdot = x(2,:);

Fx = Fx_ideal_contact(q,qdot,model);

rdotx = rdotx_ideal_contact(q,qdot,model);
rho   = rho_ideal_contact(q,model);

% compute cost
if isnan(t)
	% final cost
	l = 0;
else
	% running cost
	l = (Fx-Fx_desired).^2 + w*sum(u.^2,1);
end

