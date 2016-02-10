% Calculate acceleration for MACCEPA
%
%  in:
%     q     - joint angles
%     qdot  - joint velocities
%     u     - command [desired motor positions in radiens]
%     model - model struct
%
% out:  
%       acc           - acceleration
% 
function qddot = qddot_ideal_contact ( q, qdot, u, model )

%tau = tau_maccepa ( q, qdot, u, model )

m    = model.m;
g    = model.g;
l    = model.l;

% actuator torque

Fx = Fx_ideal_contact(q,qdot,model);

tau   = tau_maccepa_ideal_contact(u,q,qdot);
rdotx = rdotx_ideal_contact(q,qdot,model);
rho   = rho_ideal_contact(q,model);

qddot = 1/(m*l^2).*( tau + (m*g + Fx).*l.*sin(q));

end 

%> \file get_acceleration_ideal_contact.m
%> \author Matthew Howard (MH), matthew.howard@ed.ac.uk
%> \ingroup MACCEPA
%> \brief Matlab wrapper to libmaccepa -> maccepa_model_get_acceleration
%> \sa <a href="../m2html/MACCEPA/m-files/dynamics/models/get_acceleration_ideal_contact.html">m2html documentation</a>
