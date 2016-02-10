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
function qddot = qddot_maccepa ( q, qdot, u, model )

qddot = model_maccepa ( 'maccepa_model_get_acceleration', [q;qdot], u, model );

%% add damping torque
%% TODO % migrate the below to libmaccepa once the damping model is finalised
%I = model.I;
%b = get_damping_maccepa ( x, u, model );
%acc = acc - b*x(2)/I;
%
%%function tau = get_torque_maccepa(x,u,model)
%%B     = model.lever_length;
%%C     = model.pin_displacement;
%%kappa = model.spring_constant;
%%r     = model.drum_radius;   
%%a     = u(1)-x(1); % we control psi, so calculate alpha from this and current joint angle
%%
%%tau = kappa*B*C*sin(a)*( 1 + (r*u(2)-(C-B))/sqrt(B^2+C^2+2*B*C*cos(a)) );

%> \file get_acceleration_maccepa.m
%> \author Matthew Howard (MH), matthew.howard@ed.ac.uk
%> \ingroup MACCEPA
%> \brief Matlab wrapper to libmaccepa -> maccepa_model_get_acceleration
%> \sa <a href="../m2html/MACCEPA/m-files/dynamics/models/get_acceleration_maccepa.html">m2html documentation</a>
