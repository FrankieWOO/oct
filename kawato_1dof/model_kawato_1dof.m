% Kawato 1-DOF model (2 muscles, 1 joint). Returns a struct containing model parameters.
% Parameters are roughly set to model the human wrist, but these are guesstimates and not based on any measurment.
%
%  out:
%      model.L           - Link length
%           .I           - Inertia
%           .M           - Mass
%           .L_g1        - Centre of gravity
%           .umax,umin   - Max/min. commands
%           .dt          - sample time for control
%           .robot       - Matlab Robotics Toolbox model
%           .q0          - Joint angle at rest position (u=0)
%           .A           - Moment arms
%           .gk          - Muscle elasticity coefficients
%           .k0          - Initial muscle elasticity
%           .gb          - Muscle viscosity coefficients
%           .b0          - Initial muscle viscosity 
%           .gr          - difference with rest length when maximally contracted (assuming u_max=ones)
%           .lm_l0       - Constant for standard posture
%
function model = model_kawato_1dof

model.name = 'Kawato 1-DOF';

model.dimQ = 1; % number of joints
model.dimU = 2; % number of actuators (commands) 


% Model geometry and dynamics parameters.
model.L = 0.19;                       % link length (~19cm)
model.M = .5;                         % mass (~500g)
model.I  = model.M*(model.L/2)^2;      % inertia ~ mass * (length/2)^2
model.viscous_friction = .005;

% muscle parameter vectors
model.A     =  [0.025;-0.025];  % constant moment model matrix
                                % (sign sensitive)
                                % flexor +, extensor -
model.gk   = 1621.6*ones(2,1); % elasticity coefficients
model.k0   =  810.8*ones(2,1); % initial elasticity
model.gb   =  108.1*ones(2,1); % viscosity coefficients
model.b0   =   54.1*ones(2,1); % initial viscosity 
model.gr   =   0.05*[1;1];    % activation constant (both positive)

% In the muscle model, only (lm-l0) matters, and 
% actual independent values of lm and l0 do not matter
% independent definition of lm and l0 is removed
model.lm_l0 = [0;0];

% joint angle at rest position (u=0)
% q0 = inv(A'*K(0)*A)*A'*K(0)(lm-l0)
A  = model.A;
K0 = diag(model.k0);
model.q0 = inv(A'*K0*A)*A'*K0*(model.lm_l0);

% model ranges

% command range
model.umax =  ones(model.dimU,1); % maximum motor command
model.umin = zeros(model.dimU,1); % minimum motor command

% joint range
model.qmax =  pi/2;
model.qmin = -pi/2;

