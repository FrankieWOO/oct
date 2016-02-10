% Function for computing the muscle damping for the Kawato arm model.
%
%  in:
%      x      - state (pos,vel)
%      u      - motor commands 
%      model  - model structure
%  out:
%      bm     - muscle damping
%
function bm = get_muscle_damping_kawato_1dof(u,model) %#eml

N  = size(u,2);
b0 = model.b0;  % muscle damping parameters
gb = model.gb;  %
bm = repmat(b0,1,N) + repmat(gb,1,N).*u;

%> @file get_muscle_damping_kawato_arm.m
%> @author Matthew Howard (MH), matthew.howard@ed.ac.uk
%> @date 11/07/11 17:50:47
