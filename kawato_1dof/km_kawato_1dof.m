% Function for computing the muscle stiffnesses for the Kawato arm model.
%
%  in:
%      x      - state (pos,vel)
%      u      - motor commands 
%      model  - model structure
%  out:
%      km     - muscle stiffnesses
%
function K = km_kawato_1dof(u,model) %#eml

N  = size(u,2);
k0 = model.k0;  % muscle stiffness parameters
gk = model.gk;  %
K  = repmat(k0,1,N) + repmat(gk,1,N).*u;

%> @file get_muscle_stiffness_kawato_1dof.m
%> @author Matthew Howard (MH), matthew.howard@ed.ac.uk
%> @date 11/07/11 17:50:47
