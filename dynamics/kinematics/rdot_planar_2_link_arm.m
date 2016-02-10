% Estimate end-effector velocity of (horizontal planar) 2-link arm.
% 
%  rdot = get_endeffector_position_planar_2_link_arm ( q, qdot, model )
% 
%  in:
%      q       - joint angles
%      qdot    - joint angular velocity
%      model   - model structure
%
%  out:
%      rdot    - end-effector velocity
%
function rdot = rdot_planar_2_link_arm ( q, qdot, model )

J = J_planar_2_link_arm ( q, model );
N = size(J,3);
rdot=zeros(2,N);
for n=1:N
rdot(:,n) = J(:,:,n)*qdot(:,n);
end

