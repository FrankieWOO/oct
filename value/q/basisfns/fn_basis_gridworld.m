% 
%
%  phi = fn_basis_gridworld ( x, u )
%
% in: 
%     x,u - state, action
%
% out: 
%     phi - basis function prediction
%
function phi = fn_basis_gridworld ( x, u )

N=size(x,2);
phi = zeros(80,N);
for n=1:N
	phin = zeros(16,5);
	phin(x(n),u(n)) = 1;
	phin = phin(:);
	phi(:,n) = phin;
end

