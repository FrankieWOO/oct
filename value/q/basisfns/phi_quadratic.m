% Quadratic basis function.
%
%  phi = fn_basis_quadratic ( x )
%
% Implements quadratic basis function:
%
%  phi(x) = [ x1^2; x2^2; ...; x1x2; ...; x1; x2; ... 1 ]
%
% in: 
%     x   - input
%
% out: 
%     phi - basis function prediction
%
% Note: This uses the outer product to compute the basis elements, 
%       in matrix form. These are then selected from the upper 
%       triangle of the matrix. There may be a more efficient way
%       to do this, avoiding the uncessary computation of the lower
%       triangular part, and the find() operation.
%
% TODO % Return offset element, but include projection matrix
% in argmaxQ_fn_basis_quadratic.m
function phi = fn_basis_quadratic ( x )

%N = size(x,2);
%for n=1:N
%xxT = x(:,n)*x(:,n)';
%phi(:,n) = [diag(xxT); xxT(find(~tril(ones(size(xxT))))); x(:,n); 1];
%end

N = size(x,2);
for n=1:N
xxT = x(:,n)*x(:,n)';
phi(:,n) = [diag(xxT); xxT(find(~tril(ones(size(xxT))))); x(:,n); 1];
end

%function phi = fn_basis_quadratic ( x, u )
%
%z = [x;u]
%N = size(z,2);
%for n=1:N
%zzT = z(:,n)*z(:,n)';
%phi(:,n) = [diag(zzT); zzT(find(~tril(ones(size(zzT))))); z(:,n); 1];
%end
%
