% Solve Riccati equation and compute optimal gains for infinite horizon linear-quadratic regulator problem in discrete time.
%
% in: 
%     A,B    - matrices defining the dynamics (x_{n+1} = A*x_n + B*u_n)
%     Q,R    - matrices defining the cost (J = \sum_0^\infty x_n^T Q x_n + u_n^T R u_n)
% 
% out: 
%     L      - optimal feedback gains
%     S      - optimal value
%
function [pi,L,S] = lqr_ih_d(A, B, Q, R)

dimX = size(A,2); % state dimensionality
dimU = size(B,2); % command dimensionality

% initialise matrices
S = nan(dimX,dimX); % value
L = nan(dimU,dimX); % control gains

% solve Riccati equation
%S = Q;
S = rand(dimX); % initialise Q (however you like)
err = inf; iter = 0;
while err>1e-6 && iter < 1000
	iter = iter + 1; Sp = S; 
	S = Q + A'*(S - S*B*inv(B'*S*B+R)*B'*S)*A;
	err = max(max(abs(S-Sp)));
end
% compute gains
L = inv(B'*S*B+R)*B'*S*A;

pi=@(x)pi_linear(x,L);
