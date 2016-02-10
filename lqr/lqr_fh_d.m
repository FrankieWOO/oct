% Solve discrete Riccati equation and compute optimal gains for finite horizon LQR problem.
%
% in: 
%     A,B    - matrices defining the dynamics: x_{n+1} = A*x_n + B*u_n
%     Q,R,QT - matrices defining the cost: J = x_N^T QT x_N + \sum_0^{N-1} x_n^T Q x_n + u_n^T R u_n
%     p      - parameter struct containing:
%      .N    - number of steps to simulate
% 
% out: 
%     L      - optimal feedback gains
%     S      - optimal value
%
function [pi,L,S] = lqr_fh_d ( A, B, QT, Q, R, p )

N = p.N;

dimX = size(A,2); % state dimensionality
dimU = size(B,2); % command dimensionality

% initialise matrices
S = nan(dimX,dimX,N  ); % value
L = nan(dimU,dimX,N-1); % control gains

% solve Riccati equation
S(:,:,N) = QT; % initialise
for n=N-1:-1:1
    S (:,:,n) = Q + A'*(S(:,:,n+1) - S(:,:,n+1)*B*inv(B'*S(:,:,n+1)*B+R)*B'*S(:,:,n+1))*A;
end
% compute gains
for n=N-1:-1:1
   L (:,:,n) = inv(B'*S(:,:,n+1)*B+R)*B'*S(:,:,n+1)*A;
end

pi=@(x,n)pi_linear(x,L(:,:,n)); % policy for each time step
