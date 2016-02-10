% Solve Riccati equation and compute optimal gains for finite horizon linear-quadratic regulator problem in continuous time.
%
% in: 
%     A,B    - matrices defining the dynamics (dx/dt = A*x + B*u)
%     Q,R,QT - matrices defining the cost (J = x'*QT*x + \int_0^T x'*Q*x + u'*R*u dt)
%     T      - final time
%     p      - parameter struct containing:
%      .N    - number of steps to simulate
%      .dt   - time step

% out: 
%     L      - optimal feedback gains
%     S      - optimal value
%
% Note: This function uses Euler integration to simulate the Riccati equation
% backwards in time (more accuracy may be acieved with more accurate integration!)
%
function [pi,L,S] = lqr_fh (A, B, QT, Q, R, p)

dt= p.dt;
N = p.N;

dimX = size(A,2); % state dimensionality
dimU = size(B,2); % command dimensionality

% initialise matrices
S = nan(dimX,dimX,N  ); % value
L = nan(dimU,dimX,N-1); % control gains

% solve Riccati equation
S(:,:,N) = QT; % initialise
for n=N:-1:2
    S (:,:,n-1) = S (:,:,n) + dt*(A'*S(:,:,n) + S(:,:,n)*A + Q - S(:,:,n)*B*inv(R)*B'*S(:,:,n)');
end
% compute gains
for n=1:N-1
	L (:,:,n  ) = R\(B'*S(:,:,n));
end

pi=@(x,n)pi_linear(x,L(:,:,n)); % policy for each time step
