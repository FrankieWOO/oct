% Solve Riccati equation and compute optimal gains for infinite horizon linear-quadratic regulator problem in continuous time.
%
% in: 
%     A,B    - matrices defining the dynamics (dx/dt = A*x + B*u)
%     Q,R    - matrices defining the cost (J = \int_0^\infty x'*Q*x + u'*R*u dt)
% 
% out: 
%     L      - optimal feedback gains
%     S      - optimal value
%
% Note: This function uses Euler integration to simulate the Riccati equation
% backwards in time (more accuracy may be acieved with more accurate integration!)
%
function [pi,L,S] = lqr_ih (A, B, Q, R)

dimX = size(A,2); % state dimensionality
dimU = size(B,2); % command dimensionality

% initialise matrices
S    = zeros(dimX,dimX); % value
L    = zeros(dimU,dimX); % control gains
Sdot = zeros(dimX,dimX); % value derivative

% solve Riccati equation
% (numerical solution: integrate DRE until steady state solution is reached)
i=0; 
dt     = 1e-4; % step size
i_max  = 1e6; % max iterations 
ds_max = 1e-6;  % convergence threshold on S
while i<i_max
   % enforce symmetry of S
   S = .5*(S+S');
   % HACK: Integrating (1) seems to result in the wrong solution.  There are
   % two solutions to the ARE (since it is quadratic), but we want the one in
   % which S is positive definite (and symmetric). According to Stengel,
   % selecting a nonnegative definite S_\infty (e.g., S_\infty = 0) and
   % integrating the DRE should result in positive definite S, but it doesn't
   % here! For some reason, reversing the sign of the DRE as in (2) (i.e.,
   % integrating backward in time) does give a reasonable result.
   %Sdot = -(A'*S + S*A + Q - S*B*(R\(B'*S))); % (1)
   Sdot = (A'*S + S*A + Q - S*B*(R\(B'*S))); % (2)
   S    = S + dt*Sdot;
   err = max(max(abs(dt*Sdot)));
   if err<ds_max % if S has converged, break
       break;
   end
   i = i+1;
end
if i==i_max,warning('Max. iterations reached: solution could be inaccurate.'),end
% compute gains
L = R\(B'*S);

pi=@(x)pi_linear(x,L);
