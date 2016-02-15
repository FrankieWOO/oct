% Function for solving (finite horizon) optimal control problems with the Iterative Linear Quadratic Regulator method.
%
%    [x, u, L, cost] = ilqr(f, j, dt, N, x0, u0, p )
%
% in:
%     f  - dynamics function (function handle)
%     j  - cost function (function handle)
%     dt - time step
%     N  - number of steps
%     x0 - start state
%     u0 - initial command sequence
%     p  - parameter struct (optionally) containing:
%      .lambda_init     - initial value of Levenberg-Marquardt lambda
%      .lambda_factor   - factor for multiplying or dividing lambda
%      .lambda_max      - threshold on lambda (exit if lambda exceeds this value)
%      .iter_max        - threshold on number of iterations (exit if exceeded)
%      .dcost_converge  - threshold on relative improvement in cost (exit if improvement less than this value)
%      .online_printing - print: {0:never, 1:every iter, 2:final}
%      .online_plotting - plot:  {0:never, 1:every iter, 2:final}
%      .umax            - maximum command 
%      .umin            - minimum command 
%      .solver          - simulation solver
%      (any fields that are unspecified will be set by default values).
%
% out:
%     x      - predicted optimal state sequence
%     u      - optimal feed-forward command sequence
%     L      - optimal feedback gain sequence
%     cost   - predicted cost
%
%> \author Matthew Howard (MH) matthew.howard@ed.ac.uk
%> \date 19/06/11 19:14:08
%
function [x, u, L, cost] = ilqr(f, j, dt, N, x0, u0, p )

% unpack parameter struct
if isfield(p,'lambda_init'        ),lambda_init         = p.lambda_init        ;else lambda_init        = 1e-2    ;end
if isfield(p,'lambda_factor'      ),lambda_factor       = p.lambda_factor      ;else lambda_factor      = sqrt(10);end
if isfield(p,'lambda_max'         ),lambda_max          = p.lambda_max         ;else lambda_max         = 1e-2    ;end
if isfield(p,'dcost_converge'     ),dcost_converge      = p.dcost_converge     ;else dcost_converge     = 1e-9    ;end
if isfield(p,'iter_max'           ),iter_max            = p.iter_max           ;else iter_max           = 100     ;end
if isfield(p,'online_printing'    ),online_printing     = p.online_printing    ;else online_printing    = 1       ;end
if isfield(p,'online_plotting'    ),online_plotting     = p.online_plotting    ;else online_plotting    = 1       ;end
if isfield(p,'umax'               ),umax                = p.umax               ;else umax               = inf     ;end
if isfield(p,'umin'               ),umin                = p.umin               ;else umin               =-inf     ;end
if isfield(p,'solver'             ),solver              = p.solver             ;else solver             ='euler'  ;end

% initialise everything
dimX = size(x0, 1);   % state dimensionality
dimU = size(u0, 1);   % command dimensionality
ps=[];ps.dt=dt;ps.solver=solver; % simulation parameters

if size(u0,2)==1,                                   % initial command sequence
u = repmat(u0,1,N-1);                               % ...if one initial command given, use as full initial command sequence
else                                               %
u    = u0;
end                                       % ...if full command sequence given, use as initial command sequence
L    = zeros(dimU, dimX, N-1);                      % initial feedback gain sequence
x    = simulate_feedforward ( x0, f, u, ps );       % initial state sequence
cost = evaluate_trajectory_cost_fh ( x, u, j, ps ); % initial cost
t    = (0:N-1)*dt;                                  % time 

% initialise other matrices (for speed)
A  = zeros(dimX,dimX,N-1); 
B  = zeros(dimX,dimU,N-1); 
I  =   eye(dimX         );
q0 = zeros(        1,N-1); 
q  = zeros(     dimX,N-1); 
Q  = zeros(dimX,dimX,N-1); 
r  = zeros(     dimU,N-1); 
R  = zeros(dimU,dimU,N-1); 
P  = zeros(dimU,dimX,N-1); 
s0 = zeros(        1,N  ); 
s  = zeros(     dimX,N  ); 
S  = zeros(dimX,dimX,N  ); 
l  = zeros(     dimU,N-1); 

% initialise lambda
lambda = lambda_init;
% initialise update flag
update = 1;

% set up plotting
if online_plotting
online_plotting_fig=figure;clf,set(online_plotting_fig,'Name','iLQR: Cost and Lambda Convergence'),set(online_plotting_fig,'NumberTitle','off')
subplot(1,2,1),hold on,grid on,set(gca,'Yscale','linear'),title('cost'  ),ylabel('cost'  ),xlabel('iteration')
subplot(1,2,2),hold on,grid on,set(gca,'Yscale','log')   ,title('lambda'),ylabel('lambda'),xlabel('iteration')
end

% main loop
for iter = 1:iter_max
    %------ STEP 1: approximate dynamics and cost along new trajectory'
    if update,
        update = 0;
		% compute LQ approximation
        for n = 1:N-1
            % linearize dynamics, adjust for dt
            [ff, f_x, f_u] = f(x(:,n), u(:,n));
            A(:,:,n) = I + dt*f_x;
            B(:,:,n) =     dt*f_u;

            % quadratize cost, adjust for dt
            [l0,l_x,l_xx,l_u,l_uu,l_ux] = j(x(:,n), u(:,n), t(n));
            q0(    n) = dt*l0;
            q (  :,n) = dt*l_x;
            Q (:,:,n) = dt*l_xx;
            r (  :,n) = dt*l_u;
            R (:,:,n) = dt*l_uu;
            P (:,:,n) = dt*l_ux;
        end

        % initialise value function approximation with final cost
        [s0(N),s(:,N),S(:,:,N)] = j(x(:,N), nan, nan);
    end
    
    %------ STEP 2: compute optimal control law and cost-to-go
	diverged=0;
    for n = N-1:-1:1
        % compute shortcuts g,G,H
        g = r(  :,n) + B(:,:,n)'*s(  :,n+1); 
        G = P(:,:,n) + B(:,:,n)'*S(:,:,n+1)*A(:,:,n);
        H = R(:,:,n) + B(:,:,n)'*S(:,:,n+1)*B(:,:,n);

		% check for divergence
		if any(~isfinite(H)), diverged=1; break; end % divergence: EXIT 

        % find control law
        [l(:,n), L(:,:,n)] = u_optimal(g,G,H,u(:,n),umin,umax,lambda);

        % update cost-to-go approximation
        S (:,:,n) = Q (:,:,n) + A(:,:,n)'*S (:,:,n+1)*A(:,:,n) +    L(:,:,n)'*H*L(:,:,n) + L(:,:,n)'*G + G'*L(:,:,n);
        s (  :,n) = q (  :,n) + A(:,:,n)'*s (  :,n+1)          +    L(:,:,n)'*H*l(  :,n) + L(:,:,n)'*g + G'*l(:,n);
        s0(    n) = q0(    n) +           s0(    n+1)          + .5*l(  :,n)'*H*l(  :,n) + l(  :,n)'*g;

    end
	if diverged, fprintf('Optimal control law and cost-to-go computation diverged. '); break; end% divergence: EXIT 

    %------ STEP 3A: new control sequence, trajectory, cost
    % simulate linearized system to compute new control
    dx = zeros(dimX,1);
    for n=1:N-1
        du = l(:,n) + L(:,:,n)*dx;   
        du = min(max(du+u(:,n),umin),umax) - u(:,n);
        dx = A(:,:,n)*dx + B(:,:,n)*du ;
        unew(:,n) = u(:,n) + du;
    end

    %------ STEP 3B: simulate system to compute new trajectory and cost
    xnew    = simulate_feedforward ( x0, f, unew, ps );
    costnew = evaluate_trajectory_cost_fh ( xnew, unew, j, ps );% initial cost

    %------ STEP 4: Levenberg-Marquardt method
    if costnew<cost,
        % decrease lambda (get closer to Newton method)
        lambda = lambda / lambda_factor;
		dcost  = cost-costnew; % decrease in cost
        % update x, u, cost
        u    = unew;
        x    = xnew;
		cost = costnew;
		% check for convergence
        if iter>1 && dcost<dcost_converge,
            fprintf('Cost improvement threshold reached. '); break; % improvement too small: EXIT
        end
		% flag update
        update = 1;
    else
        % increase lambda (get closer to gradient descent)
        lambda = lambda * lambda_factor;
        if lambda>lambda_max,
            fprintf('Max. lambda reached. '); break; % lambda too large: EXIT
        end
    end

	% plot/print stuff out
    if online_printing==1,fprintf('Iteration = %d; Cost = %.4f; log(Lambda) = %.1f\n',iter,cost,log10(lambda));end
	if online_plotting==1,
		lambdas(iter)=lambda; costs(iter)=cost;
		set(0,'CurrentFigure',online_plotting_fig),
		subplot(1,2,1),plot([0:iter-1],costs);  
		subplot(1,2,2),plot([0:iter-1],lambdas);
	end
end
if iter==iter_max
	fprintf('Max. number of iterations reached. ');
end
% print final result if necessary
if online_printing==2,
    fprintf('Iterations = %d;  Cost = %.4f', iter, cost);
end
fprintf('\n');
% close figure
if online_plotting==1,close(online_plotting_fig),end

% compute optimal control law
function [l,L] = u_optimal(g, G, H, u, umin, umax, lambda)

% eigenvalue decomposition, modify eigenvalues
[V,D] = eig(H);
d = diag(D);
d(d<0) = 0;
d = d + lambda;

% inverse modified Hessian, unconstrained control law
H1 = V*diag(1./d)*V';
l = -H1*g;
L = -H1*G;

% enforce constraints
l = min(max(l+u,umin),umax) - u;
L((l+u==umin)|(l+u==umax),:) = 0;

