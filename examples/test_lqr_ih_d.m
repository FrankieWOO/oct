% Demo script: Test numerical solution of infinite horizon linear-quadratic regulator problem in discrete time.

clear all;

% set up numerical simulation
dt = 0.02;       % time step
N  = 200;        % number of time steps
t  = (0:N-1)*dt; % sample times

% simulation parameters
ps = []; ps.N = N;

% start state
x0 = [1;0];

% dynamics
model.A = [ 1 dt ; ...
            0  1 ];
model.B = [ 0 ; ...
           dt ];
f = @(x, u) f_linear_d ( x, u, model );         % linear dynamics (discrete time)

% cost
pc = [];
% state cost 
pc.Q  = dt*[1  0;  0  0.1];
% command cost
pc.R  = dt*.1;
l  = @(x,u) l_quadratic ( x, u, pc ); % quadratic costs

% optimise
[pi,L,V] = lqr_ih_d(model.A,model.B,pc.Q,pc.R);

% run controller on plant
[x,u] = simulate_feedback_d(x0,f,pi,ps);

% evaluate cost of trajectory on plant
cost = evaluate_trajectory_cost_ih_d(x,u,l,1);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost)

% plot
fn=1;figure(fn),clf
plot_lq_fh(fn,'LQR-IH (discrete time)',t,x,u,[],[],'b');

