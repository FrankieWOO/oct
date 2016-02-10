% Demo script: Test numerical solution of finite horizon linear-quadratic regulator problem in discrete time.

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
% terminal state cost
pc.QT = [1  0;  0  0.1];
% state cost 
pc.Q  = dt*pc.QT;
% command cost
pc.R  = dt*.1;
l  = @(x,u,t) l_quadratic_fh ( x, u, t, pc ); % quadratic costs

% optimise
[pi,L,V] = lqr_fh_d(model.A,model.B,pc.QT,pc.Q,pc.R,ps);

% run controller on plant
[x,u] = simulate_feedback_time_indexed_d(x0,f,pi,ps);

% evaluate cost of trajectory on plant
cost = evaluate_trajectory_cost_fh_d(x,u,l,ps);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost)

% plot
fn=1;figure(fn),clf
plot_lq_fh(fn,'LQR-FH (discrete time)',t,x,u,L,[],'r');

