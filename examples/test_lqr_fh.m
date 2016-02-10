% Demo script: Test numerical solution of finite horizon linear-quadratic regulator problem in continuous time.

clear all;

% set up numerical simulation
dt = 0.02;       % time step
N  = 200;        % number of time steps
t  = (0:N-1)*dt; % sample times

% simulation parameters
ps = []; ps.N = N; ps.dt = dt; ps.solver = 'euler';

% start state
x0 = [1;0];

% dynamics
model = model_linear; %
f = @(x, u) f_linear ( x, u, model );

% cost
pc = [];
% terminal state cost
pc.QT = [1  0;  0  0.1];
% state cost 
pc.Q  = pc.QT;
% command cost
pc.R  = .1;
l  = @(x,u,t) l_quadratic_fh ( x, u, t, pc ); % quadratic costs

% optimise
[pi,L,V] = lqr_fh(model.A,model.B,pc.QT,pc.Q,pc.R,ps);

% run controller on plant
[x,u] = simulate_feedback_time_indexed(x0,f,pi,ps);

% evaluate cost of trajectory on plant
cost = evaluate_trajectory_cost_fh(x,u,l,ps);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost)

% plot
fn=1;figure(fn),clf
plot_lq_fh(fn,'LQR-FH',t,x,u,L,[],'g');

