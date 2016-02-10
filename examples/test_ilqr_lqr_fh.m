% Demo script: Test solution of finite horizon linear-quadratic regulator problem in continuous time using ILQR.

clear all;

% set up numerical simulation
dt = 0.02;       % time step
N  = 200;        % number of time steps
t  = (0:N-1)*dt; % sample times

% simulation parameters
ps = []; ps.dt = dt; ps.N = N; ps.solver = 'euler';

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

% set ilqr parameters
u0 = 1; % command initialisation
po = [];
po.lambda_init = 1e-3;
% function handles including derivatives for ilqr
g  = @(x, u) g_linear        ( x, u, model ); % linear dynamics
j  = @(x,u,t) j_quadratic_fh ( x, u, t, pc ); % quadratic costs

% optimise
[xx, uu, L] = ilqr(g,j,dt,N,x0,u0,po);

% run controller on plant
ppi = []; ppi.xn = xx; ppi.un = uu; ppi.Ln = L;
pi = @(x,n)pi_ilqr(x,n,ppi);
[x,u] = simulate_feedback_time_indexed (x0,f,pi,ps);

% evaluate cost of trajectory on plant
cost = evaluate_trajectory_cost_fh(x,u,l,ps);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost)

% evaluate error in solution (compared to numerical)
A = model.A; B = model.B; QT = pc.QT; Q = pc.Q; R = pc.R;
[pit,Lt,Vt] = lqr_fh(model.A,model.B,pc.QT,pc.Q,pc.R,ps);
[xt,ut] = simulate_feedback_time_indexed ( x0, f, pit, ps );
RMSE_x=sqrt(sum((xt-x).^2,2)/N);
RMSE_u=sqrt(sum((ut-u).^2,2)/N);
RMSE_L=sqrt(sum((squeeze(Lt)-squeeze(-L)).^2,2)/N);
fprintf(1,'RMSE(x,x*)=[%f %f],RMSE(u,u*)=%f,RMSE(L,L*)=[%f %f]\n',RMSE_x(1),RMSE_x(2),RMSE_u,RMSE_L)

% plot
fn=1;figure(fn),clf
plot_lq_fh(fn,'iLQR',t,x,u,-L,[],'b');

