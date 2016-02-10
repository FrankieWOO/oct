% Demo script: Test ilqr on reaching problem for 6-muscle, 2-joint system with 'kawato model' muscle dynamics.

clear all;
tic

% time
dt = 0.02;       % time step
N  = 25 ;        % number of time steps
t  = (0:N-1)*dt; % sample times

% simulation parameters
ps = []; ps.dt = dt; ps.N = N; ps.solver = 'euler';

model = model_kawato_arm; %

% dynamics
umax = model.umax;
umin = model.umin;
f = @(x, u) g_kawato_arm ( x, u, model ); % kawato arm dynamics

% cost
pc = [];
pc.w   = [10000,100,0.1]; pc.w = pc.w/sum(pc.w);
pc.qt  = (pi/180)*[80; 40]; % center target shifted
pc.tau = @(x,u) tau_kawato_arm ( x(1:2,:), x(3:4,:), u, model );
j = @(x,u,t) j_reaching_task_2dof_plants ( x, u, t, pc );

% start state
x0 = (pi/180)*[45; 70 ; zeros(2,1)]; % start state

% set ilqr parameters
u0 = [zeros(3,1);ones(3,1)]; % command initialisation
%u0 = rand(6,1); % command initialisation
po = [];
po.umax = umax;
po.umin = umin;
%po.lambda_max = inf;

% optimise
[xx, uu, L] = ilqr(f,j,dt,N,x0,u0,po);

% run controller on plant
ppi = []; ppi.xn = xx; ppi.un = uu; ppi.Ln = L;
pi = @(x,n)pi_ilqr(x,n,ppi);
[x,u] = simulate_feedback_time_indexed ( x0, f, pi, ps );

% evaluate cost of trajectory on plant
cost = evaluate_trajectory_cost_fh(x,u,j,ps);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost)

% plot example trajectory
name='Kawato Arm'; figure(1),set(gcf,'Name',name),set(gcf,'NumberTitle','off'),clf
subplot(2,2,1);
hold on
plot(t,x');
plot(t(N),pc.qt','o');
xlabel('t')
ylabel('x')
axis tight

subplot(2,2,2);
hold on
plot(t(1:end-1),u');
xlabel('t')
ylabel('u')
axis tight

subplot(2,2,3);
hold on
for n=1:N-1
l(n)=j(x(:,n), u(:,n), t(n));
end
l(N)=j(x(:,N), nan, nan);
plot(t,l);
xlabel('t')
ylabel('cost')
axis tight

toc

