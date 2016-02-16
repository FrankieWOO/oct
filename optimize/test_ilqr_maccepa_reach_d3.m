% Demo script: Test ilqr on reaching problem for MACCEPA actuator with U dimension of 3.

clear all;

%% add path
curPath = pwd;
curPaths = strsplit(curPath,{'\','/'});
fatherPath = strjoin(curPaths(1:end-1),'/');
addpath([fatherPath,'/external/genpath_exclude']);

addpath(genpath_exclude(fatherPath,{'/maccepa/model_maccepa_d2','/maccepa/model_maccepa_d3'}));

addpath([fatherPath,'/maccepa/model_maccepa_d3']);
%%
tic

% time
dt = 0.02;       % time step
N  = 100 ;        % number of time steps
t  = (0:N-1)*dt; % sample times

% simulation parameters
ps = []; ps.dt = dt; ps.N = N; ps.solver = 'euler';

model = model_maccepa('maccepa_model'); %

% dynamics
umax = [ pi/2; pi/2; 1];
umin = [ -pi/2; 0; 0];
f = @(x, u) g_maccepa ( x, u, model ); % state space dynamics

% cost/reward
pc = [];
pc.x_target   = pi/4;
pc.epsilon = 10^-8;
j = @(x,u,t) j_reaching_rapid ( x, u, t, pc );

% start state
x0 = zeros(2,1);

% set ilqr parameters
u0 = [pi/4;pi/8;0.01]; % command initialisation
po = [];
po.umax = umax;
po.umin = umin;

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
name='MACCEPA'; figure(1),set(gcf,'Name',name),set(gcf,'NumberTitle','off'),clf
subplot(2,2,1);
hold on
plot(t,x');
plot(t(N),pc.x_target,'o');
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

subplot(2,2,4);
hold on
q0=nan(1,N-1); k=nan(1,N-1);
for n=1:N-1
q0(n) = q0_maccepa(              u(:,n),model);
 k(n) = k_maccepa (x(1,n),       u(:,n),model);
end
h(1)=plot(t(1:end-1),q0,'b');
h(2)=plot(t(1:end-1), k,'r');
xlabel('t')
axis tight
legend(h,'q_0','k','Location','Best')

%name='Profiles'; figure(sum(double(name))),set(gcf,'Name',name),set(gcf,'NumberTitle','off'),clf
%for n=1:N-1
%xdot(:,n)=fnDyn(x(:,n),u(:,n));
%tau (:,n)=fnTorque(x(:,n),u(:,n));
%end
%subplot(5,1,1),hold on,ylabel('pos.'  ),plot(t         ,x(1,:)),plot(t(1:end-1),q0,'--'),legend('q','q_0','Location','Best'),plot(t(N),qt,'o');
%subplot(5,1,2),hold on,ylabel('vel.'  ),plot(t         ,x(2,:)) 
%subplot(5,1,3),hold on,ylabel('acc.'  ),plot(t(1:end-1),xdot(2,:))
%subplot(5,1,4),hold on,ylabel('tor.'  ),plot(t(1:end-1),tau)
%subplot(5,1,5),hold on,ylabel('stiff.'),plot(t(1:end-1),k),%ylim([umin(2),umax(2)])
%xlabel('t')

toc
%%
rmpath(genpath(fatherPath))
