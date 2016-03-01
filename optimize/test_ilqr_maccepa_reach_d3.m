% Demo script: Test ilqr on reaching problem for MACCEPA actuator with U dimension of 3.

clear variables;

%% add path

curPath = pwd;
curPaths = strsplit(curPath,{'\','/'}) ;
fatherPath = strjoin(curPaths(1:end-1),'/') ;
addpath(genpath(fatherPath)) ;

%addpath([fatherPath,'/external/genpath_exclude']);
%addpath(genpath_exclude(fatherPath,{'/maccepa/model_maccepa_d2','/maccepa/model_maccepa_d3'}));
%addpath([fatherPath,'/maccepa/model_maccepa_d3']);
%%
tic

% time
dt = 0.02;       % time step
<<<<<<< HEAD
N  = 25 ;        % number of time steps
=======
N  = 50 ;        % number of time steps
>>>>>>> b741d17fc094c79aafd243b4ab3ae20a52e190b0
t  = (0:N-1)*dt; % sample times

% simulation parameters
ps = []; ps.dt = dt; ps.N = N; ps.solver = 'euler';

model = model_maccepa('maccepa_model'); %

% dynamics
<<<<<<< HEAD
umax = [ pi/2; pi/2; 0.1];
umin = [ -pi/2; 0; 0];
=======
umax = [ pi/6; pi/8; 1];
umin = [ pi/6; pi/8; 0];
>>>>>>> b741d17fc094c79aafd243b4ab3ae20a52e190b0
f = @(x, u) g_maccepa ( x, u, model ); % state space dynamics

% cost/reward
pc = [];
pc.x_target   = pi/6;
pc.epsilon = 10^-8;
pc.dt = dt;
j = @(x,u,t) j_reaching_rapid ( x, u, t, pc );

% start state
x0 = zeros(2,1);

% set ilqr parameters
<<<<<<< HEAD
u0s = [ linspace(0,pi/2,5) ; linspace(0,pi/4,5); linspace(0,0.1,5)] ; % command initialisation
=======
u0 = [0.5;0.3;0.1]; % command initialisation
>>>>>>> b741d17fc094c79aafd243b4ab3ae20a52e190b0
po = [];
po.umax = umax;
po.umin = umin;
po.lambda_init = 0.01;
po.lambda_max  = 0.1;

xs = zeros(2,N,5,5,5);
us = zeros(3,N-1,5,5,5);
costs = zeros(5,5,5);
% optimise
for iu=1:5
    for ju=1:5
        for ku=1:5

u0 = [ u0s(1,iu);u0s(2,ju);u0s(3,ku) ] ;
[xx, uu, L] = ilqr(f,j,dt,N,x0,u0,po);

% run controller on plant
ppi = []; ppi.xn = xx; ppi.un = uu; ppi.Ln = L;
pi = @(x,n)pi_ilqr(x,n,ppi);
[xs(:,:,iu,ju,ku),us(:,:,iu,ju,ku)] = simulate_feedback_time_indexed ( x0, f, pi, ps );

% evaluate cost of trajectory on plant
costs(iu,ju,ku) = evaluate_trajectory_cost_fh(xs(:,:,iu,ju,ku),us(:,:,iu,ju,ku),j,ps);

        end
    end
end

[cost,index_cost] = min(costs(:)) ;

[I1,I2,I3] = ind2sub(size(costs),index_cost);

x = xs(:,:,I1,I2,I3);
u = us(:,:,I1,I2,I3);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost);
%% plot example trajectory
name='MACCEPA'; figure(2),set(gcf,'Name',name),set(gcf,'NumberTitle','off'),clf
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
l = zeros(N);
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
