%%  Optimization problem 1: Optimized fixed damping
%   
%   ilqr on reaching problem for MACCEPA actuator with U dimension of 3.
%   u_1 and u_2 fixed to be constant. u_3 optimizaed as a parameter w.r.t rapid movement
%   
clear variables;
%% set experiment paras
u1 = pi/6 ;
u2 = pi/8 ;

%% add paths

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
N  = 100 ;        % number of time steps
t  = (0:N-1)*dt; % sample times

% simulation parameters
ps = []; ps.dt = dt; ps.N = N; ps.solver = 'euler';

model = model_maccepa('maccepa_model'); %

% dynamics

f = @(x, u) f_maccepa ( x, u, model ); % state space dynamics

% start state
x0 = zeros(2,1);

% cost/reward
pc = [];
pc.x_target   = u1 ;
pc.dt = dt ;
pc.N = 100;
pc.epsilon = 1e-8;
pc.solver = ps.solver;
j = @(u3) J_maccepa_reaching_fixed_damping ( u3, x0, f, pc, u1, u2 );

umax =   1  ;
umin =   0  ;
uu = fminbnd(j,0.01,umin,umax);

u3 = uu*ones(1,N-1);

% run controller on plant
u_final = repmat([u1;u2;uu],1,N-1);
x_final = simulate_feedforward(x0,f,u_final,ps);
% evaluate cost of trajectory on plant
cost = evaluate_trajectory_cost_fh(x_final,u_final,j,ps);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost)

% plot example trajectory
name='MACCEPA'; figure(1),set(gcf,'Name',name),set(gcf,'NumberTitle','off'),clf
subplot(2,2,1);
hold on
plot(t,x_final');
plot(t(N),pc.x_target,'o');
xlabel('t')
ylabel('x')
axis tight

subplot(2,2,2);
hold on
plot(t(1:end-1),u3');
xlabel('t')
ylabel('u3')

subplot(2,2,3);
hold on
for n=1:N-1
l(n)=l_rapid_movement(x_final(:,n), u_final(:,n), t(n));
end
l(N)=l_rapid_movement(x_final(:,N), nan, nan);
plot(t,l);
xlabel('t')
ylabel('cost')
axis tight

subplot(2,2,4);
hold on
q0=nan(1,N-1); k=nan(1,N-1);
for n=1:N-1
q0(n) = q0_maccepa(              u_final(:,n),model);
 k(n) = k_maccepa (x_final(1,n),       u_final(:,n),model);
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
