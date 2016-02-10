% Demo script: Test LSPI solution of infinite horizon gridworld problem.

clear all;
rand('seed',1);randn('seed',1)
tic

N  = 7;        % number of time steps

% set up numerical simulation
s = []; s.N = N;

% start state
x0 = 11;

% dynamics
f = @(x,u)f_gridworld(x,u);

% cost
l = @(x,u)l_gridworld(x,u);

% observer
h = @(x)x;

% get training data
Ntr = 1000;                                       % no. data points
for n=1:Ntr
xr = randperm(16); xr = xr(1);
ur = randperm(5);  ur = ur(1);
D.X (:,n)= xr;
D.U (:,n)= ur;
D.Xn(:,n)=  f(D.X(  n),D.U(  n));
D.R (:,n)= -l(D.X(  n),D.U(  n)); % reward is negative cost
end
D.Y = h(D.X );
D.Yn= h(D.Xn);

% set up LSPI parameters, functions
model.phi  = @(z)fn_basis_gridworld(z(1,:),z(2,:));              % basis functions
fnArgmaxuQ = @(theta)argmaxuQ_fn_basis_gridworld(theta); % function for finding the argmax of the Q-function
theta = randperm(80); pi0=argmaxuQ_fn_basis_gridworld(theta); % function for finding the argmax of the Q-function
gamma = .99;
p          = [];

% optimise
[pi, Q, model] = lspi( D, pi0, model, fnArgmaxuQ, gamma, p );

% run controller on plant
[x,u] = simulate_feedback_d ( x0, f, pi, s );

% evaluate cost of trajectory on plant
cost = evaluate_trajectory_cost_ih_d(x,u,l,gamma);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost)

% plot
fn=1;figure(fn),clf
subplot(1,2,1)
hold on
r = get_coords_gridworld(x);
plot(r(1,:),r(2,:),'r')
plot(r(1,end),r(2,end),'r.')
xlim([0 4]),ylim([0 4])
[X,Y]=meshgrid(linspace(0,4,5),linspace(0,4,5));
for i=1:5
plot(X(:,i),Y(:,i),'k')
plot(X(i,:),Y(i,:),'k')
end
for i=1:16,r=get_coords_gridworld(i);text(r(1),r(2),num2str(i));end
axis square,axis off
% visualise policy
X=zeros(2,16); U=zeros(2,16);
for x=1:16, X(:,x)=get_coords_gridworld(x); U(:,x)=.5*get_action_vector_gridworld(pi(x)); end
quiver(X(1,:),X(2,:),U(1,:),U(2,:),0,'k');

% plot optimal V-function
subplot(1,2,2)
V=zeros(16,1);for x=1:16,V(x)=Q(x,pi(x));end,
k=1;for i=4:-1:1,for j=1:4,VV(i,j)=V(k);k=k+1;end,end
imagesc(VV); colorbar
axis square,axis off


