% Demo script: Test LWLSPI solution of infinite horizon linear-quadratic regulator problem in discrete time.

clear all;
rand('seed',1),randn('seed',1)
datafile = ['D_',mfilename,'.mat'];
if 1,unix(['rm -f ',datafile]);end % remove old data if needed

% set up numerical simulation
dt = 0.02;       % time step
N  = 200;        % number of time steps
t  = (0:N-1)*dt; % sample times

% simulation parameters
ps = []; ps.N = N;

% dimensionality
dimX = 2;
dimU = 1;
dimY = dimX;

% start state
x0 = [1;0];

% dynamics
model.A = [ 1 dt ; ...
            0  1 ];
model.B = [ 0 ; ...
           dt ];
f = @(x, u) f_linear_d ( x, u, model );         % linear dynamics (discrete time)

% sensor
h = @(x)x;

% cost
pc = [];
% state cost 
pc.Q  = dt*[1  0;  0  0.1];
% command cost
pc.R  = dt*.1;
l  = @(x,u) l_quadratic ( x, u, pc ); % quadratic costs

% get training data
try   % load data, if possible
load(datafile);
catch % otherwise, regenerate
% generate training data
xmax = 1.1*ones(dimX,1); xmin=-xmax; umax = 10; umin = -umax; % range of data
Ntr = 500;                                        % no. data points
X  = nan(dimX,Ntr);for i=1:dimX,X(i,:) = rand_interval(1,Ntr,xmax(i),xmin(i));end
U  = nan(dimU,Ntr);for i=1:dimU,U(i,:) = rand_interval(1,Ntr,umax(i),umin(i));end
Xn = nan(dimX,Ntr);
RR = nan(   1,Ntr);
for n=1:Ntr
Xn(:,n)=  f(X(:,n),U(:,n));
RR(:,n)= -l(X(:,n),U(:,n));
end
Y  = h(X );
Yn = h(Xn);
D.X = X; D.Y = Y; D.R = RR; D.Xn = Xn; D.U = U; D.Yn = Yn;
% save data
save(datafile,'D');
end
% unpack data
Ntr = size(D.X,2);
fprintf(1,'#Data: %5d, ',Ntr);   

% set up LSPI parameters, functions
gamma      = 1-1e-6; % discount factor
p          = [];
p.p_pi     = [];
% set up regression model
cmax = xmax-.1*(xmax-xmin); cmin = xmin+.1*(xmax-xmin); Ngp  = 5; Nc = Ngp^dimX;
[c1,c2] = ndgrid(linspace(cmin(1),cmax(1),Ngp),linspace(cmin(2),cmax(2),Ngp)); c = [c1(:),c2(:)]';
cDs = sqrt(distances(c,c)); sortcDs = sort(cDs); s2 = sortcDs(2,1);
model.W    = @(z)fn_weight_gaussians( z(1:dimX,:), c, s2 );
model.phi  = @(z)phi_quadratic ( z );
argmaxuQ   = @(model)argmaxuQ_lwlspi(model,dimX,dimU,p.p_pi); % function for finding the argmax of the Q-function
pi0        = @(x)pi_linear(x,[1,1]);        % initial policy

% optimise
[pi, Qp, model] = lwlspi( D, pi0, model, argmaxuQ, gamma, p );

% estimate cost from computed Q function
costp = Qp(x0,pi(x0));
fprintf(1,'Predicted cost from x0    = %f\n',costp)

% run controller on plant
[x,u] = simulate_feedback_d(x0,f,pi,ps);

% evaluate cost of trajectory on plant
cost = evaluate_trajectory_cost_ih_d(x,u,l,1);
fprintf(1,'Cost (evaluated on plant) = %f\n',cost)

% evaluate error in solution (compared to analytical)
[pit,Lt,Vt] = lqr_ih_d(model.A, model.B, pc.Q, pc.R); % optimise policy numerically
[xt,ut] = simulate_feedback_d ( x0, f, pit, ps );
Nx=min(size(x,2),size(xt,2)); RMSE_x=sqrt(sum((xt(:,1:Nx)-x(:,1:Nx)).^2,2)/Nx);
Nu=min(size(u,2),size(ut,2)); RMSE_u=sqrt(sum((ut(:,1:Nu)-u(:,1:Nu)).^2,2)/Nu);
fprintf(1,'RMSE(x,x*)=[%f %f],RMSE(u,u*)=%f\n',RMSE_x,RMSE_u)

% plot
fn=2;figure(fn),clf
plot_lq_fh(fn,'LWLSPI/LQR-IH (discrete time)',t,x,u,[],[],'g');

subplot(2,4,[1 2 5 6])

% plot out data points
plot3(D.X(1,:),D.X(2,:),D.U,'x','Color',.9*ones(1,3))

% plot centres
plot3(c(1,:),c(2,:),zeros(size(c,2)),'bo')

% plot optimal V-function
Ngp=50; [X1,X2]=meshgrid(linspace(xmin(1),xmax(1),Ngp),linspace(xmin(2),xmax(2),Ngp)); 
contour(X1,X2,reshape(Qp([X1(:),X2(:)]',pi([X1(:),X2(:)]')),Ngp,Ngp))

xlim([xmin(1),xmax(1)])
ylim([xmin(2),xmax(2)])
zlim([umin(1),umax(1)])

% plot Q-function
subplot(2,4,8)
Ngp=10; [X1,X2,U]=meshgrid(linspace(xmin(1),xmax(1),Ngp),linspace(xmin(2),xmax(2),Ngp),linspace(umin(1),umax(1),Ngp)); Z=[X1(:),X2(:),U(:)]';
scatter3(Z(1,:),Z(2,:),Z(3,:),200*ones(1,size(Z,2)),Qp([Z(1,:);Z(2,:)],Z(3,:)),'.')
xlabel('x_1')
ylabel('x_2')
zlabel('u')


