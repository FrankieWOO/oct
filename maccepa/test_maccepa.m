clear all

% set up simulation
dt = .02;              % control frequency
N  = 400;              % no. samples
t  = (0:N-1)*dt;       % sample times
x0 = zeros(2,1);       % start state

% set up dynamics function
m  = model_maccepa('maccepa_model'); % model parameters
f = @(x, u) f_maccepa  ( x, u, m ); % dynamics function

% simulation parameter struct
p = [];
p.dt = dt;
p.solver = 'rk4';

% define feed forward commands
umax=.5*m.umax;
w=100*2*pi/N; % frequency
u1=[umax(1)*sin(w*t);
       zeros(1,N);zeros(1,N)]; 

w=200*2*pi/N; % frequency
t=(0:N-1)*dt; % time
u2=[zeros(1,N); .5*umax(2)*(sin( w*t - pi/2 )+1);zeros(1,N)];

u3=u1+u2;

U = [u1,u2,u3]; % concatenate the three phases

% simulate
X = simulate_feedforward ( x0, f, U, p );

% plot joint angles, velocities, commands
subplot(3,1,1),plot(X(1,:)'),ylabel('q (rad)')
subplot(3,1,2),plot(X(2,:)'),ylabel('dq/dt (rad/s)')
subplot(3,1,3),plot(U'),ylabel('u'),xlabel('t (s)')
