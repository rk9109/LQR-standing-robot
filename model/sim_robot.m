clear, close all
clc

% Model constants
LENGTH = 4;      % length
RADIUS = 2;      % wheel radius
MASS_WHEEL = 5;  % wheel mass
MASS_BODY = 15;  % body mass

% Simulation constants
TIMESTEP = 0.1;  % timestep size
TIMESPAN = 25;   % simulation time

% Model parameters
states = {'phi' 'theta' 'phi_dot' 'theta_dot'};
inputs = {'u'};
outputs = {'phi' 'theta' 'phi_dot' 'theta_dot'};

% Matrices
[A, B, C, D] = getmatrices(LENGTH, RADIUS, MASS_WHEEL, MASS_BODY);

% Controllability
Co = ctrb(A,B);
disp('Rank of controllability matrix: ')
disp(rank(Co))

%% Open-loop System

% System poles
poles = eig(A);
disp('Open-loop poles: ')
disp(poles)

% Create state-space model
sys_open = ss(A, B, C, D, 'statename', states, 'inputname', inputs, 'outputname', outputs);

%% Closed-loop System
% Using LQR controller

Q = [1 0 0 0 ;
     0 5 0 0 ;
     0 0 1 0 ;
     0 0 0 5];
 
R = 1;

% Determine LQR gain
K = lqr(A, B, Q, R);
disp('LQR Gain: ')
disp(K)

% Update A matrix
A = A - B*K;

% System poles
poles = eig(A);
disp('Closed-loop poles: ')
disp(poles)

% Create state-space model
sys_closed = ss(A, B, C, D, 'statename', states, 'inputname', inputs, 'outputname', outputs);

%% Simulate
t = 0 : TIMESTEP : TIMESPAN;

u       = zeros(size(t));           % zero disturbance
u_noise = normrnd(0, 10, size(t));  % N(0, 10) gaussian disturbance

% Initial state   
x0 = [0; 0.1; 0; 0];                      

[y_open, t, ~]   = lsim(sys_open, u, t, x0);
[y_closed, t, ~] = lsim(sys_closed, u, t, x0);

%% Plot

figure(1)
yyaxis left  % plot theta
plot(t, y_closed(:, 2))
ylim([-0.5 0.5])
ylabel('$\theta$', 'Interpreter', 'latex')

yyaxis right % plot x
plot(t, RADIUS*y_closed(:, 1))
ylim([-10 10])
ylabel('$x$', 'Interpreter', 'latex')

title('Response with LQR Control')


%% Animate

% Draw closed loop system
figure(2)
for k=1:length(t)
    draw_robot(y_closed(k,:), RADIUS, LENGTH);
end





