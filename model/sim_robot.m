clear, close all
clc

% % Model constants
% Model constants
LENGTH = 2;      % length
RADIUS = 1;      % wheel radius
MASS_WHEEL = 1;  % wheel mass
MASS_BODY = 4;   % body mass

% Simulation constants
TIMESTEP = 0.1;  % timestep size
TIMESPAN = 15;   % simulation time

% Model parameters
states =  {'phi' 'theta' 'phi_dot' 'theta_dot'};
inputs =  {'u'};
outputs = {'x' 'theta' 'x_dot' 'theta_dot'};

% Matrices
[A, B, C, D, E] = getmatrices(LENGTH, RADIUS, MASS_WHEEL, MASS_BODY);

% Controllability
Co = ctrb(A,B);
disp('Rank of controllability matrix: ')
disp(rank(Co))

%% Open-loop System

% Create state-space model
sys_open = dss(A, B, C, D, E,...
               'statename', states, 'inputname', inputs, 'outputname', outputs);

% System poles
disp('Open-loop poles: ')
disp(pole(sys_open))

%% Closed-loop System
% Using LQR controller

Q = [10   0   0   0 ;
     0    10   0   0 ;
     0    0   1   0 ;
     0    0   0   1];
 
R = 1;

% Determine LQR gain
K = lqr(sys_open, Q, R);
disp('LQR Gain: ')
disp(K)

% Update A matrix
Acl = A - B*K;

% Create state-space model
sys_closed = dss(Acl, B, C, D, E,...
                 'statename', states, 'inputname', inputs, 'outputname', outputs);

% System poles
disp('Closed-loop poles: ')
disp(pole(sys_closed))

%% Simulate
t = 0 : TIMESTEP : TIMESPAN;

u         = zeros(size(t));            % zero disturbance
u_noise   = normrnd(0, 0.1, size(t));  % N(mean, var) gaussian disturbance

% Initial state   
x0 = [1; 1; 0; 0];

[y_closed, t, ~] = lsim(sys_closed, u, t, x0);

%% Plot

figure(1)
yyaxis left  % plot theta
plot(t, y_closed(:, 2))
ylim([-pi pi])
ylabel('$\theta$', 'Interpreter', 'latex')

yyaxis right % plot x
plot(t, y_closed(:, 1))
ylim([-10 10])
ylabel('$x$', 'Interpreter', 'latex')

title('Response with LQR Control')

%% Animate

% Draw closed loop system
figure(2)
for k=1:length(t)
    draw_robot(y_closed(k,:), RADIUS, LENGTH);
end



