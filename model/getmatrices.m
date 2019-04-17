function [A, B, C, D] = getmatrices(L, R, M_w, M_b)
% Return state-space matrices for balancing robot
%    input:   L    | length
%             R    | wheel radius
%             M_w  | wheel mass
%             M_b  | body mass
% 
%    output:  [A B C D] | state-space matrices
%                         x_dot = Ax + Bu
%                         y     = Cx + Du
%

% Constants
g = 9.8;    % acceleration of gravity

B_y = 0.1;  % rolling damping ratio
B_m = 0.1;  % friction damping ratio

I_w = (1/2)*M_w*R^2;     % wheel moment of inertia
I_b = (1/3)*M_b*(2*L)^2; % body moment of inertia

% intermediate matrices
E = [I_w+(M_w+M_b)*R^2   M_b*R*L     ;  
     M_b*R*L             I_b+M_b*L^2];

F = [B_y+B_m   -B_m ;
     -B_m       B_m];

G = [0   0      ;
     0   -M_b*g*L];

H = [1; -1];

% state-space matrices
A_11 = [0 0; 0 0];
A_21 = [1 0; 0 1];
A_12 = -E\G;
A_22 = -E\F;
A = [A_11 A_21; A_12 A_22];

B_1 = [0; 0];
B_2 = -E\H;
B = [B_1; B_2];

C = eye(4);

D = 0;

end

