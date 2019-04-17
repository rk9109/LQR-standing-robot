function draw_robot(state, R, L)
% Draw the balancing robot
%     input:   state | state vector of robot
%                      state = [phi theta phi_dot theta_dot]
%              R     | wheel radius
%              L     | length from center of wheel to center of mass
%
%     output:  plot
%

% coordinates
phi   = state(1);
theta = state(2);

% cartesian coordinates
x   = R*phi;
x_c = R*phi + L*sin(theta);
y_c = R + L*cos(theta);

% floor
plot([-20 20],[0 0],'w','LineWidth', 2) 
hold on
% wheel
rectangle('Position', [x-R 0 2*R 2*R], 'Curvature', 1, 'FaceColor', [0.8500 0.3250 0.0980])
% body
plot([x 2*x_c-x], [R 2*y_c-R], 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 4) 

% parameters
limit = 15; 
xlim([-limit limit]);
ylim([-limit limit]);

set(gca,'Color','k','XColor','w','YColor','w')

set(gcf,'Color','k')
set(gcf,'Position',[250 250 750 750])
set(gcf,'InvertHardcopy','off')   

drawnow
hold off

end

