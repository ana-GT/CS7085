# Question 2
# addpath("/home/ana/Documents/CoursesGaTech/Fall2011/CS7085/HW2/Assignment");


printf("Question2 file definition \n");
global K;

printf(" Loading function q2 \n");
% ------------------------------------
% function q2
% ------------------------------------
function q2 ()

tf = 8;
%s0 = [ -1; 2.5; pi/2; 0.2 ];
s0 = [ -1; 2.5; 0; 0.2 ];
global K; K = [1,0; 0, 1];

vopt = odeset( "RelTol", 1e-6, "AbsTol", 1e-6, "InitialStep", 0.01, "MaxStep", 0.1 );
[t,s] = ode45( @car, [0, tf], s0, vopt );

disp([ 't = ' num2str( t(end), '%6.4f')...
       ' :: x = ' num2str( s(end, 1), ' %6.4f ' ) ...
       ' :: y = ' num2str( s(end, 2), ' %6.4f ' ) ...
       ' :: theta = ' num2str( s(end, 3), ' %6.4f ' ) ]);

% Plot positions x and y vs time
figure(1)
plot( t, s(:, 1:2), 'linewidth', 4 );
legend( 'x','y', 1 );
grid on
hold off;
title('XY vs time');
print('Question2aXyt.png', '-dpng');
replot


% Plot orientation vs time
figure(2)
plot( t, s(:, 3 ), 'linewidth', 4 );
grid on
hold off;
title('\theta vs time');
print('Question2aThetat.png', '-dpng');
replot

% Plot Trajectory
figure(3)
plot( s(:, 1), s(:, 2), 'linewidth', 4 );
legend( 'xy trajectory', 1 );
xlabel('x');
ylabel('y');
grid on
hold off;
title('Trajectory');
print('Question2aTrajectory.png', '-dpng');
replot

endfunction


% ----------------------------
% car
% ODE for the car movement
% ----------------------------
function sdot = car( t, s )

global K;

c = 1; e = 0.01;
 
x = s(1); 
y = s(2);
theta = s(3); 
lambda = s(4);

A = [ 0; 0; 0; -c*(lambda - e) ];
B = [ cos(theta), 0; sin(theta), 0; 0, 1; 0, 0 ];

Rlambda = [cos(theta), -lambda*sin(theta); sin(theta), lambda*cos(theta)];

R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
e1 = [1; 0];
z = [x;y] + lambda*R*e1;
vu = -K*z;
lambdadot = A(4);

u = inv(Rlambda)*vu - e1*lambdadot;

sdot = A + B*u;

endfunction
