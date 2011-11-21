# Question 1.a
# addpath("/home/ana/Documents/CoursesGaTech/Fall2011/CS7085/HW2/Assignment");


printf("Question1a file definition \n");


printf(" Loading function q1a \n");
% ------------------------------------
% function q1a
% ------------------------------------
function q1a ()

tf = 80;
x0 = [ 0; 0; 0; 0; 0; 0 ];
global thetad; thetad = [ pi/8; pi/5; -pi/8 ];

vopt = odeset( "RelTol", 1e-6, "AbsTol", 1e-6, "InitialStep", 0.01, "MaxStep", 0.1 );
[t, x] = ode45( @eom1a, [0, tf], x0, vopt );

disp([ 't = ' num2str( t(end), '%6.4f')...
       ' :: theta = ( ' num2str( x(end, 1:3), ' %6.4f ' ) ...
       ' ) :: theta dot = ( ' num2str( x(end,4:6),' %6.4f ') ' )' ]);

% Plot Joint angles vs time
figure(1)
plot( t, x(:, 1:3), 'linewidth', 4 );
axis( [0, t(end), -1, 1.5 ] );
legend( '\theta_1', '\theta_2', '\theta_3', 1 );
hold on;
plot( [t(1); t(end)], [thetad, thetad], '-.' );
grid on;
hold off;
title('Joint angles vs time');
print('Question1aJoints.png', '-dpng');
replot

% Plot velocities vs time
figure(2);
plot(t, x(:,4:6), 'linewidth', 4);
hold on;
plot([t(1); t(end)], [0 0], 'k-.');
grid on;
hold off;
legend( '\theta_1', '\theta_2', '\theta_3', 1 );
title('Joint velocities vs. time');
print('Question1aVels.png', '-dpng');
replot

% Snapshots
figure(3);
disp(['Desired goal is theta_d = (' num2str(thetad','%6.4f ') ')']);
visualize(3, t, x, false);	% Set to true for movie mode.
title('Manipulator snapshots');

% Settle Time
figure(4);
errnorm = x(:,1:3) - transpose(thetad(:,ones(1,size(x,1))));
errnorm = errnorm .* errnorm;
errnorm = sqrt(sum(errnorm, 2));
errnorm = errnorm/norm(thetad,2);
istab   = find(errnorm < 0.1);
itime   = find(t > 37);
imin    = find(istab > itime(1));
disp(['Settle time to 10% is: ' num2str(t(istab(imin(1))),'%4.2f') ...
                                                              ' seconds.'] );
disp(' '); disp(' ');
plot(t, errnorm, 'linewidth', 4);
hold on;
plot( [t(1) t(end)], [0.1 0.1]);
title('Error norm vs. time');
print('Question1aErrors.png', '-dpng');
replot

endfunction

printf(" Loading function eom1a \n");
% -------------------------------- 
%  eom1a 
%
%  function xdot = eom1a(t, x)
%
% --------------------------------
function xdot = eom1a(t, x)

global l m aI thetad;

theta = x(1:3);			% Extract joint configuration.
thetadot = x(4:6);		% Extract joint velocity.

M = massmatrix(theta, l, m, aI);
C = coriolis(theta, thetadot, l, m);
N = others(theta, thetadot, l);

				% This is control torque, you fill it in.
Kp = eye(3);
Kv = 3*eye(3);

tau =  -Kp*(theta - thetad) - Kv*thetadot; % simple control, ignore manipulator dynamics.

thetaddot = inv(M)*(-C*thetadot + N + tau);

xdot = [thetadot; thetaddot];	% Joint first and second time derivatives
				%   to get state velocity for integration.
endfunction
