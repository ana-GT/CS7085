%============================= planarR3_fblin ============================
%
%  function planarR3_fblin
%
%
%  Simulates the equations of motion for a manipulator with three revolute
%  joints.  The initial conditions and equations of motion vary according
%  to the problem being solved. 
%
%============================= planarR3_fblin ============================

%
%  Name:	planarR3_fblin.m
%
%  Author:	Patricio A. Vela, pvela@mail.gatech.edu
%
%  Created:	11/29/2005
%  Modified:	12/06/2006
%
%============================= planarR3_fblin ============================
function planarR3_fblin

global l m aI thetad gdes poly2b tpf;
l = [1 1 1.5];			% Link lengths for plotting manipulator.
m  = [1 , 2, 2];		% Link masses.
aI = [1/4, 1/2, 9/8];		% Link angular inertias.

%----- Problem 1a -----
tf = 80;			% Final time, you fill it in.
x0 = [0,0,0,0,0,0];		% Initial condition, you fill it in.
thetad = [pi/8; pi/5; -pi/8];	% Desired joint configuration.

[t, x] = ode45(@eom1a, [0, tf], x0');
disp(['t = ' num2str(t(end),'%6.4f') ...
      ' :: theta = (' num2str(x(end,1:3),'%6.4f ') ...
      ') :: theta dot = ('  num2str(x(end,4:6),'%6.4f ') ')']);

figure(1);
plot(t, x(:,1:3));
axis([0 t(end) -1 1.5]);
legend('joint 1','joint 2','joint 3',1);
hold on;
plot([t(1); t(end)], [thetad , thetad], '-.');
hold off;
title('Joint angles vs. time');

figure(2);
plot(t, x(:,4:6));
hold on;
plot([t(1); t(end)], [0 0], 'k-.');
hold off;
legend('joint 1','joint 2','joint 3',1);
title('Joint velocities vs. time');

disp(['Desired goal is theta_d = (' num2str(thetad','%6.4f ') ')']);
visualize(3, t, x, false);	% Set to true for movie mode.
title('Manipulator snapshots');

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
plot(t, errnorm, [t(1) t(end)], [0.1 0.1]);
title('Error norm vs. time');

%return;			% Uncomment to just do part 1a.

%----- Problem 1b -----
tf = 20;			% Final time, you fill it in.
x0 = [0,0,0,0,0,0];		% Initial condition, you fill it in.

[t, x] = ode45(@eom1b, [0, tf], x0');
disp(['t = ' num2str(t(end),'%6.4f') ...
      ' :: theta = (' num2str(x(end,1:3),'%6.4f ') ...
      ') :: theta dot = ('  num2str(x(end,4:6),'%6.4f ') ')']);

figure(5);
plot(t, x(:,1:3));
axis([0 t(end) -1 1.5]);
legend('joint 1','joint 2','joint 3',1);
hold on;
plot([t(1); t(end)], [thetad , thetad], '-.');
hold off;
title('Joint angles vs. time');

figure(6);
plot(t, x(:,4:6));
hold on;
plot([t(1); t(end)], [0 0], 'k-.');
hold off;
legend('joint 1','joint 2','joint 3',1);
title('Joint velocities vs. time');

disp(['Desired goal is theta_d = (' num2str(thetad','%6.4f ') ')']);
visualize(7, t, x, false);	% Set to true for movie mode.
title('Manipulator snapshots');

figure(8);
errnorm = x(:,1:3) - transpose(thetad(:,ones(1,size(x,1))));
errnorm = errnorm .* errnorm;
errnorm = sqrt(sum(errnorm, 2));
errnorm = errnorm/norm(thetad,2);
istab   = find(errnorm < 0.1);
itime   = find(t > 4);
imin    = find(istab > itime(1));
disp(['Settle time to 10% is: ' num2str(t(istab(imin(1))),'%4.2f') ...
                                                              ' seconds.'] );
disp(' '); disp(' ');
plot(t, errnorm, [t(1) t(end)], [0.1 0.1]);
title('Error norm vs. time');

%return;			% Uncomment to just do part 1a and 1b.

%----- Problem 1c -----
tf = 20;			% Final time, you fill it in.
x0 = [0,0,0,0,0,0];		% Initial condition, you fill it in.

[t, x] = ode45(@eom1c, [0, tf], x0');
disp(['t = ' num2str(t(end),'%6.4f') ...
      ' :: theta = (' num2str(x(end,1:3),'%6.4f ') ...
      ') :: theta dot = ('  num2str(x(end,4:6),'%6.4f ') ')']);

figure(10);
plot(t, x(:,1:3));
axis([0 t(end) -1 1.5]);
legend('joint 1','joint 2','joint 3',1);
hold on;
plot([t(1); t(end)], [thetad , thetad], '-.');
hold off;
title('Joint angles vs. time');

figure(11);
plot(t, x(:,4:6));
hold on;
plot([t(1); t(end)], [0 0], 'k-.');
hold off;
legend('joint 1','joint 2','joint 3',1);
title('Joint velocities vs. time');

disp(['Desired goal is theta_d = (' num2str(thetad','%6.4f ') ')']);
visualize(12, t, x, false);	% Set to true for movie mode.
title('Manipulator snapshots');

figure(13);
errnorm = x(:,1:3) - transpose(thetad(:,ones(1,size(x,1))));
errnorm = errnorm .* errnorm;
errnorm = sqrt(sum(errnorm, 2));
errnorm = errnorm/norm(thetad,2);
istab   = find(errnorm < 0.1);
itime   = find(t > 4);
imin    = find(istab > itime(1));
disp(['Settle time to 10% is: ' num2str(t(istab(imin(1))),'%4.2f') ...
                                                              ' seconds.'] );
disp(' '); disp(' ');
plot(t, errnorm, [t(1) t(end)], [0.1 0.1]);
title('Error norm vs. time');


return;

%================================= extras ================================


%======================== Differential Equations =======================

%-------------------------------- eom1a --------------------------------
%
%  function xdot = eom1a(t, x)
%
function xdot = eom1a(t, x)

global l m aI thetad;

theta = x(1:3);			% Extract joint configuration.
thetadot = x(4:6);		% Extract joint velocity.

M = massmatrix(theta, l, m, aI);
C = coriolis(theta, thetadot, l, m);
N = others(theta, thetadot, l);

				% This is control torque, you fill it in.
Kp = 
Kv = 
tau =  % simple control, ignore manipulator dynamics.

thetaddot = inv(M)*(-C*thetadot + N + tau);

xdot = [thetadot; thetaddot];	% Joint first and second time derivatives
				%   to get state velocity for integration.

%-------------------------------- eom1b --------------------------------
%
%  function xdot = eom1b(t, x)
%
function xdot = eom1b(t, x)

global l m aI thetad;

theta = x(1:3);			% Extract joint configuration.
thetadot = x(4:6);		% Extract joint velocity.

M = massmatrix(theta, l, m, aI);
C = coriolis(theta, thetadot, l, m);
N = others(theta, thetadot, l);

				% This is control torque, you fill it in.
appM = % Approximate M here.
appC = zeros(size(C)); 		% Approximate C is zero.
Kp = 
Kv = 
tau = % Approximate feedback linearization here. 

thetaddot = inv(M)*(-C*thetadot + N + tau);

xdot = [thetadot; thetaddot];	% Joint first and second time derivatives
				%   to get state velocity for integration.

%-------------------------------- eom1c --------------------------------
%
%  function xdot = eom1c(t, x)
%
function xdot = eom1c(t, x)

global l m aI thetad;

theta = x(1:3);			% Extract joint configuration.
thetadot = x(4:6);		% Extract joint velocity.

M = massmatrix(theta, l, m, aI);
C = coriolis(theta, thetadot, l, m);
N = others(theta, thetadot, l);

				% This is control torque, you fill it in.
Kp = 
Kv = 
tau = % Full on feedback linearization feedback control here.

thetaddot = inv(M)*(-C*thetadot + N + tau);

xdot = [thetadot; thetaddot];	% Joint first and second time derivatives
				%   to get state velocity for integration.

%==================== Equations of Motion Components ===================

%------------------------------ massmatrix -----------------------------
%
%  M = massmatrix(theta, l, m, aI)
%
function M = massmatrix(theta, l, m, aI)

M = zeros(3, 3);

%--Recall that first index is row, second index is column.
c2 = cos(theta(2));
c3 = cos(theta(3));
c23 = cos(theta(2)+theta(3));

M(1,1) = 0.25*m(1)*l(1)^2 + aI(1) ...
           + m(2)*(l(1)^2+0.25*l(2)^2) + m(2)*l(1)*l(2)*c2 + aI(2) ...
	   + m(3)*(l(1)^2 + l(2)^2 + (9/25)*l(3)^2) ...
	     + 2*m(3)*l(1)*l(2)*c2 + (6/5)*m(3)*l(1)*l(3)*c23 ...
	     + (6/5)*m(3)*l(2)*l(3)*c3 + aI(3);
M(2,1) = 0.5*m(2)*l(2)*(l(1)*c2 + 0.5*l(2)) + aI(2) ...
           + m(3)*(l(2)^2 + (9/25)*l(3)^2) + m(3)*l(1)*l(2)*c2 ...
	     + (3/5)*m(3)*l(1)*l(3)*c23 + (6/5)*m(3)*l(2)*l(3)*c3 + aI(3);
M(3,1) = (9/25)*m(3)*l(3)^2 + (3/5)*m(3)*l(1)*l(3)*c23 ...
           + (3/5)*m(3)*l(2)*l(3)*c3 + aI(3);

M(1,2) = M(2,1);
M(2,2) = 0.25*m(2)*l(2)^2 + aI(2) ...
           + m(3)*(l(2)^2 + (9/25)*l(3)^2) + (6/5)*m(3)*l(2)*l(3)*c3 + aI(3);
M(3,2) = (9/25)*m(3)*l(3)^2 + (3/5)*m(3)*l(2)*l(3)*c3 + aI(3);

M(1,3) = M(3,1);
M(2,3) = M(3,2);
M(3,3) = (9/25)*m(3)*l(3)^2 + aI(3);

%------------------------------- coriolis ------------------------------
%
%  C = coriolis(theta, thetadot, l, m)
%
function C = coriolis(theta, thetadot, l, m)
Cij1 = zeros(3,3);
Cij2 = zeros(3,3);
Cij3 = zeros(3,3);

s2 = sin(theta(2));
s3 = sin(theta(3));
s23 = sin(theta(2)+theta(3));

cterm1 = m(2)*l(1)*l(2)*s2;
cterm2 = 2*m(3)*l(1)*l(2)*s2;
cterm3 = (6/5)*m(3)*l(1)*l(3)*s23;
cterm4 = (6/5)*m(3)*l(2)*l(3)*s3;

%----- Do non-zero terms of Cij1. -----
Cij1(2,1) =  cterm1+cterm2+cterm3;
Cij1(3,1) =  cterm3+cterm4;
Cij1(1,2) = -cterm1-cterm2-cterm3;
Cij1(3,2) =  cterm4;
Cij1(1,3) = -cterm3-cterm4;
Cij1(2,3) = -cterm4;

%----- Do non-zero terms of Cij2. -----
Cij2(1,1) = -cterm1 - cterm2 - cterm3;
Cij2(3,1) =  cterm4;
Cij2(1,2) = -cterm1 - cterm2 - cterm3;
Cij2(3,2) =  cterm4;
Cij2(1,3) = -cterm3 - cterm4;
Cij2(2,3) = -cterm4;

%----- Do non-zero terms of Cij3. -----
Cij3(1,1) = -cterm3 - cterm4;
Cij3(2,1) = -cterm4;
Cij3(1,2) = -cterm3 - cterm4;
Cij3(2,2) = -cterm4;
Cij3(1,3) = -cterm3 - cterm4;
Cij3(2,3) = -cterm4;

C = 0.5*(Cij1*thetadot(1) + Cij2*thetadot(2) + Cij3*thetadot(3));

%-------------------------------- others -------------------------------
%
%  N = others(theta, thetadot, l)
%
function N = others(theta, thetador, l)

N = 0;


%=========================== Output Functions ==========================

%------------------------------ visualize ------------------------------
%
%  function visualize(fignum, t, x, asMovie)
%
function visualize(fignum, t, x, asMovie)
global l;

figure(fignum);
tf = t(end);
if (asMovie)			% Plot manipulator as a movie.
  clf; hold off;
  tstep = 0.25;
  for time=0:tstep:tf
    theta = interp1(t,x,time);
    plot3R(theta, l);
    drawnow;
    title(['Time - ' num2str(time,'%5.2f')]);
    pause(0.05);
  end
else				% Plot snapshots of manipulator in time.
  clf; hold on;
  nsnaps = 10;
  for time=0:(tf/nsnaps):tf
    theta = interp1(t,x,time);
    plot3R(theta, l,'noclear');
    drawnow;
    pause(0.05);
  end
  hold off;
  title('Snapshots of manipulator motion.');
end

%
%============================= planarR3_fblin ============================
