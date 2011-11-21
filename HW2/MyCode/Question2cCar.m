%================================== car ==================================
%
%  Simulate the stabilization of the car-like vehicle using the partial
%  feedback linearization trick of Reza Olfati-Saber.
%
%================================== car ==================================
% addpath("/home/ana/Documents/CoursesGaTech/Fall2011/CS7085/HW2/MyCode");
printf("Loading car.m \n");

function car

tspan = [0, 15];
x0 = [2.3;0.2;pi/2; 0.02];			% Try out other initial conditions.
%vopt = odeset( "RelTol", 1e-6, "AbsTol", 1e-6, "InitialStep", 0.01, "MaxStep", 0.05 );
[t, x] = ode45(@eomcar, tspan, x0);


figure(1);
  plot(x(:,1), x(:,2), 'linewidth', 4 );
  axis equal;
  title('XY vs time');
  print('Question2aXyt.png', '-dpng');
  replot

figure(2);
  plot(t, x(:,3), 'linewidth', 4 );
  title('\theta vs time');
  print('Question2aThetat.png', '-dpng');
  replot

figure(3);
  plot(t, x(:,4), 'linewidth', 4 );
  title('\lambda vs time');
  print('Question2aLambdat.png', '-dpng');
  replot

% Plot positions x and y vs time
figure(4)
plot( t, x(:, 1:2), 'linewidth', 4 );
legend( 'x','y', 1 );
grid on
hold off;
title('X and Y vs t');
print('Question2aParam.png', '-dpng');
replot

%ge = SE2([0;0],0);
%for ii = 1:length(t)
%  g = SE2(x(ii,1:2),x(ii,3));
%  figure(5);
%    plot(x(:,1),x(:,2));
%    hold on;
%      plotSE2(ge,[],'r',[1 2]);
%      plotSE2(g);
%    hold off;
%    axis([-2, 5, -2, 5]);
%  drawnow;
%  pause(0.02);
%end

endfunction

%=========================== Extra functions =============================

% -------------
% zdest
% -------------
function pos = xdest(t)
r = 2.0;
w = 1.0;
pos = r*cos(w*t);
endfunction

function pos = ydest(t)
r = 2.0;
w = 1.0;
pos = r*sin(w*t);
endfunction


%-------------------------------- eomcar -------------------------------
%
%  Equations of motion, with control, of the car-like vehicle.
%
function xdot = eomcar(t, x)

xdot = zeros(size(x));

epsilon = 0.01;			% Or pick something else if you'd like.
%R = % Rotation matrix here.
R = [cos(x(3)), -sin(x(3)); sin(x(3)), cos(x(3))];
% Intermediate code????
Rlambda = [cos(x(3)), -x(4)*sin(x(3)); sin(x(3)), x(4)*cos(x(3))];
e1 = [1;0];
z = [x(1);x(2)] + x(4)*R*e1;
K = [1,0;0,1]*1;
vu = -K*( z - [ xdest(t); ydest(t) ] );
c = 1;
xdot(4) = -c*(x(4) - epsilon);
u = inverse(Rlambda)*vu - e1*xdot(4);

%u = % What?  Fill it out.  You may want to define some intermediate variables.

xdot(1:2) = R*[1;0]*u(1);
xdot(3)   = u(2);
endfunction


function g = SE2(d, theta)

if (nargin == 0)
  g.d = [0;0];
  theta = 0;
elseif ( (size(d,1) == 2) && (size(d,2) == 1) )
  g.d = d;
elseif ( (size(d,1) == 1) && (size(d,2) == 2) )
  g.d = d';
else
  error('The translation vector has incorrect dimensions');
end

g.R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

endfunction



%================================== plot =================================
%
%  function plot(g, label, linecolor, sc)
%
%  Plots the coordinate frame associated to g.  The figure is cleared, 
%  so this will clear any existing graphic in the figure.  To plot on
%  top of an existing figure, set hold to on.
%
%  Inputs:
%    g		- The SE2 coordinate frame to plot.
%    label	- The label to assign the frame.
%    linecolor  - The line color to use for plotting.  (See `help plot`) 
%
%  Output:
%    The coordinate frame, and possibly a label,  is plotted.
%
%================================== plot =================================
function plotSE2(g, flabel, lcol, sc)

if ( (nargin < 2) )
  flabel = '';
end

if ( (nargin < 3) || isempty(lcol) )
  lcol = 'b';
end

if ( (nargin < 4) || isempty(sc) )
  sc = [1 1];
elseif (size(sc,2) == 1)
  sc = [sc 2];
end

o = g.d;

x = g.R*[sc(1);0];
y = g.R*[0;sc(1)];

isheld = ishold;

plot(o(1)+[0 x(1)],o(2) + [0 x(2)],lcol);
hold on;
plot(o(1)+[0 y(1)],o(2) + [0 y(2)],lcol);
plot(o(1), o(2), [lcol 'o'],'MarkerSize',7);

if (~isempty(flabel))
  text(o(1) - sc(2)*(x(1)+y(1))/4, o(2) - sc(2)*(x(2)+y(2))/4, flabel);
end

if (~isheld)
 hold off;
end

axis equal;
endfunction
