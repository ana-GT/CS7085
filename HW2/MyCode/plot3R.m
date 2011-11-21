%================================= plot3R ================================
%
%  function plot3R(theta, links, options)
%
%
%  Plot the 3R manipulator for the given joint configuration and
%  link lengths.
%
%  Inputs:
%    theta	- The three revolute joint angles.
%    links	- The three link lengths.
%
%  Options are:
%    'clear'	- Clears the current figure prior to drawing (default).
%    'noclear'	- Do not clear current figure prior to drawing.
%
%================================= plot3R ================================

printf("Loading plot3R \n");

function plot3R(theta, links, varargin)

g1 = [[cos(theta(1)) , -sin(theta(1)), 0]; ...
      [sin(theta(1)) ,  cos(theta(1)), 0]; [0, 0, 1]];

g2 = [[cos(theta(2)) , -sin(theta(2)), links(1)]; ...
      [sin(theta(2)) ,  cos(theta(2)), 0]; [0, 0, 1]];

g3 = [[cos(theta(3)) , -sin(theta(3)), links(2)]; ...
      [sin(theta(3)) ,  cos(theta(3)), 0]; [0, 0, 1]];

g4 = [[1, 0, 0.95*links(3)]; [0, 1, 0]; [0, 0, 1]];

delta = links(3)*0.1;

if (nargin < 3)
  varargin = {'clear'};
end
for i =1:length(varargin)
  switch varargin{i}
    case 'clear'
      clf;
  end
end

hold on;

g12 = g1*g2;
g123 = g12*g3;
g1234 = g123*g4;

g41 = g1234*[[1, 0, delta]; [0, 1, 0.5*delta]; [0, 0, 1]];
g42 = g1234*[[1, 0, 0]; [0, 1, 0.5*delta]; [0, 0, 1]];
g43 = g1234*[[1, 0, 0]; [0, 1, -0.5*delta]; [0, 0, 1]];
g44 = g1234*[[1, 0, delta]; [0, 1, -0.5*delta]; [0, 0, 1]];
gripper = [ g41(1,3), g41(2, 3) ; g42(1,3), g42(2, 3) ; ...
            g43(1,3), g43(2, 3) ; g44(1,3), g44(2, 3) ];

lim = sum(links);
axis(1.2*[-lim, lim, -lim, lim]);
t = [0:.1:2*pi+0.1];
plot(lim*cos(t), lim*sin(t), 'c:');

plot(g1(1,3), g1(2,3),'ro');
plot(g1(1,3), g1(2,3),'k+');
plot(g12(1,3), g12(2,3),'ro');
plot(g123(1,3), g123(2,3),'ro');
line([g1(1,3) g12(1,3)], [g1(2,3) g12(2,3)]); 
line([g12(1,3) g123(1,3)], [g12(2,3) g123(2,3)]); 
line([g123(1,3) g1234(1,3)], [g123(2,3) g1234(2,3)]); 
line(gripper(:,1), gripper(:,2));

axis equal;
hold off;

endfunction

