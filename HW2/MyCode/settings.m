% ----------------
% settings.m
% ----------------

printf( "Setting global variables and the like \n" );

global l m aI thetad gdees poly2b tpf;

l = [ 1, 1, 1.5 ];	% Link lenghts
m = [ 1, 2, 2 ];	% Link masses
aI = [ 1/4, 1/2, 9/8 ]; % Link angular inertias

plot3R;

% -------------------------------------
% massmatrix function
%
%  M = massmatrix(theta, l, m, aI)
% -------------------------------------
function M = massmatrix( theta, l, m, aI )

M = zeros(3, 3);

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

endfunction

%--------------------------------------- 
% coriolis function
%
% C = coriolis(theta, thetadot, l, m)
% ---------------------------------------

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

endfunction

% ---------------------------------- 
% others 
%
%  N = others(theta, thetadot, l)
% -----------------------------------
function N = others(theta, thetador, l)

N = 0;

endfunction


%-----------------------------------------------
% visualize 
%
%  function visualize(fignum, t, x, asMovie)
% -----------------------------------------------
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
    plot3R(theta, l,'clear');
    drawnow;
    pause(0.05);
  end
  hold off;
  title('Snapshots of manipulator motion.');
end

endfunction

