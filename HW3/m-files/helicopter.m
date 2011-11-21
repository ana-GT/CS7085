
%% Helicopter Matrices

A1 = [          0                  0                  0   0.99857378005981;
                 0                  0   1.00000000000000  -0.00318221934140;
                 0                  0 -11.57049560546880  -2.54463768005371;
                 0                  0   0.43935656547546  -1.99818229675293;
                 0                  0  -2.04089546203613  -0.45899915695190;
-32.10360717773440                  0  -0.50335502624512   2.29785919189453;
  0.10216116905212  32.05783081054690  -2.34721755981445  -0.50361156463623;
 -1.91097259521484   1.71382904052734  -0.00400543212891  -0.05741119384766];

A2 = [0.05338427424431             0                  0                  0;
  0.05952465534210                  0                  0                  0;
 -0.06360262632370   0.10678052902222  -0.09491866827011   0.00710757449269;
                 0   0.01665188372135   0.01846204698086  -0.00118747074157;
 -0.73502779006958   0.01925575733185  -0.00459562242031   0.00212036073208;
                 0  -0.02121581137180  -0.02116791903973   0.01581159234047;
  0.83494758605957   0.02122657001019  -0.03787973523140   0.00035400385968;
                 0   0.01398963481188  -0.00090675335377  -0.29051351547241];

A=[A1 A2];

B=[              0                  0                  0                  0;
                  0                  0                  0                  0;
   0.12433505058289   0.08278584480286  -2.75247764587402  -0.01788876950741;
  -0.03635892271996   0.47509527206421   0.01429074257612                  0;
   0.30449151992798   0.01495801657438  -0.49651837348938  -0.20674192905426;
   0.28773546218872  -0.54450607299805  -0.01637935638428                  0;
  -0.01907348632812   0.01636743545532  -0.54453611373901   0.23484230041504;
  -4.82063293457031  -0.00038146972656                  0                 0];

C = [ 0        0         0         0         0    0.0595   0.05329  -0.9968;
     1.0        0         0         0         0         0         0        0;
       0      1.0         0         0         0         0         0        0;
       0        0         0  -0.05348       1.0         0         0        0;
       0        0       1.0         0         0         0         0        0;
       0        0         0       1.0         0         0         0       0];
   
   

% Stability
[evectA, evalA] = eig(A);

% Controllability
co = ctrb(A,B);
co_rank = rank(co);

% Observability
oo = obsv(A,C);
oo_rank = rank(oo);   
   
   %% Simulation paramaters
   dt=0.01; tf=5;
   x0=0.1.*ones(8,1);
   hatx0=[-0.0384,-0.0548,0.3060,-0.0498,-0.2128,0.3207,0.2469,-0.0459]';

   
   %% Storage vectors
   N=[]; hatN=[]; T=[]; U=[];
   
   t=0; x=x0; hatx=hatx0;
   
   %%%%%%%%%%%
   %% HERE IS WHERE YOU COMPUTE YOUR GAINS
    PolesController = [-1.7, -1.8 + i ,-1.8-i ,-2.0 + 0.5*i ,-2.0 - 0.5*i ,-2.1,-2.2 + i,-2.2-i];
    PolesObserver = 4*PolesController;
    K=place(A,B,PolesController);   % Note - this needs to be replaced!!
    L=place(A', C', PolesObserver); % Note - this needs to be replaced!!
    L = L';

K
L

   %%%%%%%%%%%
   
   %% Start the process 
   while (t<=tf);
       N=[N;norm(x)];
       hatN=[hatN;norm(hatx)];
       T=[T;t];
       
       y=C*x;
       
       %% Compute the controller
       u=-K*hatx;  
       U=[U;norm(u)];
       
       %% Update the state estimate
       dothatx=A*hatx+B*u+L*(y-C*hatx); 
       
       %% Update the dynamics and the state estimator
       x=x+dt.*(A*x+B*u);
       hatx=hatx+dt.*dothatx;
       t=t+dt;
   end;
   
   %% Plot the solutions
   figure(1);
   hold off;
   plot(T,N,'g', 'linewidth', 4);
   hold on;
   plot(T,hatN,'b:','linewidth', 4);
   grid on;
   plot([0,tf],[1,1],'r','linewidth', 4);
   xlabel('t');
   ylabel('||x||');
   legend('||x||', '||\^x||');
   title('||x|| vs t');
   print('Question5x.png', '-dpng');

   figure(2);
   hold off;
   plot(T,U,'linewidth', 4);
   grid on;
   hold on;
   plot([0,tf],[50,50],'r','linewidth', 4);
   xlabel('t');
   ylabel('||u||');
   legend('||u||');
   title('||u|| vs t');
   print('Question5u.png', '-dpng');   
   
       
   
   
   
   
   
   
