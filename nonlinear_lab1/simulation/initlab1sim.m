clear;

% Set estimated and compensator parameters
  global TU_hat TL_hat DZPOS_hat fixwidth
  
  DZPOS_hat = 5;     % Estimated dead zone position
  TU_hat = - 1.0;      % Upper spring compensation
  TL_hat = + 1.3;     % Lower spring compensation
  fixwidth = 2;  % Interval around dzpos for active fix

% Initialize true parameters
  global h DZPOS TU TL OVERLAP USE_FIX
  
  h= 0.001;      % Sample time
  DZPOS = 0;     % Dead zone position
  TU = - 1.0;    % Upper spring torque 
  TL = + 1.3;    % Lower spring torque
%   OVERLAP = 2;   % Dead zone width in degrees
  OVERLAP = 0; 
  dDZ = 0.1;     % Sticky area
  USE_FIX = 0;   % Internal variable
  
% ------------------  Upper Process  -------------------------
% Note: PHI in the code below corresponds to the system matrix of the discrete system

  usys=zpk([2.1806 0.4922],[0.9906 0.9689+0.0371j 0.9689-0.0371j],-0.0034,h);
  sssys=ssbal(ss(usys));
  [uPHI,uGAM,uC,uD]=ssdata(sssys);

% Controller design

  uphie=[uPHI zeros(length(uC),1);-h*uC 1];
  ugame=[uGAM;0];
  uce=[uC 0];

  Q1e=diag([1 2 2 1]);
  Q12e=[0;0;0;0];  
  Q2e=[65];
  [uLe,uLVe,uS] = lqrd(uphie,ugame,Q1e,Q2e);

  % Move the integral action state eigenvalue (last state)
     nStates=length(uce);
    [Ve,De]=eig(uphie-ugame*uLe); 
    V=[Ve(:,1:nStates-1)';zeros(1,nStates-1) 1];
    sol = [zeros(1,nStates-1) 1]';
    q = real(V\sol);
    alfa = 2e4;
    Q1efix=Q1e+alfa*q*q';
    [uLe,uLVe,uS] = lqrd(uphie,ugame,Q1efix,Q2e);

  uLc = 1;

% State observer design
  ob=obsv(uPHI,uC);
  nbrunobs = length(uPHI)-rank(ob);

  R1=diag([1 2 2]);
  R12=[0;0;0];               
  R2=[0.1];
  [uK,uKf,uKv,uP,uPf] = lqed(uPHI,uC,R1,R2);


% -----------  Lower Process  -----------------------------------
  lsys=zpk([4.1036 0.3815],[0.9697 0.9514 0.9098],-0.9410e-3,h);
  sssys=ssbal(ss(lsys));
  [lPHI,lGAM,lC,D]=ssdata(sssys);

% Controller design

  lphie=[lPHI zeros(length(lC),1);-h*lC 1];
  lgame=[lGAM;0];
  lce=[lC 0];

  Q1e=diag([1 1 1 1]);
  Q12e=[0;0;0;0];  
  Q2e=[105];
  [lLe,lLVe,lS] = lqrd(lphie,lgame,Q1e,Q2e);

  % Move the integral action state eigenvalue (last state)
     nStates=length(lce);
    [Ve,De]=eig(lphie-lgame*lLe); 
    V=[Ve(:,1:nStates-1)';zeros(1,nStates-1) 1];
    sol = [zeros(1,nStates-1) 1]';
    q = real(V\sol);
    alfa = 2e4;
    Q1efix=Q1e+alfa*q*q';
    [lLe,lLVe,lS] = lqrd(lphie,lgame,Q1efix,Q2e);

  lLc = 1;

% State observer design
  ob=obsv(lPHI,lC);
  nbrunobs = length(lPHI)-rank(ob);

  R1=diag([1 2 2]);
  R12=[0;0;0];               
  R2=[0.01];
  [lK,lKf,lKv,lP,lPf] = lqed(lPHI,lC,R1,R2);


% -----  Simulation parameters ------
  global Le Lc PHI GAM C K SimSys SimNum SimDen Offset Lower Convert
  SimSys=lsys;
  SimSysC=d2c(SimSys);
  [SimNum,SimDen]=tfdata(SimSysC,'v');

  PHI = lPHI;
  GAM = lGAM;
  C = lC;
  K = lK;

  PHIe = lphie;
  GAMe = lgame;
  Ce = lce;
  Le = lLe;
  Lc = lLc;
  LVe = lLVe;
  Kf = lKf;
  Kv = lKv;

% !VERSION=R2012a mex compensator2.c;