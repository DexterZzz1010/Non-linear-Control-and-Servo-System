function []=initthrottle(i);

% Set estimated and compensator parameters
  global TU_hat TL_hat DZPOS_hat fixwidth  

  dzpos_vec = [18.2 17.5 17.2 18.0];  % Estimated dead zone position
  DZPOS_hat=dzpos_vec(i);
  tu_vec=[-0.9 -0.9 -0.9 -1]; % Upper spring compensation
  TU_hat = tu_vec(i); 
  tl_vec=[1.1 1.2 1.1 1.1]; % Lower spring compensation
  TL_hat = tl_vec(i); 
  fixwidth = 2;  % Fix activation interval

% Initialize true parameters
  global h TR

  h= 0.01;      % Sample time
  TR = 100;       % Tracking constant
  
% ------------------  Upper Process  -------------------------
  global uphie ugame uPHI uGAM uC

  usys=zpk([1.3588+0.9721i 1.3588-0.9721i],[0.8182 0.7883+0.2621i 0.7883-0.2621i],1.3884,h);

  sssys=ssbal(ss(usys));
  [uPHI,uGAM,uC,uD]=ssdata(sssys);

% Controller design
  global uLe uLc

  uphie=[uPHI zeros(length(uC),1);-h*uC 1];
  ugame=[uGAM;0];
  uce=[uC 0];

  pc=conv([1 40],[1 30]);
  pc=conv(pc,[1 2*0.9*50 50*50]);
  sysd=c2d(tf(1,pc),h);
  [num,pd]=tfdata(sysd,'v');
  uLe=place(uphie,ugame,roots(pd));
  
  uLc = 0.1;

% State observer design
  global uK  

  ob=obsv(uPHI,uC);
  nbrunobs = length(uPHI)-rank(ob);

  pco=conv([1 50],[1 2*0.95*100 100*100]);
  sysd=c2d(tf(1,pco),h);
  [num,pdo]=tfdata(sysd,'v');
  uK=place(uPHI',uC',roots(pdo));
  uK=uK';


% -----------  Lower Process  -----------------------------------
  global lphie lgame lPHI lGAM lC

  lsys=zpk([1.4538+1.2714i 1.4538-1.2714i],[0.1470 0.7656+0.1803i 0.7656-0.1803i],0.8646,h); 

  sssys=ssbal(ss(lsys));
  [lPHI,lGAM,lC,D]=ssdata(sssys);

% Controller design
  global lLe lLc

  lphie=[lPHI zeros(length(lC),1);-h*lC 1];
  lgame=[lGAM;0];
  lce=[lC 0];

  pc=conv([1 40],[1 30]);
  pc=conv(pc,[1 2*0.7*50 50*50]);
  sysd=c2d(tf(1,pc),h);
  [num,pd]=tfdata(sysd,'v');

  lLe=place(lphie,lgame,roots(pd));

  lLc = 0.1;

% State observer design
  global lK

  ob=obsv(lPHI,lC);
  nbrunobs = length(lPHI)-rank(ob);

  lK=place(lPHI',lC',roots(pdo));
  lK=lK';


% -----  Calibration parameters ------
  global Offset Lower Convert Convert2

  Offset = -4.7181;                          % Trottle offset
  Convert_vec = [4.6470 4.8028 4.8284 4.8675]; % Scales output to 90 degrees 
  Lower_vec = [-4.7605 -4.0219 -3.9217 -3.7949]; % Makes lower limit zero degrees.
  Convert=Convert_vec(i);
  Lower=Lower_vec(i);
  Convert2 = 1.95; % Scales input


