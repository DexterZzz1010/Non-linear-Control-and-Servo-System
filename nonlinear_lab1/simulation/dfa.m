%  ____  initlab3sim has to be run before this file. ---

%Initialization
  y=[];      % Empty result vector

% Constants
  global DZPOS DZPOS_hat TU_hat TL_hat
  h = 0.001;

% Calculations
  offset = DZPOS_hat - DZPOS;  % Offset
  A = [0.01:0.005:abs(offset)+1];      % Amplitude vector  
  for i = 1:length(A)
    if abs(A(i)) > abs(offset)
      
% **************  Fill in expressions here *************    
%        phiF = asin(abs(offset)/A(i));
%        b1 = 2/pi * (TU_hat - TL_hat)*(1-sqrt(1-abs(offset)/A(i)));
   phiF = asin(abs(offset)/A(i));
   b1 = 2/pi * (TU_hat - TL_hat)*(1 - sqrt(1 - (abs(offset)/A(i))^2));
% ******************************************************
    
    else

% **************  Fill in expressions here *************
       b1 = 2/pi *(TU_hat - TL_hat);
% ******************************************************    
    
    end
    y(i)=-A(i)/b1;          % y = -1/N(A)
  end

% Create system from output of nonlin. to input of nonlin.

% Controller
% The throttlers way
  [Lx,Ly,Phic,Gamy,Gamyr,Cc,Dy,Dyr] = lqgd(PHIe,GAMe,Ce,Le,LVe,Lc,[K;-h],[Kf;0],[Kv;0]);
  [cnum,cden] = ss2tf(Phic,Gamy,Cc,Dy);

% The Bo Lincoln way
  %PHIe(4,1)=0;
  %Phic = PHIe-GAMe*Le-[K;0]*Ce;
  %Gamy = [K;-h];
  %Cc = Le;
  %Dy = 0;
  %[cnum,cden] = ss2tf(Phic,Gamy,Cc,Dy);

% Gf=P/(1+PC)
  Ps=SimSys;
  Cs=tf(cnum,cden,h);
  GSYSD = Ps / (1+Ps*Cs); 
  [RE, IM, W ] = nyquist(GSYSD,logspace(-2,3,500));

% Plot Nyquist curve for the system with controller
% (Click on curve for information)
figure(1);                      
nyquist(GSYSD, logspace(-2,3,500));
title('Nyquist plot of system with controller, click on curve for info.')
grid on;

% Plot Nyquist curve and -1/N(A) in the same figure
% The index, i, for the intersection gives the amplitude from A(i)
figure(2);   
for i = 1:length(RE)
  re(i)=RE(:,:,i);
  im(i)=IM(:,:,i);
end
hold on;
plot(complex(re,im));
plot(conj(complex(re,im)));
plot(complex(y,0),'r');
xlabel('Real axis');
ylabel('Imaginary axis');
title('Nyquist, blue, and -1/N(A), red.')
zoom on;
grid on;

% Plot -1/N(A) as a function of A
figure(3);
plot(A,y);
title('-1/N(A) as a function of A')
xlabel('A, the amplitude, [deg]')
ylabel('-1/N(A)')
grid on;
zoom on;





