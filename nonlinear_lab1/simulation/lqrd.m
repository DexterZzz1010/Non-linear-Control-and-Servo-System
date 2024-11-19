function [L,Lv,S] = lqrd(Phi,Gam,Q1,Q2,Q12)
% LQRD	Linear quadratic regulator design for discrete-time systems
%
%	[L,Lv,S] = lqrd(Phi,Gam,Q1,Q2,Q12)
%	[L,Lv,S] = lqrd(Phi,Gam,Q1,Q2)
%
%	A state feedback gain matrix L is calculated such that the feedback
% 	law u = -Lx minimizes the cost function:
%
%	   J = Sum {x'Q1x + 2x'Q12u + u'Q2u} 
%
%	subject to the constraint equation: 
%
%	   x(k+1) = Phi x(k) + Gam u(k) 
%                
%	When using a Kalman filter, the control signal is formed as
%
%          u(k) =  - L x(k|k-1)               (no direct term)
%          u(k) =  - L x(k|k) - Lv v(k|k)     (direct term)
%
%	Also returned is S, the steady-state solution to the associated 
%	algebraic Riccati equation. 
%
%	The cross-term Q12 is optional. If omitted it is regarded as zero.

% Kjell Gustafsson
% LastEditDate : Mon Jun 10 09:47:25 1991
% Copyright (c) 1990 by Kjell Gustafsson and Department of Automatic Control,
% Lund Institute of Technology, Lund, SWEDEN

% check arguments

error(abcdchk(Phi,Gam));

[ma,na] = size(Phi);
[mb,nb] = size(Gam);
[mq1,nq1] = size(Q1);
if (ma ~= mq1) | (na ~= nq1) 
  error('Phi and Q1 must be the same size')
end
[mq2,nq2] = size(Q2);
if (mq2 ~= nq2) | (nb ~= mq2)
  error('Gam and Q2 must be consistent')
end

if nargin == 5
  [mq12,nq12] = size(Q12);
  if (mq12 ~= ma) | (nq12 ~= nq2)
    error('Q12 must be consistent with Q1 and Q2')
  end
else
  Q12 = zeros(mq1,nq2);
end

Q = [Q1 Q12; Q12' Q2];
% Check if Q is positive semi-definite and symmetric
if any(eig(Q) < -eps) | (norm(Q'-Q,1) > eps*norm(Q,1))
  error('[Q1 Q12; Q12'' Q2] must be symmetric and positive semi-definite')
end

% Check if Q2 is positive semi-definite and symmetric
if any(eig(Q2) < -eps) | (norm(Q2'-Q2,1) > eps*norm(Q2,1))
  error('Q2 must be symmetric and positive semi-definite')
end

% Calculate solution to Riccati equation

S = dare(Phi,Gam,Q1,Q2,Q12);
L  = (Q2 + Gam'*S*Gam)\(Gam'*S*Phi + Q12');  % (Remark 2), p341 CCS
Lv = (Q2 + Gam'*S*Gam)\(Gam'*S);             % p353 CCS

