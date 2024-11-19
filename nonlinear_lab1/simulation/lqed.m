function [K,Kf,Kv,P,Pf] = lqed(Phi,C,R1,R2,R12)
% LQED	Linear quadratic estimator for discrete-time systems
%
%	[K,Kf,Kv,P,Pf] = lqed(Phi,C,R1,R2,R12)
%	[K,Kf,Kv,P,Pf] = lqed(Phi,C,R1,R2)
%
%	The gain matrices K and Kf are calculated to minimize the variance 
%       of the estimation error in the stationary Kalman filter
%
%	   x(k+1|k) = Phi x(k|k-1) + Gam u(k) + K (y(k) - C x(k|k-1))
%	   x(k|k) = x(k|k-1) + Kf (y(k) - C x(k|k-1))
%	   v(k|k) = Kv (y(k) - C x(k|k-1))
%
%	given the discrete-time system
%
%	   x(k+1) = Phi x(k) + Gam u(k) + v(k)
%	   y(k)   = C x(k) + e(k)
%               
%	with
%
%	   E{v} = 0, E{e} = 0, E{vv'} = R1, E{ee'} = R2, E{ve'} = R12
%
%	Also returned is P, the steady-state solution to the associated 
%	algebraic Riccati equation. P equals the steady state covariance of
%	x(k+1|k) while Pf equals the covariance after the measurment update,
%	i.e. x(k|k)
%
%	The cross-term R12 is optional. If omitted it is regarded as zero.

% Kjell Gustafsson
% LastEditDate : Mon Jun 10 09:34:55 1991
% Copyright (c) 1990 by Kjell Gustafsson and Department of Automatic Control,
% Lund Institute of Technology, Lund, SWEDEN

% check arguments

error(abcdchk(Phi,ones(length(Phi),1),C));

[ma,na] = size(Phi);
[mc,nc] = size(C);
[mr1,nr1] = size(R1);
if (ma ~= mr1) | (na ~= nr1) 
  error('Phi and R1 must be the same size')
end
[mr2,nr2] = size(R2);
if (mr2 ~= nr2) | (mc ~= mr2)
  error('C and R2 must be consistent')
end

if nargin == 5
  [mr12,nr12] = size(R12);
  if (mr12 ~= ma) | (nr12 ~= nr2)
    error('R12 must be consistent with R1 and R2')
  end
else
  R12 = zeros(mr1,nr2);
end

R = [R1 R12; R12' R2];
% Check if R is positive semi-definite and symmetric
if any(eig(R) < -eps) | (norm(R'-R,1) > eps*norm(R,1))
  error('[R1 R12; R12'' R2] must be symmetric and positive semi-definite')
end

% Check if R2 is positive definite and symmetric
if any(eig(R2) <= -eps) | (norm(R2'-R2,1) > eps*norm(R2,1))
  error('R2 must be symmetric and positive definite')
end

% Calculate solution to Riccati equation

P = dare(Phi',C',R1,R2,R12)';
Kf = P*C'/(C*P*C'+R2);                % (11.50), p353, CCS
Kv = R12/(C*P*C'+R2);                 % (11.50), p353, CCS
K  = Phi*Kf+Kv;                       % Remark 4, p352, CCS
Pf = P - Kf*C*P;                      % (11.50), p353, CCS
