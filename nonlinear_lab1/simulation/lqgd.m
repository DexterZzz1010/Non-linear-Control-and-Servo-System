function [Lx,Ly,Phic,Gamy,Gamyr,Cc,Dy,Dyr]=lqgd(Phi,Gam,C,L,Lv,lr,K,Kf,Kv,dir)
% LQGD	Calculates a discrete-time linear quadratic gaussian controller 
%       from data given by LQRD and LQED.
%
%	[Lx,Ly,Phic,Gamy,Gamyr,Cc,Dy,Dyr] = lqgd(Phi,Gam,C,L,Lv,lr,K,Kf,Kv,dir)
%	[Lx,Ly,Phic,Gamy,Gamyr,Cc,Dy,Dyr] = lqgd(Phi,Gam,C,L,Lv,lr,K,Kf,Kv)
%
%	A discrete-time LQG controller can be implemented through
%
%	   x(k+1|k) = Phi x(k|k-1) + Gam usat(k) + K (y(k) - C x(k|k-1))
%	   u(k) = lr yr(k) - Lx x(k|k-1) - Ly y(k)
%          usat(k) = sat u(k)
%
%	where the values of Lx and Ly depend on whether the controller
%	contains a direct term or not. If the parameter 'dir' is supplied
%       (any non-empty value) the direct term case is chosen.
%
%       The controller can also be expressed as (excluding the saturation)
%
%          u(k) = Hff(q) yr(k) - Hfb(q) y(k)
%
%       with
%
%          Hff(q) = Cc ( qI - Phic )^(-1) Gamyr + Dyr
%          Hfb(q) = Cc ( qI - Phic )^(-1) Gamy  + Dy 
%
%       Note that the 'feedback minus sign' is not included in Hfb.

% Kjell Gustafsson
% LastEditDate : Thu Jan 17 16:04:07 1991
% Copyright (c) 1990 by Kjell Gustafsson and Department of Automatic Control,
% Lund Institute of Technology, Lund, SWEDEN

% check arguments

if nargin<10
  % no direct term
  Ly = zeros(size(Gam)*[0;1],size(C)*[1;0]);
  Lx = L;
else
  % direct term
  Ly = L*Kf + Lv*Kv;
  Lx = L - Ly*C;
end

% controller dynamics

Phic = Phi - Gam*Lx - K*C;
Gamy  = K-Gam*Ly;
Gamyr = -lr*Gam;
Cc = Lx;
Dy  = Ly;
Dyr = lr;

