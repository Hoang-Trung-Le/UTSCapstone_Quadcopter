function traj = TrajectoryGeneration(pos, velo, t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

   const = 4/pi^3 + 1/pi;
   b = 0.5;
   a = velo / (3*b^3*const);
   c = (pos - 12*a*b^4*const)/(3*a*b^3*const);
   inc = t(2)-t(1);
   startDece = find(t == round(4*b+c, 2));
   [accelerate, endAcce] = f(a, b, 1, t);
   decelerate = finv(a, b, c, startDece, t);
   steady = zeros(1,startDece - endAcce);
   traj.jounce = [accelerate, steady, decelerate];
   traj.jerk = cumtrapz(inc, traj.jounce);
   traj.acce = cumtrapz(inc, traj.jerk);
   traj.velo = cumtrapz(inc, traj.acce);
   traj.pos = cumtrapz(inc, traj.velo);

end

function [F, endVec] = f(a, b, start, t)
   idx = round(b/(t(2)-t(1)));
   f1 = a*sin(pi*t(start:start+idx)/b);
   f2 = -a*sin(0.5*pi*t(start+idx+1:start+3*idx)/b - pi/2);
   f3 = a*sin(pi*t(start+3*idx+1:start+4*idx-1)/b - 3*pi);
   F = [f1 f2 f3];
   endVec = round(start+4*idx);
end

function F = finv(a, b, c, start, t)
   idx = round(b/(t(2)-t(1)));
   f1 = a*sin(pi*t(start:start+idx)/b - (4*b+c)*pi/b);
   f2 = -a*sin(pi*t(start+idx+1:start+3*idx)/(2*b) - (5*b+c)*pi/(2*b));
   f3 = a*sin(pi*t(start+3*idx+1:start+4*idx-1)/b - (7*b+c)*pi/b);
   F = -[f1 f2 f3];
end