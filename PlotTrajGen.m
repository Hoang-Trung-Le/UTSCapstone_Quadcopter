function PlotTrajGen(a, t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

   s = size(a.acce,2);

   fig = tiledlayout(2,3,'TileSpacing','Compact');
   title(fig,'Trajectory generation','FontWeight','bold')
   xlabel(fig,'Time (s)')

   nexttile;
   plot(t(1:s),a.jounce)
   ylim([min(a.jounce-0.5) max(a.jounce+0.5)]);
   ylabel('Jounce (m/s^4)')
   title('Jounce');
   
   nexttile
   plot(t(1:s), a.jerk);
   ylim([min(a.jerk-0.5) max(a.jerk+0.5)]);
   ylabel('Jerk (m/s^3)')
   title('Jerk');
   
   nexttile
   plot(t(1:s), a.acce);
   ylim([min(a.acce-0.5) max(a.acce+0.5)]);
   ylabel('Acceleration (m/s^2)')
   title('Acceleration');
   
   nexttile
   plot(t(1:s), a.velo);
   ylim([min(a.velo-0.5) max(a.velo+0.5)]);
   ylabel('Velocity (m/s)')
   title('Velocity');
   
   nexttile([1 2])
   plot(t(1:s), a.pos);
   ylim([min(a.pos-1) max(a.pos+1)]);
   ylabel('Position (m)')
   title('Position');

end