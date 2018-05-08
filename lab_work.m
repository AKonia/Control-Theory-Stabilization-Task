%function numeric_decision = solve_diff_matrix(diff_system, init)
clear all;
close all;
graphics_toolkit fltk;

function [t, y] = custom_ode(x, y, w, a, b, u0)
  lab_sys_linear_case = @(t,y) [y(2);
                  (w^2)*y(1) + a*y(1) + b*y(2) + u0];
  [t, y] = ode45(lab_sys_linear_case, [0:0.1:50], [x, y]);
endfunction


hold on;
axis([-1.5 1.5 -1.5 1.5]);
draw_curves = 1;

plot([-1.5:0.1:1.5], 1 - 2*[-1.5:0.1:1.5], 'linewidth', 2, 'k');

plot([-1.5:0.1:1.5], -1 - 2*[-1.5:0.1:1.5], 'linewidth', 2, 'k');

while(draw_curves == 1)
  [x, y] = ginput(1);
  [t,y] = custom_ode(x, y, 0.5, 2, -1, 0.0);
  plot(y(:,1), y(:,2), 'linewidth', 1);
endwhile
