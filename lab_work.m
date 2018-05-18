% Copyright 2018 Александр Конаков
clear all;
close all;
graphics_toolkit fltk;


% Управление отрисовкой и задачей
square_size = 5.0;
draw_lines = 1;

global_a = -0.28; % a < -w^2
global_b = -0.1; % b < 0
global_w = 0.5; %
global_u0 = 1.0; %
global global_isLinearSystem = 2;




function [t, y] = custom_ode(x_coord, y_coord, w, a, b, u0)
  global global_isLinearSystem;
  back_a = a;
  back_b = b;
  back_u0 = u0;
  calc_step = 0.01;
  max_time = 50.0;
  timing = 0.0;
  
  p_x = x_coord;
  p_y = y_coord;
  descision_y = []; 
  descision_t = [];
  is_control_type_smart = 1;
  
  if(abs([a, b]*[p_x; p_y]) < u0)
    is_control_type_smart = 1;
  else
    is_control_type_smart = 0;
  endif
  
  while(timing < max_time)
    printf("Iteration  WHILE\n");
    if(is_control_type_smart)
      a = back_a;
      b = back_b;
      u0 = 0;
    else
      a = 0;
      b = 0;
      u0 = back_u0 * sign([a, b]*[p_x; p_y]);
    endif  
    lab_sys_linear_case = 0;
    if(global_isLinearSystem)
      lab_sys_linear_case = @(t,y) [y(2); (w^2)*y(1) + a*y(1) + b*y(2) + u0];
    else
      lab_sys_linear_case = @(t,y) [y(2); (w^2)*sin(y(1)) + a*y(1) + b*y(2) + u0];
    endif
    
    [t, y] = ode45(lab_sys_linear_case, [timing:calc_step:max_time], [p_x, p_y]);

    a = back_a;
    b = back_b;
    u0 = back_u0;
    
    for indx = [1:1:length(y)]
      last_control = is_control_type_smart; 
      if(abs(y(indx)*[a;b]) < u0)
        is_control_type_smart = 1;
      else
        is_control_type_smart = 0;
      endif
        
      if(is_control_type_smart != last_control)
        p_x = y(indx, 1);
        p_y = y(indx, 2);
        timing += calc_step;
        break;
      endif
        descision_y = [descision_y; y(indx, :)];
        timing += calc_step;
    endfor
  endwhile
  t = [0:calc_step:max_time];
  y = descision_y;
  t = t(1:length(y));
  length(t)
  length(y)
endfunction




hold on;
axis([-square_size square_size -square_size square_size]);
draw_curves = 1;
if(draw_lines)
	fi_dot = [-10:0.1:10];
    plot(fi_dot, ( global_u0 - global_a*fi_dot)/global_b, 'linewidth', 2, 'k');
    plot(fi_dot, (-global_u0 - global_a*fi_dot)/global_b, 'linewidth', 2, 'k');
endif
while(draw_curves == 1)
  [x_coord, y_coord] = ginput(1);
  [t,y] = custom_ode(x_coord, y_coord, global_w, global_a, global_b, global_u0);
  plot(y(:,1), y(:,2), 'linewidth', 1);
endwhile
