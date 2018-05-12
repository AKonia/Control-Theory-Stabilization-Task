% Copyright 2018 Alexandr Konakov
clear all;
close all;
graphics_toolkit fltk;

function [t, y] = custom_ode(x_coord, y_coord, w, a, b, u0)
  back_a = a;
  back_b = b;
  back_u0 = u0;
  calc_step = 0.1;
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
      u0 = back_u0;
    endif  
  
    lab_sys_linear_case = @(t,y) [y(2); (w^2)*y(1) + a*y(1) + b*y(2) + u0];

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
        descision_y = [descision_y; y(1:indx-1, :)]
        timing += indx*calc_step;
        p_x = y(indx, 1);
        p_y = y(indx, 2);
      endif
        
    endfor
  endwhile
  [t, y] = [[0:calc_step:max_time], descision_y];
  length(t)
  length(y)
endfunction




hold on;
axis([-1.5 1.5 -1.5 1.5]);
draw_curves = 1;
plot([-1.5:0.1:1.5], 1 - 2*[-1.5:0.1:1.5], 'linewidth', 2, 'k');
plot([-1.5:0.1:1.5], -1 - 2*[-1.5:0.1:1.5], 'linewidth', 2, 'k');
while(draw_curves == 1)
  [x_coord, y_coord] = ginput(1);
  [t,y] = custom_ode(x_coord, y_coord, 0.5, 2, 1, 0.0);
  plot(y(:,1), y(:,2), 'linewidth', 1);
endwhile
