% Copyright 2018 Alexandr Konakov
clear all;
close all;
graphics_toolkit fltk;


% Управление отрисовкой
square_size = 5.0;
draw_lines = 1;











function [t, y] = custom_ode(x_coord, y_coord, w, a, b, u0)
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
        is_control_type_smart = 1
      else
        is_control_type_smart = 0
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
  %[t, y] = [[0:calc_step:max_time], ];
  length(t)
  length(y)
endfunction




hold on;
axis([-square_size square_size -square_size square_size]);
draw_curves = 1;
if(draw_lines)
    plot([-1.5:0.1:1.5], 1 - 2*[-1.5:0.1:1.5], 'linewidth', 2, 'k');
    plot([-1.5:0.1:1.5], -1 - 2*[-1.5:0.1:1.5], 'linewidth', 2, 'k');
endif
while(draw_curves == 1)
  [x_coord, y_coord] = ginput(1);
  [t,y] = custom_ode(x_coord, y_coord, 1, -2.0, -3, 5.0);
  plot(y(:,1), y(:,2), 'linewidth', 1);
endwhile
