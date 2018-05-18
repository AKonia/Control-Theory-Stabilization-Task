% Copyright 2018 Александр Конаков
clear all;
close all;
graphics_toolkit fltk;


% Управление отрисовкой и задачей
square_size = 2.0;
draw_lines = 1;

global_a = -2; % a < -w^2
global_b = -1; % b < 0
global_w = 1; %
global_u0 = 2.0; %
global global_isLinearSystem = 1;


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
      u0 = back_u0 * sign([p_x, p_y]*[a; b]);
    endif  
    lab_sys_linear_case = 0;
    if(global_isLinearSystem == 1)
      lab_sys_linear_case = @(t,y) [y(2); (w^2)*y(1) + a*y(1) + b*y(2) + u0];
    else
      lab_sys_linear_case = @(t,y) [y(2); (w^2)*sin(y(1)) + a*y(1) + b*y(2) + u0];
    endif
    if(length([timing:calc_step:max_time]) < 2)
    	break
    endif
    [t, y] = ode45(lab_sys_linear_case, [timing:calc_step:max_time], [p_x, p_y]);

    a = back_a;
    b = back_b;
    u0 = back_u0;
    
    for indx = [1:1:length(y)]
      last_control = is_control_type_smart; 
      if(abs(y(indx,:)*[a;b]) < back_u0)
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
endfunction


hold on;
axis([-square_size square_size -square_size square_size]);
draw_curves = 1;
if(draw_lines)
	fi_dot = [-1000:100:1000];
    plot(fi_dot, ( global_u0 - global_a*fi_dot)/global_b, 'linewidth', 1, 'k');
    plot(fi_dot, (-global_u0 - global_a*fi_dot)/global_b, 'linewidth', 1, 'k');
    
    u0 = global_u0;
    a = global_a;
    b = global_b;
    w = global_w;
    calc_step = 0.01;
    
    root_x_right = -(b*u0*w^2 + a*b*u0)/(b^2*w^2 + (a - 1)*b^2 - 1);
    root_x_left = (b*u0*w^2 + a*b*u0)/(b^2*w^2 + (a - 1)*b^2 - 1);
    
    root_y = @(x)(-u0/a - b/a*x);
    root_y_right = root_y(root_x_right);
    root_y = @(x)(u0/a - b/a*x);
    root_y_left = root_y(root_x_left);
    
    plot([root_x_right], [root_y_right], '*');
    plot([root_x_left], [root_y_left], '*');
    
    u0 = 0;
    if(global_isLinearSystem == 1)
      lab_sys_linear_case = @(t,y) [y(2); (w^2)*y(1) + a*y(1) + b*y(2) + u0];
    else
      lab_sys_linear_case = @(t,y) [y(2); (w^2)*sin(y(1)) + a*y(1) + b*y(2) + u0];
    endif
    
    
    
    
    
    [t, y] = ode45(lab_sys_linear_case, [0:-calc_step:-10], [root_x_right, root_y_right]);
    desc = [];
    next_point = 0;
    last_t = 0;
    for indx = [1:1:length(y)]
    	if(abs(y(indx, :)*[a;b]) < global_u0)
    		desc = y(1:indx, :);
    		next_point = y(indx, :);
    		last_t = t(indx);
    		break;
    	endif
    endfor
    
	[t, y] = ode45(lab_sys_linear_case, [last_t:-calc_step:-10], next_point);    
    desc = [];
    for indx = [1:1:length(y)]
    	if(abs(y(indx, :)*[a;b]) > global_u0)
    		desc = [desc, y(1:indx, :)];
    		break;
    	endif
    endfor
    plot(desc(:,1), desc(:,2), 'linewidth', 2, 'k');
    
    [t, y] = ode45(lab_sys_linear_case, [0:-calc_step:-10], [root_x_left, root_y_left]);
    for indx = [1:1:length(y)]
    	if(abs(y(indx, :)*[a;b]) < global_u0)
    		desc = y(1:indx, :);
    		next_point = y(indx, :);
    		break;
    	endif
    endfor
    
    [t, y] = ode45(lab_sys_linear_case, [last_t:-calc_step:-10], next_point);    
    desc = [];
    for indx = [1:1:length(y)]
    	if(abs(y(indx, :)*[a;b]) > global_u0)
    		desc = [desc, y(1:indx, :)];
    		break;
    	endif
    endfor
   
    plot(desc(:,1), desc(:,2), 'linewidth', 2, 'k');
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    if(global_isLinearSystem == 0)
    	fi_dot = [-1:0.1:1];
    	[seps, eigens] = eig([0, 1; global_w^2, 0]);
    	plot(fi_dot - global_u0/(global_w^2), fi_dot*(seps(:,2)(2)/seps(:,2)(1)),'linewidth', 1, 'k');
    	plot(fi_dot - global_u0/(global_w^2), fi_dot*(seps(:,1)(2)/seps(:,1)(1)),'linewidth', 1, 'k');
    	[seps, eigens] = eig([0, 1; global_w^2, 0]);
    	plot(fi_dot + global_u0/(global_w^2), fi_dot*(seps(:,2)(2)/seps(:,2)(1)),'linewidth', 1, 'k');
    	plot(fi_dot + global_u0/(global_w^2), fi_dot*(seps(:,1)(2)/seps(:,1)(1)),'linewidth', 1, 'k');
    endif
endif
while(draw_curves == 1)
  [x_coord, y_coord] = ginput(1);
  [t,y] = custom_ode(x_coord, y_coord, global_w, global_a, global_b, global_u0);
  plot(y(:,1), y(:,2), 'linewidth', 1);
endwhile
