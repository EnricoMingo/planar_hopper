function result = control()

global h_axes body leg
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global control_state height_desired leg_angle_desired last_bounce_time
global last_touchdown_time last_takeoff_time max_height last_max_height
global speed_desired
global leg_angle_vec leg_angle_desired_vec y_vec xd_vec delta_foot_vec foot_x_vec body_angle_vec

% control_state values
init = 0;
in_air = 1;
on_ground_going_down = 2;
on_ground_going_up = 3;

hip_air_k = 2000;
hip_air_b = 100;
hip_grnd_k = 1.;
hip_grnd_b = 0.01;

leg_length_default = 0.5;

leg_length_gain = 0.0;

rest_leg_length = leg_length_default;
hip_torque = 0;   

foot_y_new = y - rest_leg_length*cos( leg_angle );
leg_length_new = sqrt( (x - foot_x)^2 + (y - foot_y)^2 );

y_vec = [y_vec y];
xd_vec = [xd_vec xd];
body_angle_vec = [body_angle_vec body_angle];
foot_x_vec = [foot_x_vec foot_x-x];

% initialization
if control_state == init
  control_state = in_air;
  result = control_state;
  return;
end;


if control_state == in_air
  if y <= rest_leg_length
    last_touchdown_time = time;
    if yd <= 0
      control_state = on_ground_going_down;
    else
      control_state = on_ground_going_up;
    end;
    result = control_state;
    return;
  end;
  K = 0.108;
  delta_foot = xd*last_bounce_time/2. + K*(xd-speed_desired);
  delta_theta = delta_foot/rest_leg_length;
  leg_angle_desired = -body_angle + asin(delta_theta);
  hip_torque = hip_air_k*(leg_angle - leg_angle_desired) + hip_air_b*leg_angled;
  
  leg_angle_vec = [leg_angle_vec leg_angle];
  leg_angle_desired_vec = [leg_angle_desired_vec leg_angle_desired];
  delta_foot_vec = [delta_foot_vec delta_foot];
  if ( y > max_height )
    max_height = y;
  end;
  if ( yd < 0 )
    last_max_height = max_height;
  end;
end;


if control_state == on_ground_going_down
  if leg_length_new > rest_leg_length
    control_state = in_air;
    max_height = y;
    result = control_state;
    last_takeoff_time = time;
    return;
  end;
  if yd > 0
    control_state = on_ground_going_up;
    result = control_state;
    return;
  end;
  hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
end;

if control_state == on_ground_going_up
  % SET rest_leg_length TO ADD ENERGY
  %rest_leg_length = leg_length_default;
  Kh = 0.25;
  Dh = 0.05;
  rest_leg_length = leg_length_default + Kh*(height_desired-y) -Dh*yd;
  hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
  if leg_length_new > rest_leg_length
    control_state = in_air;
    max_height = y;
    result = control_state;
    last_takeoff_time = time;
    if ( last_touchdown_time > 0 )
      last_bounce_time = last_takeoff_time - last_touchdown_time;
    end;
    return;
  end;
  if yd < 0
    control_state = on_ground_going_down;
    result = control_state;
    return;
  end;
  %hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;





end