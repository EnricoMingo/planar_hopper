% simulate a Raibert hopper

% allocate global variables
global h_axes body leg min_x max_x
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global control_state height_desired leg_angle_desired last_bounce_time
global last_touchdown_time last_takeoff_time max_height last_max_height last_takeoff_yd last_takeoff_leg_angle
global speed_desired leg_angle_flip_vec leg_angle_flip T_flip
global leg_angle_vec leg_angle_desired_vec y_vec xd_vec delta_foot_vec foot_x_vec body_angle_vec body_angled_vec

leg_angle_vec = []; 
leg_angle_desired_vec = [];
y_vec = [];
xd_vec = [];
delta_foot_vec = [];
foot_x_vec = [];
body_angle_vec = [];
body_angled_vec = [];
leg_angle_flip_vec = [];

figure(1) % choose right plot target

% intialize variables.
% stuff we want to control
height_desired = rest_leg_length + 0.5;
speed_desired = 0.;

% constants
dt = 0.001;

% initial state of robot
time = 0.0;
x = 0.0;
y = 2 ;%1.0;
xd = 0.0;
yd = 0.0;
body_angle = 0;
leg_angle = 0.0;
body_angled = 0;
leg_angled = 0;
hip_torque = 0;
leg_state = 0; % simulator state: leg not on ground
% controller state
control_state = 0; 
last_bounce_time = 0.25;
last_touchdown_time = -1;
last_takeoff_time = -1;
max_height = y;
last_max_height = y;
last_takeoff_yd = 0;
last_takeoff_leg_angle = 0;

max_n_points = 1000;

% allocate array
array(max_n_points,9) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% outer loop: save data and draw picture at slow rate
while 1
    
%      pause
    
%     if time > 10 && time < 20
%         speed_desired = 0.9;
%     end
%     
%     if time > 15 && time < 20
%         height_desired = rest_leg_length + 0.8;
% %     else 
% %         height_desired = rest_leg_length + 0.2;
%     end
%     
%     if time > 20 && time < 30
%         speed_desired = -0.5;
%     end    
%     
%     if time > 30
%         speed_desired = 0.;
%     end
    
    
% simulate and control at faster rate
 for j = 1:10
  control;
  simulate();
 end;

% save data in array
%  if ( i <= max_n_points )
%    array(i,1) = time;
%    array(i,2) = y;
%    array(i,3) = yd;
%    array(i,4) = control_state;
%    array(i,5) = body_angle;
%    array(i,6) = leg_angle;
%    array(i,7) = hip_torque;
%    array(i,8) = leg_angle_desired;
%    array(i,9) = xd;
%  end;

% hack to keep it in view
 if ( x > max_x )
  x = x - (max_x - min_x); 
  foot_x = foot_x - (max_x - min_x); 
 end;

 if ( x < min_x )
  x = x + (max_x - min_x); 
  foot_x = foot_x + (max_x - min_x); 
 end;
 draw();

% have we crashed?
 if y < 0.1
   break;
 end;

% keep track of how long before crash.
 if ( i <= max_n_points )
   max_i = i; 
 end;
end
