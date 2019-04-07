%% UBC Sailbots Control + Position Plot
%% Author: Tyler Lum + Yiling Kang
%% http://docs.ros.org/jade/api/gazebo_msgs/html/msg/LinkStates.html
%% http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html
%% Start Date: March 9, 2019

%% MUST RUN rosshutdown before running

%% Housekeeping
close all

%% Setup ros node
%rosshutdown;
rosinit('localhost');

%% Setup reading gazebo states
gazebo = ExampleHelperGazeboCommunicator;
phys = readPhysics(gazebo);
models = getSpawnedModels(gazebo);

%% Setup publisher for boat forces
[left_thrust_pub, left_thrust_msg] = rospublisher('/left_thrust_cmd', 'std_msgs/Float32');
[right_thrust_pub, right_thrust_msg] = rospublisher('/right_thrust_cmd', 'std_msgs/Float32');
[lateral_thrust_pub, lateral_thrust_msg] = rospublisher('/lateral_thrust_cmd', 'std_msgs/Float32');

%% Setup for sailboat calculations
set_data;
nonlinear_sailboat_motion;

%% Repeatedly read state and send thrust commands -> getting position over time and plotting
position_over_time = [];
tic;
store = [];
while toc < 60
   %% Get boat state and store position over time
   wamv = ExampleHelperGazeboSpawnedModel('wamv', gazebo);
   [position, orientation, velocity] = getState(wamv);

   %% Get forces
   get_force;
   
   F_x = double(T(1)) / 1000;
   F_y = double(T(2)) / 1000;
   M_yaw = double(T(4)) / 1000;

   %% Convert to thruster forces
   [F_l, F_r, F_lat] = convert_net_force_moment_to_thrust(F_x, F_y, M_yaw);
   left_thrust_msg.Data = F_l;
   right_thrust_msg.Data = F_r;
   lateral_thrust_msg.Data = F_lat;

   %% Send forces to Gazebo
   send(left_thrust_pub, left_thrust_msg);
   send(right_thrust_pub, right_thrust_msg);
   send(lateral_thrust_pub, lateral_thrust_msg);

   %% Track position over time
   position_over_time = [position_over_time; toc position];
end

%% Plotting position over time generated over 30 seconds
plot(position_over_time(:,1), position_over_time(:,2))
xlabel('Time Elapsed (s)')
ylabel('Boat Position x (m?)')

%% Convert input moment and net force to thrusts
function [F_l, F_r, F_lat] = convert_net_force_moment_to_thrust(F_x, F_y, M_cw)
   width = 0.57135;
   F_l = F_x/2 + M_cw/width;
   F_r = F_x/2 - M_cw/width;
   F_lat = F_y;
end