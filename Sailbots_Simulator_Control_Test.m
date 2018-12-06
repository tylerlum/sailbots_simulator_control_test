%% UBC Sailbots Test Simulator Control of Sailboat from MATLAB
%% Author: Tyler Lum
%% http://docs.ros.org/jade/api/gazebo_msgs/html/msg/LinkStates.html
%% http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html
%% Start Date: Nov 22, 2018

%% Setup ros node if not already started
rosshutdown;
rosinit('localhost');

%% Setup subscriber and publishers before looping
gazebo_model_states_sub = rossubscriber('/gazebo/model_states');
pause(1);
gazebo_model_states_msg = receive(gazebo_model_states_sub, 10);

[left_thrust_pub, left_thrust_msg] = rospublisher('/left_thrust_cmd', 'std_msgs/Float32');
[right_thrust_pub, right_thrust_msg] = rospublisher('/right_thrust_cmd', 'std_msgs/Float32');
[lateral_thrust_pub, lateral_thrust_msg] = rospublisher('/lateral_thrust_cmd', 'std_msgs/Float32');

%% Get index of boat (only care about boat's position + orientation)
names = gazebo_model_states_msg.Name;
boat_index = get_index(names, 'wamv')

%% Repeatedly read state and send thrust commands
while true
    %% Get current pose + twist
    gazebo_model_states_msg = receive(gazebo_model_states_sub, 10);  % 10s timeout
    
    %% Example of how to get position, speed, etc. (in world frame)
    pose = gazebo_model_states_msg.Pose(boat_index);
    twist = gazebo_model_states_msg.Twist(boat_index);
    
    % Position
    x = pose.Position.X;
    y = pose.Position.Y;
    z = pose.Position.Z;
    
    % Orientation quaternion
    ow = pose.Orientation.W;
    ox = pose.Orientation.X;
    oy = pose.Orientation.Y;
    oz = pose.Orientation.Z;
    
    % Quaternion to euler angles 
    [r, p, y] = convert_quaternion_to_euler(ow, ox, oy, oz);
    
    % Linear speed
    vx = twist.Linear.X;
    vy = twist.Linear.Y;
    vz = twist.Linear.Z;
    
    % Angular speed
    ax = twist.Angular.X
    ay = twist.Angular.Y
    az = twist.Angular.Z    

    %% Publish thrust values
    F_x = 0;
    F_y = 1;
    M_cw = 0;
    
    [F_l, F_r, F_lat] = convert_net_force_moment_to_thrust(F_x, F_y, M_cw);
    left_thrust_msg.Data = F_l;
    right_thrust_msg.Data = F_r;
    lateral_thrust_msg.Data = F_lat;
   
    send(left_thrust_pub, left_thrust_msg);
    send(right_thrust_pub, right_thrust_msg);
    send(lateral_thrust_pub, lateral_thrust_msg);
end

%% Returns first index of name in all_names
% If names not in all_names, returns -1
function index = get_index(all_names, name)
    for i = 1:numel(all_names)
        if strcmp(name, cell2mat(all_names(i)))
            index = i;
            break;
        end
    end
    i = -1;
end

%% Convert quaternion to euler angles (radians)
% https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
function [r, p, y] = convert_quaternion_to_euler(w, x, y, z)
    r = atan2(2 * (w*x + y*z), (1 - 2 * (x^2 + y^2)));
    p = asin(2 * (w*y - z*x));
    y = atan2(2 * (w*z + x*y), (1 - 2 * (y^2 + z^2)));
end
    
%% Convert input moment and net force to thrusts
function [F_l, F_r, F_lat] = convert_net_force_moment_to_thrust(F_x, F_y, M_cw)
    width = 0.57135;
    F_l = F_y/2 + M_cw/width;
    F_r = F_y/2 - M_cw/width;
    F_lat = F_x;
end


