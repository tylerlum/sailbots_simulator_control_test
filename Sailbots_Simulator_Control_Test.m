%% UBC Sailbots Test Simulator Control of Sailboat from MATLAB
%% Author: Tyler Lum
%% http://docs.ros.org/jade/api/gazebo_msgs/html/msg/LinkStates.html
%% http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html
%% Start Date: Nov 22, 2018

%% Setup ros node if not already started
if not(robotics.ros.internal.Global.isNodeActive)
    rosinit('localhost');
end

%% Setup subscriber before looping
gazebo_link_states_sub = rossubscriber('/gazebo/link_states');
pause(1);
gazebo_link_states_msg = receive(gazebo_link_states_sub, 10);

%% Get index of base_link (only care about boat's position + orientation)
names = gazebo_link_states_msg.Name;
base_index = get_index(names, 'wamv::base_link');

%% Setup publishers before looping
[left_thrust_pub, left_thrust_msg] = rospublisher('/left_thrust_cmd', 'std_msgs/Float32');
[right_thrust_pub, right_thrust_msg] = rospublisher('/right_thrust_cmd', 'std_msgs/Float32');
[lateral_thrust_pub, lateral_thrust_msg] = rospublisher('/lateral_thrust_cmd', 'std_msgs/Float32');

%% Move robot to position x = 20, then stop
while true
    %% Get current pose + twist
    gazebo_link_states_msg = receive(gazebo_link_states_sub, 10);  % 10s timeout
    
    %% Example of how to get position, speed, etc.
    pose = gazebo_link_states_msg.Pose(base_index);
    twist = gazebo_link_states_msg.Twist(base_index);
    
    %% Position
    x = pose.Position.X;
    y = pose.Position.Y;
    z = pose.Position.Z;
    
    %% Orientation quaternion
    ox = pose.Orientation.X;
    oy = pose.Orientation.Y;
    oz = pose.Orientation.Z;
    ow = pose.Orientation.W;
    
    %% Linear speed
    vx = twist.Linear.X;
    vy = twist.Linear.Y;
    vz = twist.Linear.Z;
    
    %% Angular speed
    ax = twist.Angular.X;
    ay = twist.Angular.Y;
    az = twist.Angular.Z;    

    %% If current x is too low, keep moving forward
    if x < 20 
        speed = 2;
    else
        speed = 0;
    end

    %% Set thrust values
    left_thrust_msg.Data = speed;
    right_thrust_msg.Data = speed;
    lateral_thrust_msg.Data = 0;
    
    %% Publish thrust values
    send(left_thrust_pub, left_thrust_msg);
    send(right_thrust_pub, right_thrust_msg);
    send(lateral_thrust_pub, lateral_thrust_msg);
end

function index = get_index(all_names, name)
    for i = 1:numel(all_names)
        if strcmp(name, cell2mat(all_names(i)))
            index = i;
            break;
        end
    end
end

