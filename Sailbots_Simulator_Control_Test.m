%% UBC Sailbots Test Simulator Control of Sailboat from MATLAB
%% Author: Tyler Lum
%% http://docs.ros.org/jade/api/gazebo_msgs/html/msg/LinkStates.html
%% http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html
%% Start Date: Nov 22, 2018

%% Setup ros node
%rosinit('localhost');

%% Setup publisher and subscriber before looping
gazebo_link_states_sub = rossubscriber('/gazebo/link_states');
pause(1);
gazebo_link_states_msg = receive(gazebo_link_states_sub, 10);

names = gazebo_link_states_msg.Name;
base_index = get_index(names, 'wamv::base_link');

[left_thrust_pub, left_thrust_msg] = rospublisher('/left_thrust_cmd', 'std_msgs/Float32');
[right_thrust_pub, right_thrust_msg] = rospublisher('/right_thrust_cmd', 'std_msgs/Float32');
[lateral_thrust_pub, lateral_thrust_msg] = rospublisher('/lateral_thrust_cmd', 'std_msgs/Float32');

%% Move robot to position x = 20, then stop
while true
    %% Subscribe to /gazebo/link_states
    gazebo_link_states_msg = receive(gazebo_link_states_sub, 10);
    
    %% Get current X position
    current_pose = gazebo_link_states_msg.Pose(base_index);
    current_twist = gazebo_link_states_msg.Twist(base_index);

    current_x = current_pose.Position.X;

    %% If current x is too low, keep moving forward
    if current_x < 20
        x_speed = 2;
    else
        x_speed = 0;
    end

    %% Set thrust values
    left_thrust_msg.Data = x_speed;
    right_thrust_msg.Data = x_speed;
    lateral_thrust_msg.Data = 0;
    
    %% Publish thrust values
    send(left_thrust_pub, left_thrust_msg);
    send(right_thrust_pub, right_thrust_msg);
    send(lateral_thrust_pub, lateral_thrust_msg);
end

%% Functions
function msg = get_ros_msg(topic)
    sub = rossubscriber(topic);
    pause(0.2);
    msg = receive(sub, 10);
end

function index = get_index(all_names, name)
    for i = 1:numel(all_names)
        if strcmp(name, cell2mat(all_names(i)))
            index = i;
            break;
        end
    end
end

function set_thrusts(left_thrust, right_thrust, lateral_thrust)
    %% Setup publishing
    [left_thrust_pub, left_thrust_msg] = rospublisher('/left_thrust_cmd', 'std_msgs/Float32');
    [right_thrust_pub, right_thrust_msg] = rospublisher('/right_thrust_cmd', 'std_msgs/Float32');
    [lateral_thrust_pub, lateral_thrust_msg] = rospublisher('/lateral_thrust_cmd', 'std_msgs/Float32');
    
    %% Set thrust values
    left_thrust_msg.Data = left_thrust;
    right_thrust_msg.Data = right_thrust;
    lateral_thrust_msg.Data = lateral_thrust;
    
    %% Publish thrust values
    send(left_thrust_pub, left_thrust_msg);
    send(right_thrust_pub, right_thrust_msg);
    send(lateral_thrust_pub, lateral_thrust_msg);
end
    
