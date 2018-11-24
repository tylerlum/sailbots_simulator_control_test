%% UBC Sailbots Test Simulator Control of Sailboat from MATLAB
%% Author: Tyler Lum
%% http://docs.ros.org/jade/api/gazebo_msgs/html/msg/LinkStates.html
%% http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html
%% Start Date: Nov 22, 2018

%% Setup ros node
%rosinit('localhost');

base_index = -1;

%% Move robot to position x = 20, then stop
while true
    %% Subscribe to /gazebo/link_states
    gazebo_link_states_msg = get_ros_msg('/gazebo/link_states');
    
    %% Get index of wamv::base_link
    if base_index == -1
        names = gazebo_link_states_msg.Name;
        base_index = get_index(names, 'wamv::base_link');
    end
    
    %% Get current X position
    current_x = gazebo_link_states_msg.Pose(base_index).Position.X;

    %% If current x is too low, keep moving forward
    if current_x < 20
        x_speed = 2;
    else
        x_speed = 0;
    end

    %% Publish speeds
    set_thrusts(x_speed, x_speed, 0);
end

%% Functions
function msg = get_ros_msg(topic)
    sub = rossubscriber(topic);
    pause(1);
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
    