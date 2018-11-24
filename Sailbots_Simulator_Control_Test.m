%% UBC Sailbots Test Simulator Control of Sailboat from MATLAB
%% Author: Tyler Lum
%% Start Date: Nov 22, 2018

%% Setup ros node
%rosinit('localhost');

base_index = -1;

%% Move robot to position x = 20, then stop
while true
    %% Subscribe to /gazebo/link_states
    gazebo_link_states_sub = rossubscriber('/gazebo/link_states');
    pause(1);
    gazebo_link_states_msg = receive(gazebo_link_states_sub, 10);
    
    %% Get index of wamv::base_link
    if base_index == -1
        names = gazebo_link_states_msg.Name
        for i = 1:numel(names)
            if strcmp('wamv::base_link', cell2mat(names(i)))
                base_index = i
                break
            end
        end
    end
    
    %% Get current X position
    %% /gazebo/link_states returns a cell with 55 names, poses and speeds. 
    %% http://docs.ros.org/jade/api/gazebo_msgs/html/msg/LinkStates.html
    current_x = gazebo_link_states_msg.Pose(base_index).Position.X

    %% Setup publishing
    [left_thrust_pub, left_thrust_msg] = rospublisher('/left_thrust_cmd', 'std_msgs/Float32');
    [right_thrust_pub, right_thrust_msg] = rospublisher('/right_thrust_cmd', 'std_msgs/Float32');
    
    %% If current x is too low, keep moving forward
    if current_x < 20
        x_speed = 2;
    else
        x_speed = 0;
    end
    
    %% Set thrust values
    left_thrust_msg.Data = x_speed;
    right_thrust_msg.Data = x_speed;

    %% Publish thrust values
    send(left_thrust_pub, left_thrust_msg);
    send(right_thrust_pub, right_thrust_msg);

end