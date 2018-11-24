%% UBC Sailbots Test Simulator Control of Sailboat from MATLAB
%% Author: Tyler Lum
%% Start Date: Nov 22, 2018

%% Setup ros node
rosinit('localhost');

while true
    %% Setup publishing
    [left_thrust_pub, left_thrust_msg] = rospublisher('/left_thrust_cmd', 'std_msgs/Float32');
    [right_thrust_pub, right_thrust_msg] = rospublisher('/right_thrust_cmd', 'std_msgs/Float32');
    
    %% Set thrust values
    left_thrust_msg.Data = -1;
    right_thrust_msg.Data = -1;
    
    %% Publish thrust values
    send(left_thrust_pub, left_thrust_msg);
    send(right_thrust_pub, right_thrust_msg);
end