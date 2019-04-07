% Simulation settign of input data for sailboat motion.
% Nicolas Navarre
% UBC Sailbot
% Mar 6 2019
%%
% These will be the parameters that will be passed by software like
% Gazeebo.

v_tw_in = 20; %True wind velocity, m/s
alpha_tw_in = 3*pi/4; %True wind angle (n-frame), radians

x_in = 0; %Starting x-coordinate, m
y_in = 0; %Starting y-coordinate, m
phi_in = 0; %Starding roll angle radians
psi_in = 0; %Starting yaw angle (pointing), radians

dx_in = 1; %Starting x-velocity (n-frame), m/s
dy_in = 0; %Starting y-velocity (n-frame), m/s
roll_in = 0; %Starting roll (n-frame), rad/s
yaw_in = 0; %Starting yaw (n-frame), rad/s

sangle_in = pi/3; %Starting Sail position, radians
rangle_in = 0; %Starting rudder angle, radians

%save('inputData');