%% Set input values
v_tw_in = 5; %True wind velocity, m/s
alpha_tw_in = pi / 8; %True wind angle (n-frame), radians

x_in = position(1); %Starting x-coordinate, m
y_in = position(2); %Starting y-coordinate, m
phi_in = orientation(3) * pi / 180; %Starding roll angle radians
psi_in = orientation(1) * pi / 180; %Starting yaw angle (pointing), radians

dx_in = velocity.Linear(1); %Starting x-velocity (n-frame), m/s
dy_in = velocity.Linear(2); %Starting y-velocity (n-frame), m/s
roll_in = velocity.Angular(1) * pi / 180; %Starting roll (n-frame), rad/s
yaw_in = velocity.Angular(3) * pi / 180; %Starting yaw (n-frame), rad/s

sangle_in = get_sail_angle(psi_in, alpha_tw_in); %Starting Sail position, radians--SET THIS TO DETERMINE IF POS OR NEGATIVE
rangle_in = 0; %Starting rudder angle, radians

n_in = [x_in,y_in,phi_in,psi_in];
v_in = transpose(eval(subs(J,[psi,phi],[psi_in,phi_in])\[dx_in;dy_in;roll_in;yaw_in]));
ang_in = [sangle_in,rangle_in];

%% Substitute values into T for Force/Moment
T = subs(T, [ss_vars, ss_in, v_tw, alpha_tw], [n_in, v_in, ang_in, v_tw_in, alpha_tw_in]);

%% Get correct sail angle based on heading and wind angle
function sangle = get_sail_angle(psi_in, alpha_tw_in)
    angle_diff = (alpha_tw_in - psi_in) * 180 / pi;
    
    %% Adjust to be in [-180, 180]
    if angle_diff > 180
        angle_diff = angle_diff - 360;
    elseif angle_diff < -180
        angle_diff = angle_diff + 360;
    end
    angle_diff
    tolerance = 10
    
    %% 3 states: sangle = 0, pi/3, -pi/3
    if and(angle_diff > tolerance, angle_diff < 180 - tolerance)
        sangle = -pi / 3
    elseif and(angle_diff < -tolerance, angle_diff > -180 + tolerance)
        sangle = pi / 3
    else
        sangle = 0
    end
end