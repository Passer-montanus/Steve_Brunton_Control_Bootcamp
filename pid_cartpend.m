clear all, close all, clc

m = 1;  % mass of the pendulum
M = 5;  % mass of the cart
L = 2;  % length of the pendulum
g = -10; % gravity
d = 1;  % damping

s = 1;  % pendulum up (s=1)

% System dynamics matrix
A = [0 1 0 0;
     0 -d/M -m*g/M 0;
     0 0 0 1;
     0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

% PID Controller gains
Kp = 3;  % proportional gain
Ki = 1;    % integral gain
Kd = 1;   % derivative gain

% Simulation time span
tspan = 0:.001:10;
y0 = [-3; 0; pi+.1; 0; 0; 0];  % initial conditions [position; velocity; angle; angular velocity; integral error; previous error]

% Control target
target = [1; 0; pi; 0];  % [desired position; desired velocity; desired angle; desired angular velocity]

% ODE solver with PID control
[t,y] = ode45(@(t,y) cartpend_pid(y, m, M, L, g, d, Kp, Ki, Kd, target), tspan, y0);

% Visualization
for k=1:100:length(t)
    drawcartpend(y(k,1:4),m,M,L);  % Pass only the state variables to the drawing function
end

function dy = cartpend_pid(y, m, M, L, g, d, Kp, Ki, Kd, target)
    % Extract state variables
    x = y(1);
    x_dot = y(2);
    theta = y(3);
    theta_dot = y(4);
    integral = y(5);  % integral of error
    previous_error = y(6);  % previous error for derivative calculation

    % Error calculations
    error = target - [x; x_dot; theta; theta_dot];
    integral = integral + error(1) * 0.001; % Update integral of error
    derivative = (error(1) - previous_error) / 0.001; % Update derivative of error

    % PID Control
    u = Kp*error(1) + Ki*integral + Kd*derivative;

    % Update dynamics with control input
    dy = zeros(6,1);  % Initialize dy
    dy(1) = x_dot;
    dy(2) = (m*sin(theta)*(L*theta_dot^2 + g*cos(theta)) + d*x_dot + u) / (M + m*sin(theta)^2);
    dy(3) = theta_dot;
    dy(4) = (-m*L*theta_dot^2*cos(theta)*sin(theta) - (M+m)*g*sin(theta) - d*x_dot*cos(theta) - u*cos(theta)) / (L*(M + m*sin(theta)^2));
    dy(5) = integral;  % Pass integral forward in time
    dy(6) = error(1);  % Pass current error as previous error for next call

    return
end
