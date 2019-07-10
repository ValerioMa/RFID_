%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize and function paths
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear all; close all;

tag_pos = [0;0];

robot_pose = [-100;-4;0];
v = 1;


T_sim = 200;
dt = 0.01;

P = diag([1,1])*100;
d_error = [];
for i=0:1:(T_sim/dt)
    robot_pose = robot_pose + v*dt*[cos(robot_pose(3));sin(robot_pose(3));0];
    
    delta = tag_pos - robot_pose(1:2);    
    alpha = atan2(delta(2),delta(1)) - robot_pose(3);
    d = norm(delta);
    ideal_x = [alpha;d];
    if i==0
        x = ideal_x;
    end
    
    [x,P] = prediction(x,P,v,dt);
    error = x-ideal_x;
    d_error(end+1) = error(2);
    [x,P] = update(x,P,alpha+randn()*0.1);
    
end

plot(d_error)

function [x,P] = prediction(x,P,v,dt)    
    x_dot(1,1) = v*sin(x(1))/x(2);
    x_dot(2,1) = -v*cos(x(1));
    
    R =eye(2,2)*1e-1;

    A = dt*v*[cos(x(1))/x(2) -sin(x(1))/x(2)^2; -cos(x(1)) 0];
    P = A*P*A' + R; 
    x = x + x_dot*dt;
end

function  [x,P] = update(x,P,z)
    H = [1 0];
    R = 0.1;
    z_pred = H*x;
    
    y = z - z_pred; % innovation
    while y>pi
        y = y - 2*pi;
    end
    while y<-pi
        y = y + 2*pi;
    end
    S = R + H*P*H'; % innovation covariance
    K = P*H'/S;     % kalman gain
    
    % Update state and covariance
    x = x + K*y;     % state update
    P = P - K*H*P;   % covariance update
end