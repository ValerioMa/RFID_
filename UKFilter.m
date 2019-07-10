%
clc;
clear all;
close all force;
addpath('./utility');
addpath(genpath('./utility/UKF/'));
% rng('default'); % for repeatable result

%% Parameters
num_rfid = 1; 
% % for num_rfid = 1:4

num_filters = 1;
t_stop = 5000;  % seconds to simulate

% Sampling times
dt = 0.01;               % simulation time step
rfid_measure_dt = 0.01;  % measure frequency

l = 5;   % rfid semidistance

% Initial position and covariance of the robot
initialPose = [-9   0 -pi/2]';
cov_x     = 0.5^2;
cos_theta = 0.1745^2;
initialCov = diag([cov_x,cov_x,cos_theta]);

% Odometry noise
add_odom_noise = true;
std_v     = 0.5/10;
std_omega = 0.17/10;

% Plo params
plot_dt  = 1/5;          % seconds between frame
L = 0.4; % wheelbase


RFID = [0, 0,  l, -l
        l, -l, 0,  0]; % Position of RFID
RFID = RFID(:,1:num_rfid);

% Measure noise
add_measure_noise = true;
R = (0.05)^2*eye(size(RFID,2),size(RFID,2)); % measure covariance

%% Define the path
follow_path = true;
p1(1,:) = -100:0.3:-0.2;
p1(2,:) = -4;
p1(3,:) = 0;
p1(4,:) = 0;
p2(2,:) = -4:0.2:4.7;
p2(1,:) = -0;
p2(3,:) = pi/2;
p2(4,:) = 0;
p3(1,:) = 0.0:0.1:100;
p3(2,:) = 5;
p3(3,:) = 0;
p3(4,:) = 0;
path = [p1,p2,p3]; 
Rag = 6;
x_c = -3;
y_c = 1;
dTheta = 0.05/Rag;
thetas = 0:dTheta:2*pi;
path = [x_c;y_c] + Rag*[cos(thetas);sin(thetas)];
path(3,:) = thetas+pi/2;
path(4,:) = 1/Rag*ones(1,numel(thetas));


%% Initialize a unicycle-like Robot
tartufino = UnicycleBot(initialPose, dt, L);
ax = gca;
hold(ax, 'on') 
plot(ax,RFID(1,:),RFID(2,:),'.m','markersize',5);
hold(ax, 'off') 

filters = {};
for i=1:num_filters
    noisy_initial_pose = mvnrnd(initialPose,initialCov);
    filters{i,1} = UKF(noisy_initial_pose,initialCov);
    filters{i,2} = ['ukf',num2str(i)];
end
% filters{2} = EKF(noisy_initial_pose,initialCov);

%% Main loop
r = robotics.Rate(1/plot_dt);

reset(r);
simulationTime = 0;
last_measure = -inf;
last_plot = -inf;


real_trace = [];   % traccia seguita veramente dal robot
filters_trace = {};  % traccia data dal filtro
for i=1:size(filters,1)
    filters_trace{i} = [];
end
h_waitbar = waitbar(0, 'Simulating...'); 
while simulationTime < t_stop % if time is not up    
    % Compute the command to send to the robot
    uCmd(1) = 1;  % linear velocity
    if follow_path        
        P_frenet = find_frenet(path,tartufino.Pose);
        rho = tartufino.path_folowing(P_frenet);
        uCmd(2) = rho*uCmd(1);  % angular velocity 
    else
        uCmd(2) = 0;  % linear velocity
    end 
    % Drive the real robot
    drive(tartufino, uCmd);
    
    % Compute the measure and predict
    cov_u = [std_v^2, 0; 0, std_omega^2]; 
    for i=1:size(filters,1)
        if add_odom_noise
            measure_uCmd = mvnrnd(uCmd,cov_u)';
        else
            measure_uCmd = uCmd;
        end
        filters{i,1}.Prediction(measure_uCmd,dt,cov_u);
    end    
    
    % Measure correction
    if (simulationTime-last_measure)>rfid_measure_dt
       % Add noise to the measure 
       ideal_measurement = rfidReadings(tartufino.Pose,uCmd(1),RFID); 
       if ~isnan(ideal_measurement)                      
           for i=1:size(filters,1)
               if add_measure_noise
                measurement = mvnrnd(ideal_measurement, R)';          
                filters{i,1}.UpdateRFID(measurement,R,RFID);     
               else
                filters{i,1}.UpdateRFID(ideal_measurement,R,RFID);  
               end
           end 
       end
       last_measure = simulationTime;           
    end
        
    % keep track of the path
    real_trace(:,end+1) = tartufino.Pose;
    for i=1:size(filters,1)
        filters_trace{i}(:,end+1) = filters{i,1}.x;
    end
    
    % Update plot
    if isempty(get(groot,'CurrentFigure')) % if figure is not prematurely killed        
       break
    end
    
%     if (simulationTime-last_plot)>plot_dt
%         tartufino.updatePlot(ukf.x,ukf.P, simulationTime);
%         axis auto
%         xlim([-10,10]);
%         ylim([-10,10]);
%         axis equal;
%         last_plot = simulationTime;
%         waitfor(r);  
%     end
    
    % Update simulation time
    simulationTime = simulationTime + dt;
    
%     if(tartufino.Pose(1)>5)
%         break;
%     end
    h_waitbar = waitbar(simulationTime/t_stop); 
end
close(h_waitbar);
% save(['./data/',num2str(num_rfid)],'filters_trace','real_trace');
% % % end


%% Final plot
figure();
ax = gca;
hold on 
plot(ax,RFID(1,:),RFID(2,:),'s');
plot(ax,real_trace(1,:),real_trace(2,:),'r');
legenda = {};
legenda{1} = 'RFID location';
legenda{2} = 'real trace';
for i=1:size(filters,1)
    plot(ax,filters_trace{i}(1,:),filters_trace{i}(2,:));
    legenda{end + 1} = filters{i,2};
end
legend(legenda);
hold off

%%
figure();
for i=1:size(filters,1)
    subplot(3,1,1);
    hold on
    plot(filters_trace{i}(1,:)-real_trace(1,:))
    ylabel('x error [m]');
    xlabel('iteration number');
    subplot(3,1,2);
    hold on
    plot(filters_trace{i}(2,:)-real_trace(2,:))
    ylabel('y error [m]');
    xlabel('iteration number');
    subplot(3,1,3);
    hold on
    plot((filters_trace{i}(3,:)-real_trace(3,:))*180/pi)
    ylabel('theta error grad');
    xlabel('iteration number');
end