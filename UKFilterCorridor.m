%
clc;
clear all;
close all force;
addpath('./utility');
addpath(genpath('./utility/UKF/'));
% rng('default'); % for repeatable result

%% Parametri simulazione
num_filters = 10;
path_num = 100;
t_stop = 150;        % seconds to simulate
detection_range = 5; % max detection range of RFID

% Sampling times
dt = 0.01;               % simulation time step
rfid_measure_dt = 0.01;  % measure frequency

% Initial position and covariance of the robot
cov_x     = 0.5^2;
cos_theta = 0.1745^2;
initialCov = diag([cov_x,cov_x,cos_theta]);

% Odometry noise
add_odom_noise = true;
std_v     = 0.5/10;
std_omega = 0.17/10;

% Measure noise
add_measure_noise = true;
R_cov = (0.05)^2;

%% Parametri corridoio
larghezza_corr = 10;
lunghezza_corr = 100;

discretizz = 1:3;
discretizz = 4:6;
discretizz = 6:8;
discretizz = 9:10;
ls = discretizz.^(-1)*larghezza_corr;

% load percorsi
load('./data/percorsi')
% Main

for i=1:numel(ls)
    l = ls(i);
    total_RFID = [];
    
    % FIND THE RFID POSITION
    x_pos = 0;
    y_pos = 0;
    while x_pos<(lunghezza_corr+1e-4)
        while y_pos<(larghezza_corr+1e-4)
            total_RFID = [total_RFID, [x_pos;y_pos]];
            y_pos = y_pos + l;
        end
        y_pos = 0;
        x_pos = x_pos + l;
    end
    
    num_rfid = numel(total_RFID)/2;
    
    
    %% Define the path
    follow_path = true;
    real_traces = {};    
    filters_trace = {};  % traccia data dal filtro
    for p_ids = 1:path_num
        p_ids
        path = paths{1};
        initialPose = path(1:3,1);
        real_trace = [];
        
        %% Initialize a unicycle-like Robot
        tartufino = UnicycleBot(initialPose, dt);
        
        % Initialize UKF filters
        filters = {};
        for kk=1:num_filters
            noisy_initial_pose = mvnrnd(initialPose,initialCov);
            filters{kk,1} = UKF(noisy_initial_pose,initialCov);
            filters{kk,2} = ['ukf',num2str(kk)];
        end
        
        
        %% Main loop
        simulationTime = 0;
        last_measure = -inf;
        
        
        for kk=1:size(filters,1)
            filters_trace{i,kk} = [];
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
            for kk=1:size(filters,1)
                if add_odom_noise
                    measure_uCmd = mvnrnd(uCmd,cov_u)';
                else
                    measure_uCmd = uCmd;
                end
                filters{kk,1}.Prediction(measure_uCmd,dt,cov_u);
            end
            
            % Measure correction
            deltas_RFID = (total_RFID - tartufino.Pose(1:2));
            dist_sq = (deltas_RFID(1,:).*deltas_RFID(1,:) + deltas_RFID(2,:).*deltas_RFID(2,:));
            in_view = dist_sq < detection_range*detection_range;
            RFID = total_RFID(:,in_view);
            R = R_cov*eye(size(RFID,2),size(RFID,2)); % measure covariance
            if (simulationTime-last_measure)>rfid_measure_dt
                % Add noise to the measure
                ideal_measurement = rfidReadings(tartufino.Pose,uCmd(1),RFID);
                if ~isnan(ideal_measurement)
                    for kk=1:size(filters,1)
                        if add_measure_noise
                            measurement = mvnrnd(ideal_measurement, R)';
                            filters{kk,1}.UpdateRFID(measurement,R,RFID);
                        else
                            filters{kk,1}.UpdateRFID(ideal_measurement,R,RFID);
                        end
                    end
                end
                last_measure = simulationTime;
            end
            
            % keep track of the path
            real_trace(:,end+1) = tartufino.Pose;
            for kk=1:size(filters,1)
                filters_trace{p_ids,kk}(:,end+1) = filters{kk,1}.x;
            end
            
            % Update simulation time
            simulationTime = simulationTime + dt;
            
            
            h_waitbar = waitbar(simulationTime/t_stop);
            if tartufino.Pose(1)>100
                break;
            end
        end
        real_traces{p_ids} = real_trace;
        
        figure();
        ax = gca;
        hold on
        plot(ax,total_RFID(1,:),total_RFID(2,:),'s');
        plot(ax,real_traces{p_ids}(1,:),real_traces{p_ids}(2,:),'r');
        legenda = {};
        legenda{1} = 'RFID location';
        legenda{2} = 'real trace';
        for kk=1:size(filters,1)
            plot(ax,filters_trace{p_ids,kk}(1,:),filters_trace{p_ids,kk}(2,:));            
        end
        legend(legenda);
        axis equal;
        hold off

        close(h_waitbar);
        save(['./data/',num2str(l)],'filters_trace','real_trace');
    end
end


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