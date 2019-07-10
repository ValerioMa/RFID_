%
clc;
clear all;
close all;
addpath('./utility');
addpath(genpath('./utility/UKF/'));
rng('default'); % for repeatable result

%% Parameters
dt = 0.01;               % simulation time step
rfid_measure_dt = 0.01;  % measure frequency
plot_dt  = 1/5;      % seconds between frame


L = 0.4; % wheelbase
d = 5;   % rfid semidistance

number_ukf = 100;
% Initial position and covariance
%initialPose = [0 25  -pi/2]';
initialPose = [0 25 -pi/2]';
cov_x     = 0.05^2;
cov_y     = 0.05^2;
cos_theta = 0.2^2;

initialCov = diag([cov_x,cov_y,cos_theta]);
rfid_num = 3;
RFID = [0, 0 , d, -d
        d, -d, 0, 0]; % Position of RFID

RFID = RFID(:,1:rfid_num);
std_v     = 0.05;
std_omega = 0.17;
R = diag(ones(1,numel(RFID)/2)*0.05^2); % measure covariance
%% Initialize a unicycle-like Robot
tartufino = UnicycleBot(initialPose, dt, L);
ax = gca;
hold(ax, 'on') 
plot(ax,RFID(1,:),RFID(2,:),'.m','markersize',5);
hold(ax, 'off') 

for i=1:number_ukf
    initialPose_noisy = mvnrnd(initialPose,initialCov);
    ukf(i) = UKF(initialPose_noisy, initialCov);
end


%% Main loop
r = robotics.Rate(1/plot_dt);

reset(r);
simulationTime = 0;
last_measure = -inf;
last_plot = -inf;

n = 1;
t = 0;
real_trace = initialPose;
for i=1:number_ukf  
    ukfs_trace.x(i,n) = ukf(i).x(1);
    ukfs_trace.y(i,n) = ukf(i).x(2);
    ukfs_trace.t(i,n) = ukf(i).x(3);
end
    
% for i=1:number_ukf 
%     n = 1;
%     real_trace = [];
%     simulationTime = 0;
ideal_measurements = [];
while simulationTime < 50 % if time is not up
    punto = tartufino.Pose(1:2);
%     delta = path(1:2,:) - punto;
%     [v,idx] = min(delta(1,:).^2+delta(2,:).^2);
%     P_frenet = path(:,idx(end));

    % Generate motion command that is to be sent to the robot
    uCmd(1) = 1;  % linear velocity

    P_frenet = zeros(4,1); %find_frenet(path,tartufino.Pose);
    P_frenet(1) = initialPose(1); 
    P_frenet(2) = initialPose(2); 
    P_frenet(3) = initialPose(3);

    rho = tartufino.path_folowing(P_frenet);
    uCmd(2) = rho*uCmd(1);  % angular velocity
    % Drive the real robot
    drive(tartufino, uCmd);
    
    % Compute the measure and predict
    cov_u = [std_v^2, 0; 0, std_omega^2];      
    for i=1:number_ukf       
        measure_uCmd = mvnrnd(uCmd,cov_u)'; 
        ukf(i).Prediction(measure_uCmd,dt,cov_u);           
    end
    
    % Measure correction
%     if (simulationTime-last_measure)>=rfid_measure_dt
%if abs(tartufino.Pose(2)-d)>1 && abs(tartufino.Pose(2)+d)>1
       ideal_measurement  = rfidReadings(tartufino.Pose,uCmd(1),RFID);       
       ideal_measurements = [ideal_measurements,ideal_measurement(:)];       
       for i=1:number_ukf    
           measurement = mvnrnd(ideal_measurement, R)';
           ukf(i).UpdateRFID(measurement,R,RFID);       
       end
       last_measure = simulationTime; 
%end
%     end
        
    
    n = size(ukfs_trace.x,2) + 1;
    for i=1:number_ukf  
        ukfs_trace.x(i,n) = ukf(i).x(1);
        ukfs_trace.y(i,n) = ukf(i).x(2);
        ukfs_trace.t(i,n) = ukf(i).x(3);
    end
    
    real_trace(:,end+1) = tartufino.Pose;
    t(end+1) = simulationTime;
% %     if (simulationTime-last_plot)>plot_dt
% %         tartufino.updatePlot(ukf.x,ukf.P, simulationTime);
% %         axis auto
% %         xlim([-10,10]);
% %         ylim([-10,10]);
% %         axis equal;
% %         last_plot = simulationTime;
% %         waitfor(r);  
% %     end
    % Update simulation time
    simulationTime = simulationTime + dt;
    
%     if(tartufino.Pose(1)>5)
%         break;
%     end
end

for i=1:number_ukf
    figure(1); clf;
    hold on; grid on; box;
    plot(RFID(1,:),RFID(2,:),'sqm','markersize',10);
    plot(real_trace(1,:),real_trace(2,:),'r');
    plot(ukfs_trace.x(i,:),ukfs_trace.y(i,:),'--k');
    pause();    
end
% % plot(ukf2_trace(1,:),ukf2_trace(2,:),'--g');
% plot(ukf3_trace(1,:),ukf3_trace(2,:),'--b');
% axis equal;
% axis auto
% ax = gca;
% hold(ax, 'on') 
% plot(ax,tartufino.Trace(1,:),tartufino.Trace(2,:),'r');
% plot(ax,ukf_trace(1,:),ukf_trace(2,:),'k');
% plot(ax,ukf_trace_specular(1,:),ukf_trace_specular(2,:),'g');
% plot(ax,ukf_trace_odom(1,:),ukf_trace_odom(2,:),'b');
% 
% hold(ax, 'off') 
% % xlim([-10,10]);
% % ylim([-10,10]);
% axis auto;

%%

delta_x = ukfs_trace.x - real_trace(1,:);
delta_y = ukfs_trace.y - real_trace(2,:);
delta_t = ukfs_trace.t - real_trace(3,:);
% Normalize delta_t
for i=1:numel(delta_t)
   while  delta_t(i)>pi
       delta_t(i) = delta_t(i)-2*pi;
   end   
   while  delta_t(i)<-pi
       delta_t(i) = delta_t(i)+2*pi;
   end
end

m_delta = [mean(delta_x);mean(delta_y);mean(delta_t)];
c_delta = zeros(size(m_delta));
for i=1:numel(m_delta)/3
    c_delta(1,i) = std(delta_x(:,i));
    c_delta(2,i) = std(delta_y(:,i));
    c_delta(3,i) = std(delta_t(:,i));
end

lamb = 1;
figure(2);
hold on;
yyaxis left
x1_h = plot(t,m_delta(1,:),'-k');
y1_h = plot(t,m_delta(2,:),'--k');
x1p_h = plot(t,m_delta(1,:) + lamb*c_delta(1,:),'-k');
y1p_h = plot(t,m_delta(2,:) + lamb*c_delta(2,:),'--k');
x1p_h = plot(t,m_delta(1,:) - lamb*c_delta(1,:),'-k');
y1p_h = plot(t,m_delta(2,:) - lamb*c_delta(2,:),'--k');
yyaxis right
t1_h = plot(t,m_delta(3,:),'-.b');
t1p_h = plot(t,m_delta(3,:) + lamb*c_delta(3,:),'-.b');
t1p_h = plot(t,m_delta(3,:) - lamb*c_delta(3,:),'-.b');
legend([x1_h,y1_h,t1_h],{'x 1','y 1','\theta 1'});
hold off;

