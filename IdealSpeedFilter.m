%
clc;
clear all;
close all;
addpath('./utility');
addpath(genpath('./utility/AlphaFilters/'));
rng('default'); % for repeatable result

%% Parameters
dt = 0.01;               % simulation time step
rfid_measure_dt = 0.01;  % measure frequency
plot_dt  = 1/5;      % seconds between frame


L = 0.4; % wheelbase
l = 5;   % rfid semidistance

% Initial position and covariance
initialPose = [-5 0  0 ]';
RFID = [0, 0;
    l, -l ]; % Position of RFID



%% Initialize a unicycle-like Robot
tartufino = UnicycleBot(initialPose, dt, L);
ax = gca;
hold(ax, 'on')
plot(ax,RFID(1,:),RFID(2,:),'.m','markersize',5);
h_circ = {};
h_circd1 = {};
h_circd2 = {};
colori = ['r','g','b','m'];
for k=1:4
    h_circ{k} = plot(ax,[1],[1],colori(k));
    h_circd1{k} = plot(ax,[1],[1],colori(k));
    h_circd2{k} = plot(ax,[1],[1],colori(k));
end
hold(ax, 'off')


%% Main loop
r = robotics.Rate(1/plot_dt);

reset(r);
simulationTime = 0;
last_measure = -inf;
last_plot = -inf;

ukf_trace = [];
while simulationTime < 500 % if time is not up
    % Generate motion command that is to be sent to the robot
    uCmd(1) = 1;  % linear velocity
    %     uCmd(2) = 1/3;    % angular velocity
    P_frenet = zeros(4,1); %find_frenet(path,tartufino.Pose);
    P_frenet(2) = 2;
    P_frenet(3) = 0*pi/2;
    rho = tartufino.path_folowing(P_frenet);
    
    rho = tartufino.path_folowing(P_frenet);
    uCmd(2) = 1; %rho*uCmd(1);  % angular velocity 1; %
    % Drive the real robot
    drive(tartufino, uCmd);
    
    %drive(tartufino, uCmd);
    
    % Measure processing
    if (simulationTime-last_measure)>rfid_measure_dt
        % Retrive the measure
        [ideal_measurement,ideal_derivativemeasurement] = rfidReadings(tartufino,uCmd,RFID);        

       last_measure = simulationTime;
        
        % Invert the measure:
        alpha_1 = acos(ideal_measurement(1)/uCmd(1));
        alpha_2 = acos(ideal_measurement(2)/uCmd(1));
        
        % Cases sign
        sign = [1,1;-1,-1;1,-1;-1,1];        
        for i=1:size(sign,1)
            alpha_1_i = sign(i,1)*alpha_1;
            alpha_2_i = sign(i,2)*alpha_2;
            delta = alpha_1_i -  alpha_2_i;
            T = tan(0.5*delta);
            
            if abs(delta)<1e-4
                continue;
            end
            % Extract circumference value
            R  = abs(0.5*l*(T^2+1)/T); 
            xc = 0.5*l*(T^2-1)/T;
            yc = 0;
            
            d1 = uCmd(1)*cos(alpha_1_i)/(ideal_derivativemeasurement(1) + sin(alpha_1_i)*uCmd(1)*uCmd(2));
            d2 = uCmd(1)*cos(alpha_2_i)/(ideal_derivativemeasurement(2) + sin(alpha_2_i)*uCmd(1)*uCmd(2));
             
%             if (simulationTime-last_plot)>plot_dt
                % Draw the circles
                dthetas = 0.1/R;                
                thetas = 0:dthetas:2*pi;
                % Measure circle
                h_circ{i}.XData = R*cos(thetas) + xc;
                h_circ{i}.YData = R*sin(thetas) + yc;
                
                % Derivative measure circle RFID 1
                dthetas = 0.1/d1;                
                thetas = 0:dthetas:2*pi;
                h_circd1{i}.XData = d1*cos(thetas) + RFID(1,1);
                h_circd1{i}.YData = d1*sin(thetas) + RFID(2,1);
                
                % Derivative measure circle RFID 1
                dthetas = 0.1/d2;                
                thetas = 0:dthetas:2*pi;
                h_circd2{i}.XData = d2*cos(thetas) + RFID(1,2);
                h_circd2{i}.YData = d2*sin(thetas) + RFID(2,2);
                
                
                tartufino.updatePlot([0;0;0],eye(3,3), simulationTime);
%                 axis auto
                xlim([-10,10]);
                ylim([-10,10]);
%                 axis equal;
                
%                 last_plot = simulationTime;
%                 waitfor(r);
%             end
            
        end
        legend off
        pause();
        
        
    end
    
    % Update plot
    if isempty(get(groot,'CurrentFigure')) % if figure is not prematurely killed
        break
    end
    
    % Update simulation time
    simulationTime = simulationTime + dt;
    
end



ax = gca;
hold(ax, 'on')
plot(ax,tartufino.Trace(1,:),tartufino.Trace(2,:),'r');
plot(ax,ukf_trace(1,:),ukf_trace(2,:),'k');
hold(ax, 'off')
% xlim([-10,10]);
% ylim([-10,10]);
axis auto;
