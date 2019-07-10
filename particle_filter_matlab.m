clc;
clear all;
close all force;

addpath(genpath([pwd,'/utility/pf_matlab']));


%% Parameters
dt = 0.01; % time step
L = 0.4; % wheelbase
initialPose = [0 15  -pi/2 ]';

RFID = [0, 0, 5; 
        5, -5, 5 ]; % Position of RFID
rfid_measure_dt = 0.01;

%% Initialize a unicycle-like Robot
rng('default'); % for repeatable result

tartufino = UnicycleBot(initialPose, dt, L);
hold on;
h_part = scatter(0,0,'b');
h_part.Marker = '.';
uistack(h_part,'bottom');
hold off;

%% Set up the Particle Filter
pf = robotics.ParticleFilter;
pf1 = robotics.ParticleFilter;
pf2 = robotics.ParticleFilter;

n_particles = 500000;
initPose1 = [initialPose(1:3);0]';
initPose2 = initPose1;
initPose2(1) = - initPose1(1);
initPose2(3) = pi - initPose1(3);
pf1.initialize(n_particles, initPose1, [eye(3,4)*0.5;zeros(1,4)], 'CircularVariables',[0 0 1 0]);

pf2.initialize(n_particles, initPose2, [eye(3,4)*0.5;zeros(1,4)], 'CircularVariables',[0 0 1 0]);
%pf.initialize(2*n_particles, zeros(1,4), [eye(3,4)*10;zeros(1,4)], 'CircularVariables',[0 0 1 0]);
% pf.Particles(1:n_particles,:) = pf1.Particles;
% pf.Particles(n_particles+1:end,:) = pf2.Particles;
pf.initialize(n_particles, [-20,20;-20,20;-pi,pi;-1,1], 'CircularVariables',[0 0 1 0]);
%pf.StateEstimationMethod = 'maxweight';
% pf.ResamplingMethod = 'systematic';
pf.StateEstimationMethod = 'maxweight';
pf.ResamplingMethod = 'residual';



% StateTransitionFcn defines how particles evolve without measurement
pf.StateTransitionFcn = @stateTransition;

% MeasurementLikelihoodFcn defines how measurement affect the our estimation
pf.MeasurementLikelihoodFcn = @(pf, predictParticles, measurement)measurementLikelihood(pf, predictParticles, measurement,RFID);

% Last best estimation for x, y and theta
lastBestGuess = [0 0 0];




%% Main loop
r = robotics.Rate(1/dt);

reset(r);

simulationTime = 0; 

last_measure = -inf;
while simulationTime < 20 % if time is not up
    % Generate motion command that is to be sent to the robot
    uCmd(1) = 1;  % linear velocity
    
    % Fake P_frenet
    P_frenet = zeros(4,1); %find_frenet(path,tartufino.Pose);
    P_frenet(1) = initialPose(1); 
    P_frenet(2) = initialPose(2); 
    P_frenet(3) = initialPose(3);

    % P_frenet from path
%     delta = path(1:2,:) - punto;
%     [v,idx] = min(delta(1,:).^2+delta(2,:).^2);
%     P_frenet = path(:,idx(end));    
    rho = tartufino.path_folowing(P_frenet);
    uCmd(2) = rho*uCmd(1);  % angular velocity
    
    % Drive the real robot
    drive(tartufino, uCmd);
    
    
    % Predict the carbot pose based on the motion model
    [statePred, covPred] = pf.predict(dt, uCmd);
    

    % Measure correction
    if (simulationTime-last_measure)>rfid_measure_dt
       measurement = rfidReadings([tartufino.Pose;uCmd(1)],RFID);
       [stateCorrected, covCorrected] = pf.correct(measurement');
       
%        [circles] = invert_measure(measurement/uCmd(1),RFID);
       
       last_measure = simulationTime;
    else
        stateCorrected = statePred;
        covCorrected = covPred;      
    end
        
    
    lastBestGuess = stateCorrected(1:3);
    
    % Update plot
    if ~isempty(get(groot,'CurrentFigure')) % if figure is not prematurely killed
        tartufino.updatePlot(lastBestGuess,eye(3,3)*0.001, simulationTime);       
        h_part.XData = pf.Particles(:,1);
        h_part.YData = pf.Particles(:,2);        
    else
        break
    end
    
    waitfor(r);
    
    % Update simulation time
    simulationTime = simulationTime + dt;
    axis auto;
%     pause();
end
