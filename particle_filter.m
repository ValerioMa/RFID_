clc;
clear all;
close all force;

addpath(genpath([pwd,'/utility']));

%% Parameters

particle_num = 5000;
v = 1;
omega = 0;
p0 = [0;-5;pi/3];


RFID = [0, 0; 
        5, -5 ];

dt = 0.1;
sim_T = 100.0;


%% Simulation

% Initialize the particles
gaussian_init = false;
if gaussian_init
    pos_cov = 0.01^2;
    theta_cov = (1*pi/180)^2;    
    state0_cov = 0*[pos_cov,       0,  0;
                        0, pos_cov,  0;
                        0,       0,  theta_cov];
    particles = mvnrnd(p0,state0_cov,particle_num)';
else
    state_size = [-2,2; -2, 2; -pi, pi];    
    particles = zeros(3,particle_num);
    delta = diff(state_size')';
    min_v = state_size(:,1);
    random_number = rand(3,particle_num);
    for i=1:particle_num
        particles(:,i) = min_v + delta.*random_number(:,i);    
    end
    
end
particles_init = particles;
t = 0;
rel_agent = p0;

figure(1);clf;
hold on;
axis equal;
control = [v;omega]*dt;
plotParticles(particles)
plotAgent(rel_agent,control,[],RFID)
plot(RFID(1,:),RFID(2,:),'sqr','markersize',10,'MarkerFaceColor','r');           
drawnow();



while t<sim_T
    % Move real agent and particles
    control = [v;omega]*dt;
    rel_agent = system_dynamic(rel_agent,control);
    particles = particle_predict(particles, control);   
%     particles(3,:) = 0;
    %particles(:,1:50) = rel_agent(:,ones(1,50));
    % Get the measurement from real agent and update particles
    measure   = system_measure(rel_agent,RFID);   
    %TODO: sporcare misura con rumore!!
    particles = particle_correct(particles, measure, RFID);
        
    % Plot particle movements 
    figure(1);
    plotParticles(particles) 
    plotAgent(rel_agent,control,measure,RFID);   
    drawnow();
            
    t = t + dt; 
end