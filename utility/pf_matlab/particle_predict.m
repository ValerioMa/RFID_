function new_particles = particle_predict(particles, control)
%PREDICT Summary of this function goes here
%   Detailed explanation goes here

%%%%%%%%%
w = 0.4;  % with of the veicle
control_motion_factor = 0.7; %0.40;  
control_turn_factor   = 0.3; %0.50; 
%%%%%%%%%%

ds = control(1);
dtheta = control(2);

lft = ds - dtheta*w/2;
rgt = ds + dtheta*w/2;

sigma_s = abs(control_motion_factor*ds);
sigma_r = abs(control_turn_factor*dtheta*w/2);
cov_v = sigma_s*sigma_s + sigma_r*sigma_r;

A = [0.5 0.5;
     -w   w];
 
cov = A*eye(2,2)*cov_v*A';


new_particles = zeros(size(particles));
for i=1:size(particles,2)
   %p_control = [ds + sigma_s*randn(1);dtheta + sigma_r*randn(1)]; 
   p_control =  mvnrnd([ds;dtheta],cov)';
   new_particles(:,i) = system_dynamic(particles(:,i),p_control);
end


end