function new_particles = particle_correct(particles, measure,argv)
%PARTICLE_CORRECT Summary of this function goes here
%   Detailed explanation goes here
weights = compute_weights(particles, measure, argv);
% figure(2);
% plot(particles(1,:),weights,'*')
new_particles = resample(particles, weights);
end


function prob = probability_of_measurement(measurement, predicted_measurement)

%%%%%%%%%%%%%%   PARAMETERS  %%%%%%%%%%%%%%%%%%
meas_std = 1/180*pi;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
prob = 1;
for i=1:numel(measurement)
    prob = prob * gaussProb(predicted_measurement(i), measurement(i),meas_std);    
end
end


function p = gaussProb(value,mu,std)
v = -(value-mu)^2/(2*std^2);
p = exp(v);
end

function new_particles = resample(particles, weights)
% Find max weight
w_max = -1;
for w = weights
    if w>w_max
        w_max = w;
    end
end

p_number = size(particles,2);
indx = randi([1, p_number]);
new_particles = zeros(size(particles));
offset = 0;
for i = 1:p_number
    offset = offset + rand()*w_max;
    while offset > weights(indx)
        offset = offset - weights(indx);
        indx = rem((indx ),p_number) + 1;
    end
    new_particles(:,i) = particles(:,indx);
end
end


function weights = compute_weights(particles, measure,argv)
part_num = size(particles,2);
weights = zeros(1,part_num);

for i=1:part_num  
    p = particles(:,i);
    p_measure = system_measure(p,argv);
    weights(i) = probability_of_measurement(measure, p_measure);    
end
end