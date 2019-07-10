function [] = plotParticles(particles)
%PLOTPARTICLES Summary of this function goes here
%   Detailed explanation goes here

thetas = particles(3,:);
direction = 0.1*[cos(thetas); sin(thetas)];
vector_x = [particles(1,:)', (particles(1,:) + direction(1,:))'];
vector_y = [particles(2,:)', (particles(2,:) + direction(2,:))'];

h = findobj(gca,'Tag','Particles');
for i = h
    delete(i)
end

plot(particles(1,:),particles(2,:),'b.','Tag','Particles','markersize',5);
% plot(vector_x',vector_y','k--','Tag','Particles');

end

