function [] = plotAgent(rel_agent,control,measure,RFID)
%PLOTPARTICLES Summary of this function goes here
%   Detailed explanation goes here

thetas = rel_agent(3,:);
direction = control(1)*[cos(thetas); sin(thetas)];
vector_x = [rel_agent(1,:)', (rel_agent(1,:) + direction(1,:))'];
vector_y = [rel_agent(2,:)', (rel_agent(2,:) + direction(2,:))'];

h = findobj(gca,'Tag','relAgent');
for i = h
    delete(i)
end

plot(rel_agent(1,:),rel_agent(2,:),'r.','Tag','relAgent','markersize',20);
plot(vector_x',vector_y','r-','Tag','relAgent','linewidth',3);

for i=1:numel(measure)
    dx = RFID(1,i) - rel_agent(1);
    dy = RFID(2,i) - rel_agent(2);
    versor = [dx;dy]/sqrt(dx*dx+dy*dy);
    d_dot_norm = control(1)*cos(measure(i));
    d_dot = d_dot_norm*versor;
    plot(rel_agent(1) + [0,d_dot(1)],rel_agent(2) + [0,d_dot(2)],'g-','Tag','relAgent','linewidth',2);
    plot(rel_agent(1) + [0,dx],rel_agent(2) + [0,dy],'g--','Tag','relAgent','linewidth',1);
end

end

