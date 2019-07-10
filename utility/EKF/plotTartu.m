function [handle] = plotTartu(f,s,P)

hs = findobj(f,'Tag','tartuf');
for h = hs
    delete(h)
end


ang_cov = 3*sqrt(P(3,3));
t = s(3);
vector = [0, cos(t); 0, sin(t)] * 0.4;
n = ceil(ang_cov/0.05);
angles = t + ang_cov*linspace(-1,1,2*n);

p = s(1:2);
fill(p(1)+[0,cos(angles),0]* 0.4,p(2)+[0,sin(angles),0]* 0.4,'y','Tag','tartuf')
handle = plot(p(1),p(2),'g.','markersize',20,'Tag','tartuf');
plot(p(1)+vector(1,:),p(2)+vector(2,:),'g--','Tag','tartuf');

h_tmp = error_ellipse(P(1:2,1:2),s(1:2),'conf',0.95);
set(h_tmp,'Tag','tartuf');
set(h_tmp,'color','b');
end

