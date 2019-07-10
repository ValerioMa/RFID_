function [] = updatePlot(f2,map,p_predict,d_meas,p_update,dx,dy,dtheta,area,n_discretizz_theta,rel_agent,RFID,z_idx)
%UPDATEPLOT Summary of this function goes here
%   Detailed explanation goes here


z_idx = round(z_idx);
pl_theta = (z_idx-1)*dtheta;
ver2 = [cos(pl_theta),sin(pl_theta)];
pl_theta*180/pi

ag_x = rel_agent(1)/dx;
ag_y = rel_agent(2)/dy;
ag_theta = rel_agent(3);
ver = [cos(ag_theta),sin(ag_theta)];

clf;
subplot(2,2,1);
hold on;
title('Initial pdf');
%map.show(f2,z_idx); 
map.show(f2); 
xlim(area(1,:)/dx); ylim(area(2,:)/dy); %axis equal;
z_lim = zlim();
plot3(ag_x*[1,1],ag_y*[1,1],z_lim,'r*-','linewidth',4)
plot3(ag_x+[0,ver(1)],ag_y+[0,ver(2)],z_lim(1)*[1,1],'g.-','linewidth',4)
plot3(ag_x+[0,ver(1)],ag_y+[0,ver(2)],z_lim(2)*[1,1],'g.-','linewidth',4)
% title('Orientation');
% plot(0+[0,ver2(1)],0+[0,ver2(2)],'g.-','linewidth',4)
% xlim([-1,1])
% ylim([-1,1])
%    d_meas.show(f2);  xlim(area(1,:)/dx); ylim(area(2,:)/dy); axis equal;

subplot(2,2,2);
hold on;
title('Prediction');
% p_predict.show(f2,z_idx); 
p_predict.show(f2); 
xlim(area(1,:)/dx); ylim(area(2,:)/dy); %axis equal;
z_lim = zlim();
plot3(ag_x*[1,1],ag_y*[1,1],z_lim,'r*-','linewidth',4)
plot3(ag_x+[0,ver(1)],ag_y+[0,ver(2)],z_lim(1)*[1,1],'g.-','linewidth',4)
plot3(ag_x+[0,ver(1)],ag_y+[0,ver(2)],z_lim(2)*[1,1],'g.-','linewidth',4)

subplot(2,2,3);
hold on;
title('Measure');
% d_meas.show(f2,z_idx);  
d_meas.show(f2);  
xlim(area(1,:)/dx); ylim(area(2,:)/dy); %axis equal;
z_lim = zlim();
plot3(ag_x*[1,1],ag_y*[1,1],z_lim,'r*-','linewidth',4)
plot3(ag_x+[0,ver(1)],ag_y+[0,ver(2)],z_lim(1)*[1,1],'g.-','linewidth',4)
plot3(ag_x+[0,ver(1)],ag_y+[0,ver(2)],z_lim(2)*[1,1],'g.-','linewidth',4)
plot3(RFID(1,:)/dx,RFID(2,:)/dy,z_lim(2)*[1,1],'sqr','markersize',10,'MarkerFaceColor','y'); 

subplot(2,2,4);
hold on;
title('Update');
p_update.show(f2);%,z_idx);
xlim(area(1,:)/dx); ylim(area(2,:)/dy); %axis equal;
z_lim = zlim();
if abs(rel_agent(3)-pl_theta)<dtheta
    plot3(ag_x*[1,1],ag_y*[1,1],z_lim,'r*-','linewidth',4)
    plot3(ag_x+[0,ver(1)],ag_y+[0,ver(2)],z_lim(1)*[1,1],'g.-','linewidth',4)
    plot3(ag_x+[0,ver(1)],ag_y+[0,ver(2)],z_lim(2)*[1,1],'g.-','linewidth',4)
end

b = uicontrol('Parent',f2,'Style','slider',...
   'SliderStep', [1/n_discretizz_theta, 0.1], ...
   'Position',[81,54,419,23],...
   'value',z_idx, 'min',1, 'max',n_discretizz_theta);
b.Callback = @(es,ed) updatePlot(f2,map,p_predict,d_meas,p_update,dx,dy,dtheta,area,n_discretizz_theta,rel_agent,RFID,es.Value); 

end

