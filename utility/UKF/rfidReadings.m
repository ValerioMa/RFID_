
function [h] = rfidReadings(state,v_norm,RFID)
%UPDATE_AGENT Summary of this function goes here
%   Detailed explanation goes here

measure_num = size(RFID,2);
particle_num = size(state,2);

% SLOW SOLUTION
h = zeros(measure_num,particle_num); 

x_p = state(1,:);
y_p = state(2,:);
theta_p = state(3,:);
% v_norm = 0.8; %state(4,:);

v = v_norm.*[cos(theta_p); sin(theta_p)];

for i=1:measure_num
    x_rfid = RFID(1,i);
    y_rfid = RFID(2,i);
    
    dx = x_rfid - x_p;
    dy = y_rfid - y_p;
    
    normaliz = sqrt(dx.*dx + dy.*dy);
    normaliz(normaliz<1e-4) = nan;

% %         % SLOW VERSION
% %         versore = [dx;dy]/normaliz;        
% %         d_dot_norm = v'*versore;
% %         h(i) = d_dot_norm ;
        
    % FAST VERSION        
    h(i,:) = (v(1,:).*dx + v(2,:).*dy)./normaliz;        

end

end

