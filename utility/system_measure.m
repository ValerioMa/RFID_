
function [h] = system_measure(p,RFID,v_norm)
%UPDATE_AGENT Summary of this function goes here
%   Detailed explanation goes here

measure_num = size(RFID,2);

% SLOW SOLUTION
h = zeros(measure_num,1); 

x_p = p(1);
y_p = p(2);
theta_p = p(3);

v = v_norm*[cos(theta_p); sin(theta_p)];

for i=1:measure_num
    x_rfid = RFID(1,i);
    y_rfid = RFID(2,i);
    
    dx = x_rfid - x_p;
    dy = y_rfid - y_p;
    
    normaliz = sqrt(dx*dx + dy*dy);
    if(normaliz<1e-3)
        h(i) = nan;
    else
% %         % SLOW VERSION
% %         versore = [dx;dy]/normaliz;        
% %         d_dot_norm = v'*versore;
% %         h(i) = d_dot_norm ;
        
        % FAST VERSION        
        h(i) = (v(1)*dx + v(2)*dy)/normaliz;        
    end
end

end

