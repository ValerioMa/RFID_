
function [h,h2] = system_measure_derivative(p,RFID,v_norm,omega)
%UPDATE_AGENT Summary of this function goes here
%   Detailed explanation goes here

% % % syms x_p y_p theta_p x_rfid y_rfid v_norm omega
% % % 
% % % v = v_norm*[cos(theta_p); sin(theta_p)];
% % % dx = x_rfid - x_p;
% % % dy = y_rfid - y_p;
% % % normaliz = sqrt(dx*dx + dy*dy);
% % % measure = (v(1)*dx + v(2)*dy)/normaliz;       
% % % 
% % % m_dx = diff(measure,x_p);
% % % m_dy = diff(measure,y_p);
% % % m_dtheta = diff(measure,theta_p);
% % % dmeasures = [m_dx,m_dy,m_dtheta];
% % % speed = [cos(theta_p)*v_norm ;sin(theta_p)*v_norm ;omega];
% % % increments = dmeasures*speed;
% % % 
% % % m_dxdx = diff(increments,x_p);
% % % m_dydy = diff(increments,y_p);
% % % m_dthetadtheta = diff(increments,theta_p);
% % % ddmeasures = [m_dxdx,m_dydy,m_dthetadtheta];
% % % increments2 = ddmeasures*speed;

measure_num = size(RFID,2);

% SLOW SOLUTION
h = zeros(measure_num,1); 
h2 = zeros(measure_num,1); 


x_p = p(1);
y_p = p(2);
theta_p = p(3);

% v = v_norm*[cos(theta_p); sin(theta_p)];

for i=1:measure_num
    x_rfid = RFID(1,i);
    y_rfid = RFID(2,i);
    
    dx = x_rfid - x_p;
    dy = y_rfid - y_p;
    
    normaliz = sqrt(dx*dx + dy*dy);
    if(normaliz<1e-4)
        h(i) = 0;
    else
% %         % SLOW VERSION
% %         versore = [dx;dy]/normaliz;        
% %         d_dot_norm = v'*versore;
% %         h(i) = d_dot_norm ;
        
        % FAST VERSION        
        
%         m_dx = ((2*x_p - 2*x_rfid)*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (v_norm*cos(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2);
%         m_dy = ((2*y_p - 2*y_rfid)*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (v_norm*sin(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2);
%         m_dtheta = -(v_norm*cos(theta_p)*(y_p - y_rfid) - v_norm*sin(theta_p)*(x_p - x_rfid))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2);
        
        
        h(i) = v_norm*cos(theta_p)*(((2*x_p - 2*x_rfid)*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (v_norm*cos(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2)) + v_norm*sin(theta_p)*(((2*y_p - 2*y_rfid)*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (v_norm*sin(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2)) - (omega*(v_norm*cos(theta_p)*(y_p - y_rfid) - v_norm*sin(theta_p)*(x_p - x_rfid)))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2);
                
        h2(i) = omega*(v_norm*cos(theta_p)*(((2*x_p - 2*x_rfid)*(v_norm*cos(theta_p)*(y_p - y_rfid) - v_norm*sin(theta_p)*(x_p - x_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) + (v_norm*sin(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2)) - v_norm*sin(theta_p)*(((2*x_p - 2*x_rfid)*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (v_norm*cos(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2)) + v_norm*cos(theta_p)*(((2*y_p - 2*y_rfid)*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (v_norm*sin(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2)) + v_norm*sin(theta_p)*(((2*y_p - 2*y_rfid)*(v_norm*cos(theta_p)*(y_p - y_rfid) - v_norm*sin(theta_p)*(x_p - x_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (v_norm*cos(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2)) + (omega*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2)) + v_norm*cos(theta_p)*(v_norm*cos(theta_p)*((v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2) - (3*(2*x_p - 2*x_rfid)^2*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(4*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(5/2)) + (v_norm*cos(theta_p)*(2*x_p - 2*x_rfid))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) + v_norm*sin(theta_p)*((v_norm*cos(theta_p)*(2*y_p - 2*y_rfid))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (3*(2*x_p - 2*x_rfid)*(2*y_p - 2*y_rfid)*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(4*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(5/2)) + (v_norm*sin(theta_p)*(2*x_p - 2*x_rfid))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2))) + (omega*v_norm*sin(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2) + (omega*(2*x_p - 2*x_rfid)*(v_norm*cos(theta_p)*(y_p - y_rfid) - v_norm*sin(theta_p)*(x_p - x_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2))) + v_norm*sin(theta_p)*(v_norm*sin(theta_p)*((v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2) - (3*(2*y_p - 2*y_rfid)^2*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(4*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(5/2)) + (v_norm*sin(theta_p)*(2*y_p - 2*y_rfid))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) + v_norm*cos(theta_p)*((v_norm*cos(theta_p)*(2*y_p - 2*y_rfid))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (3*(2*x_p - 2*x_rfid)*(2*y_p - 2*y_rfid)*(v_norm*cos(theta_p)*(x_p - x_rfid) + v_norm*sin(theta_p)*(y_p - y_rfid)))/(4*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(5/2)) + (v_norm*sin(theta_p)*(2*x_p - 2*x_rfid))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2))) + (omega*(2*y_p - 2*y_rfid)*(v_norm*cos(theta_p)*(y_p - y_rfid) - v_norm*sin(theta_p)*(x_p - x_rfid)))/(2*((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(3/2)) - (omega*v_norm*cos(theta_p))/((x_p - x_rfid)^2 + (y_p - y_rfid)^2)^(1/2));
    end
end

end