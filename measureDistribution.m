clc;
clear all;
close all;
% addpath(genpath('./utility'))

v = 1;
l = 5;   % rfid semidistance
RFID = [0, 0
        l, -l]; % Position of RFID
    
measure = [
   -0.3162
   -0.9487
   ];

dx = 0.1;
dy = 0.1;
dtheta = pi/90;
p_start = [-15;-10;0];
p_stop  = [ 0; 10; 2*pi];

delta = p_stop - p_start;
delta_n(1) = ceil(delta(1)/dx)+1;
delta_n(2) = ceil(delta(2)/dy)+1;
delta_n(3) = ceil(delta(3)/dtheta)+1;
delta = delta_n;

values = zeros(delta(1),delta(2),delta(3));
xs = (1:delta(1))*dx + p_start(1);
xs = [xs,-flip(xs)];
ys = (1:delta(2))*dy + p_start(2);
ts = (1:delta(3))*dtheta + p_start(3);
for x_i=1:delta(1)
    x_i
    x = x_i*dx + p_start(1);    
    for y_i = 1:delta(2)
        y = y_i*dy + p_start(2);
        for z_i = 1:delta(3)
            z = z_i*dtheta + p_start(3);
            p = [x;y;z];
            prob = 0;
            for xa = (0:1:2)*dx/2
                for ya = (0:1:2)*dy/2
                    for za = (0:1:2)*dtheta/2
                        p1 = p + [xa;ya;za];
                        pred_measure = system_measure(p,RFID,v);
                        if isnan(pred_measure)
                           continue; 
                        end
                        prob = max(prob,measure_prob(measure, pred_measure)); %/8; % si puo normalizzare dopo non serve farlo ora
                    end
                end
            end                
            values(x_i,y_i,z_i) = prob;
        end
    end
end

somma = sum(values(:,:,:),3)';
tmp2 = flip(somma,2);
somma = cat(2,somma,tmp2);

% Calcolo volume
p_start = [-15;-10;0];
p_stop  = [ 15; 10; 2*pi];
delta = p_stop - p_start;
volume = delta(1)*delta(2)*delta(3);
somma = somma/(sum(sum(sum(somma))));
somma = somma/volume;
%% plot
close all;
figure();
hold on
s = surf(xs,ys,somma);
s.FaceAlpha = 1;
s.LineStyle = ':';
s.FaceColor = 'interp';
% s.EdgeColor = 'flat';
% s.FaceLighting = 'none';
xlabel('x [m]')
ylabel('y [m]')
zlabel('measure pdf []');

load('circles')
maxv = max(max(somma));
for i=1:numel(xcs)
    xc = xcs{i};
    yc = ycs{i};
    z = maxv * ones(size(xc));
    plot3(xc,yc,z,'r','linewidth',3);
end
%%
% contour(sum(values(:,:,:),3)')




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



function prob = measure_prob(measurement, predicted_measurement)

%%%%%%%%%%%%%%   PARAMETERS  %%%%%%%%%%%%%%%%%%
meas_std = 0.05;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gaussian_exponent = 0;
for i=1:numel(measurement)
    gaussian_exponent = gaussian_exponent + gaussProbExponent(predicted_measurement(i), measurement(i),meas_std);    
end
prob = exp(gaussian_exponent);
end


function v = gaussProbExponent(value,mu,std)
v = -(value-mu)^2/(2*std^2);
end