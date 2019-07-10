clc;
clear all;
close all force;

%delete(gcp('nocreate'));
if (isempty(gcp('nocreate')))
    parpool()
%     parpool('local',4)
end



addpath(genpath([pwd,'/utility']));

%% Parameters
v = 1;
omega = 0;

sigmaV = 0.5;
sigmaOmega = pi/10;

p0 = [-4;1;pi/4]; % starting position

RFID = [0, 0; 
        -3, 3];

dt = 0.1;
sim_T = 8.0;

% Discretizzazioni istogramma
n_discretizz_x = 101;
n_discretizz_y = 101;
n_discretizz_theta = 360/5+1;

% Area istogramma
area = [
    -5,  5;   % x span
    -5,  5;   % y span
    0,  2*pi; % z span
];

% Lunghezza discretizzazioni
dx = (area(1,2)-area(1,1))/(n_discretizz_x-1);
dy = (area(2,2)-area(2,1))/(n_discretizz_y-1);
dtheta = (area(3,2)-area(3,1))/n_discretizz_theta;

%% Simulation

% Initialize the map histogram
values = ones(n_discretizz_x,n_discretizz_y,n_discretizz_theta);
start = [area(1)/dx;area(2)/dy;area(3)/dtheta];
%values(7:14,7:14,:) = 1;
map = Distribution_3D(start,values);
map_start = map.start();
map_stop  = map.stop();


% % % % First plot of the map
% % % f_id = figure(1);clf;
% % % hold on;
% % % axis equal;
% % % map.show(f_id);  
% % % drawnow();

% Create array to store the prediction from the different theta planes
con_ti_matrix = cell(n_discretizz_theta,1); %Distribution_3D([0;0;0],[]);
%for i=2:n_discretizz_theta
%   con_ti_matrix (i) =  Distribution_3D([0;0;0],[]);    % fake initialization
%end

% Position of real agent
rel_agent = p0;
t = 0;
p_update = map;
tic 
f2 = figure('units','normalized','Position',[0 0 1 1]);
fig_idx = 1;
while t<sim_T
    t
    if t>0
        % Move real agent and particles
        control = [v;omega]*dt;
    else
        control = [0;0];
    end
    % Compute the real displacement
    rel_agent = system_dynamic(rel_agent,control);
    
    % Get the measurement from real agent and update particles
    measure   = system_measure(rel_agent,RFID,v);   
    
    % Displacement distribution   
    map_dim = size(map.values);
    map = p_update;
    

    values_tot = p_update.values;
    start = p_update.start();
    start = start(1:2);
    %p_predict = Distribution_3D([0;0;0],values);
   parfor t_i = 1:n_discretizz_theta       
%   for t_i = 1:n_discretizz_theta       
       theta = dtheta*(t_i-1);
%        v_theta = exp(-0.5*(d_value )'/cov*(d_value));       
%        R = [cos(theta),0;sin(theta),0;0,1];
%        cov_ctrl = [1,0,0,1];
%        cov = R*cov_ctrl*R';

       % Movement mean value and covariance converted in the custom
       % discretiz
       cov = 2*eye(3,3);
%        cov(3,3) = 0.0001;
       move_x     = control(1)*cos(theta)/dx;
       move_y     = control(1)*sin(theta)/dy;       
       move_theta = control(2)/dtheta;
       
                    
       % Retrive pdf for that theta
       values = zeros(map_dim(1),map_dim(2),1);
       values(:,:,1) = values_tot(:,:,t_i); %p_update.values(:,:,t_i);
       
       zstart = t_i-1;
       d_p = Distribution_3D([start;zstart],values);
       
       % Compute movement pdf
       d_mov =  Distribution_3D.gaussian([move_x;move_y;move_theta],cov, [1;1;1]);
       
       % Convolve the PDF for theta=ti plane
%        conv_ti = Distribution_3D.convolve(d_p, d_mov);
%        %conv_ti.wrap_angle(n_discretizz_theta);
%        %conv_ti.cut(map_start,map_stop);
        con_ti_matrix{t_i} = Distribution_3D.convolve(d_p, d_mov); 
       
%        figure(1); clf;
%        subplot(2,2,1);
%        surf(values)
%        subplot(2,2,2);
%        surf(con_ti_matrix{t_i}.values(:,:,1))       
%        subplot(2,2,3);
%        surf(con_ti_matrix{t_i}.values(:,:,2))       
%        subplot(2,2,4);
%        surf(con_ti_matrix{t_i}.values(:,:,3))
%        f2 = figure(2);
%        clf; d_p.show(f2);  xlim(area(1,:)/dx); ylim(area(2,:)/dy); axis equal;
%        pause; 
%        clf; con_ti_matrix{t_i}.show(f2); xlim(area(1,:)/dx); ylim(area(2,:)/dy); axis equal;
%        pause;
    end

    p_predict = Distribution_3D.sum(con_ti_matrix);
    p_predict.wrap_angle(n_discretizz_theta);
    p_predict.cut(map_start,map_stop);
    % Measure weight
    p_start = p_predict.start();
    p_stop  = p_predict.stop();
    delta   = p_stop-p_start+1;
    values = zeros(delta(1),delta(2),delta(3));
    for x_i=p_start(1):p_stop(1)
        x = x_i*dx;
        for y_i = p_start(2):p_stop(2)
            y = y_i*dy;
            for z_i = p_start(3):p_stop(3)
                z = z_i*dtheta;
                p = [x;y;z];
                prob = 0;
                for xa = (0:1:2)*dx/2
                    for ya = (0:1:2)*dy/2
                        for za = (0:1:2)*dtheta/2
                            p1 = p + [xa;ya;za];
                            pred_measure = system_measure(p,RFID,v);
                            prob = max(prob,measure_prob(measure, pred_measure)); %/8; % si puo normalizzare dopo non serve farlo ora
                        end
                    end
                end                
                values(x_i-p_start(1)+1,y_i-p_start(2)+1,z_i-p_start(3)+1) = prob;
            end
        end
    end
    
    % Create the measurement PDF
    d_meas =  Distribution_3D(p_start,values);
    test = (p_predict.values.*d_meas.values);
    p_update = Distribution_3D(p_start,test);
    %p_update = Distribution_3D.multilpy(p_predict,d_meas);
    
%    if t>0
%      pause;   
%    end
   
   clf(f2);
   updatePlot(f2,map,p_predict,d_meas,p_update,dx,dy,dtheta,area,n_discretizz_theta,rel_agent,RFID,1)
   F(fig_idx) = getframe(gcf) ;
   fig_idx = fig_idx + 1;
   drawnow();

        
   t = t + dt; 
end
toc


% create the video writer with 1 fps
writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = 1;
  % set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);