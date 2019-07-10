clc;
clear all;
close all;
addpath(genpath(pwd));

% Define 2 RFID position for the calculation we assume that they have x = 0
l = 5;
RFID = [0, 0;
        -l, l];
v_norm = 1;
omega = 0;

% Define measure from RFID


alpha = acos(0.7061)/2;
beta = acos(0.9486);
% alpha = pi/2 - k;
% beta  = pi/2 + k;
% measure = v_norm*[cos(alpha),cos(beta)]

measure = rfidReadings([10;0;-pi/4],v_norm,RFID);  

% Initialize the figure
figure(1); clf;
hold on;
grid on;
plot(RFID(1,1),RFID(2,1),'g.','markersize',20);
plot(RFID(1,2),RFID(2,2),'r.','markersize',20);

% Define the possible solution
segni(1,:)     = [ -1, -1];
segni(end+1,:) = [  1,  1];
segni(end+1,:) = [ -1,  1];
segni(end+1,:) = [  1, -1];


% Find all the point with the same measure
circonference = zeros(6,0);  % [x;y;R;delta0] 
xcs = {};
ycs = {};
for i = 1:size(segni,1)
    
    sol = segni(i,:).*[ acos(measure(1)/v_norm),  acos(measure(2)/v_norm)];
    
    circonf_angle = (sol(1)-sol(2)); % angolo alla circonferenza
    % Muovendo questo angolo alla circonferenza otteniamo tutti i punti con
    % sesso circonf_angle --> tutti i punti di tale circonferenza

    % Per ricavare circonferenza troviamo il vertice del triangolo isoscele il
    % cui angolo sara dato da circonf_angle e gli altri 2 da base_angle = (pi - circonf_angle)/2;

    % L'orientazione del robot nel vertice del triangolo isoscele sar'
%     theta0 = (sol(2)+sol(1))/2;
    % La base del triangolo isoscele sar'
    triangle_base =  RFID(:,2) - RFID(:,1);
    triangle_base_norm = norm(triangle_base);
    triangle_base_versor = triangle_base/triangle_base_norm;
    
    
    % Punto medio base
    mean_base_point = RFID(:,1) + triangle_base/2;

    % Cerchiamo altezza triangolo isoscele di base norm_delta
    triangle_altezza_versor = [-triangle_base_versor(2);triangle_base_versor(1)];
    triangle_altezza_norm = (triangle_base_norm/2)/tan(circonf_angle/2);

    % Fint the circle 
    triangle_vertex = mean_base_point + triangle_altezza_versor*triangle_altezza_norm; 
    triangle_point = [RFID(:,1),RFID(:,2),triangle_vertex];    
    [R,xcyc] = fit_circle_through_3_points(triangle_point');

    angolo0_al_RFID1 = atan2(RFID(2,1)-triangle_vertex(2), RFID(1,1) - triangle_vertex(1));
    angolo0_al_RFID2 = atan2(RFID(2,2)-triangle_vertex(2), RFID(1,2) - triangle_vertex(1));

    % controllare che angolo0_al_RFID2 - angolo0_al_RFID1 = circonf_angle
    theta0 = angolo0_al_RFID1 + sol(1);
    delta0 = sol(1); %theta0  - angolo0_al_RFID1; % point where start to drow the circle
    
    % Find the angle
    angolo0_al_centro_RFID1 = atan2(RFID(2,1)-xcyc(2), RFID(1,1) - xcyc(1));
    angolo0_al_centro_RFID2 = atan2(RFID(2,2)-xcyc(2), RFID(1,2) - xcyc(1));
    angolo0_al_centro_refPoint = atan2(triangle_vertex(2)-xcyc(2), triangle_vertex(1) - xcyc(1));
    
    % 
    while angolo0_al_centro_RFID2<0
        angolo0_al_centro_RFID2 = angolo0_al_centro_RFID2 + 2*pi;
    end
    while angolo0_al_centro_RFID2>2*pi
        angolo0_al_centro_RFID2 = angolo0_al_centro_RFID2 - 2*pi;
    end
    
    while angolo0_al_centro_refPoint<0
        angolo0_al_centro_refPoint = angolo0_al_centro_refPoint + 2*pi;
    end
    while angolo0_al_centro_refPoint>2*pi
        angolo0_al_centro_refPoint = angolo0_al_centro_refPoint - 2*pi;
    end
    
    if angolo0_al_centro_refPoint<angolo0_al_centro_RFID2
       max_theta = angolo0_al_centro_RFID2;
       while angolo0_al_centro_RFID1<angolo0_al_centro_RFID2
           angolo0_al_centro_RFID1 = angolo0_al_centro_RFID1 + 2*pi;
       end
       while angolo0_al_centro_RFID1>angolo0_al_centro_RFID2
           angolo0_al_centro_RFID1 = angolo0_al_centro_RFID1 - 2*pi;
       end
       min_theta = angolo0_al_centro_RFID1;
    else
       min_theta = angolo0_al_centro_RFID2;
       while angolo0_al_centro_RFID1>angolo0_al_centro_RFID2
           angolo0_al_centro_RFID1 = angolo0_al_centro_RFID1 - 2*pi;
       end
       while angolo0_al_centro_RFID1<angolo0_al_centro_RFID2
           angolo0_al_centro_RFID1 = angolo0_al_centro_RFID1 + 2*pi;
       end
       max_theta = angolo0_al_centro_RFID1;
    end
    
    theta_start = min_theta;
    theta_end = max_theta;
    
    circonference(:,end+1) = [xcyc(1);xcyc(2);R;delta0;theta_start;theta_end];
    system_measure([triangle_vertex(1);triangle_vertex(2);theta0],RFID,v_norm)    
    % Find circle points
    thetas = linspace(theta_start,theta_end,100);
    xc = xcyc(1) + R*cos(thetas);
    yc = xcyc(2) + R*sin(thetas);

%     measure in this point
%     system_measure([triangle_vertex(1);triangle_vertex(2);theta0],RFID,v_norm)
    % Draw the circle
    figure(1); 
    plot(triangle_point(1,:),triangle_point(2,:),'k.','markersize',5);
    plot(xc,yc,'b--');

    xcs{i} = xc;
    ycs{i} = yc;
end

%% Step 0, spread point on the circonference
point_distance = 0.01;
step0_data = {};
index_memo = [];
for i=1:size(circonference,2)
    % Extract circle parameters
    circ = circonference(:,i);
    x = circ(1);
    y = circ(2);
    R = circ(3);
    delta0 = circ(4);
    theta_start = circ(5);
    theta_stop  = circ(6);
    
    % Troviamo punti del cerchio    
    dangles = point_distance/R;
    angles = theta_start:dangles:theta_stop; %(theta_start+2*pi) %
    angolo0 = atan2(l,x);
    delta = (angolo0-theta_start);
    delta/dangles
    xs = x + R*cos(angles);
    ys = y + R*sin(angles);
    
    
    angolo0_al_RFID1 = atan2(RFID(2,1) - ys,RFID(1,1) - xs);

    theta1 = delta0 + angolo0_al_RFID1;
    v1 = v_norm*[cos(theta1);sin(theta1)];
    
    plot(xs,ys,'k.','markersize',20);
    solution_data = zeros(11,0);
    
    for k=1:size(v1,2)
        %% If is too close to te RFID skipp
        p_k = [xs(k);ys(k)];
        to_skip = false;
        for kk=1:size(RFID,2)
           p_RFID = RFID(:,kk); 
           if norm(p_k-p_RFID)<(point_distance/2)
               to_skip = true;
           end
        end        
        if not(to_skip)
            measures = system_measure([xs(k);ys(k);theta1(k)],RFID,v_norm);
            [dmeasures,ddmeasure] = system_measure_derivative([xs(k);ys(k);theta1(k)],RFID,v_norm,omega);
            
            solution_data(:,end+1) = [xs(k),ys(k),theta1(k),v1(:,k)',measures',dmeasures',ddmeasure']';
            plot(xs(k)+[0,v1(1,k)]*0.3,ys(k)+[0,v1(2,k)]*0.3,'k-','linewidth',2);
        end
    end
%     plot3(solution_data(1,:),solution_data(2,:),solution_data(6,:),'g');
%     plot3(solution_data(1,:),solution_data(2,:),solution_data(7,:),'r');
    
    % Store the solution
    step0_data{i} = solution_data;
end
[solution_data(6,1),solution_data(7,1)]

%% Step 0.5 compute speed
for ii=1:numel(step0_data)
    data = step0_data{ii};
    figure();
    hold on;
    title(['speed', num2str(ii)]);
    plot(data(8,:),'r');
    plot(data(9,:),'k');
end

% % for ii=1:numel(step0_data)
% %     data = step0_data{ii};
% %     figure();
% %     hold on;
% %     title(['dspeed', num2str(ii)]);
% %     plot(data(10,:),'r');
% %     plot(data(11,:),'k');
% % end


%% Step 1
step1_data = {};
for ii=1:numel(step0_data)
    data = step0_data{ii};
    solution_data = zeros(7,size(data,2));
    figure(1);
    for i=1:size(data,2)
        x1 = data(1,i);
        y1 = data(2,i);
        theta1 = data(3,i);
        v1 = data(4:5,i);

        x1_new = x1 + v1(1);
        y1_new = y1 + v1(2);    

        measures = system_measure([x1_new;y1_new;theta1],RFID,v_norm);
        solution_data(:,i) = [x1_new,y1_new,theta1,v1',measures']';
        
        
%         plot(x1_new,y1_new,'g.','markersize',10);
%         plot(x1_new+[0,v1(1)]*0.1,y1_new+[0,v1(2)]*0.1,'r-');        

    end
    plot(solution_data(1,:),solution_data(2,:),'m');
    
    % Store the solution
    step1_data{ii} = solution_data;
    
    figure();
    hold on;
    plot(solution_data(6,:),'r');
    plot(solution_data(7,:),'k');
    step1_data{ii} = solution_data;
end



