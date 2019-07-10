clc;
clear all;
close all;

dt = 0.01; 

folder = './mov_circ';

x_med = [];
y_med = [];
t_med = [];
x_cov = [];
y_cov = [];
t_cov = [];
for i=1:4
    load([folder,'/',num2str(i)]);
    x_error = [];
    y_error = [];
    t_error = [];
    for k=1:numel(filters_trace)/2
        s_ukf = filters_trace{k};
        error = s_ukf - real_trace;
        error(3,:) = rem(error(3,:)+pi,2*pi)-pi;
        x_error(end+1,:) = error(1,:);
        y_error(end+1,:) = error(2,:);
        t_error(end+1,:) = error(3,:);
    end
    
    x_med(i,:) = mean(x_error);
    y_med(i,:) = mean(y_error);
    t_med(i,:) = mean(t_error);
    
    for k=1:numel(x_med(i,:))
        x_cov(i,k) = cov(x_error(:,k)');
        y_cov(i,k) = cov(y_error(:,k)');
        t_cov(i,k) = cov(t_error(:,k)')*180/pi;
    end
end

figure();
hold on;
plts = plot(1,1);
for i=1:4
    switch i
        case 1
            style = 'k';
        case 2
            style = 'g--';
        case 3
            style = 'r:';
        case 4
            style = 'b-.';
    end
    
    for k=1:3
        switch k
            case 1
                covarianza = x_cov(i,:);
                media = x_med(i,:);
                limiti_y = [-1,1];
                ascissa_tag = 'e_x [m]';
            case 2
                covarianza = y_cov(i,:);
                media = y_med(i,:);
                limiti_y = [-1,1];
                ascissa_tag = 'e_y [m]';
            case 3
                covarianza = t_cov(i,:);
                media = t_med(i,:);
                limiti_y = [-1,1];
                ascissa_tag = 'e_{\theta} [deg]';
        end  
        
        subplot(3,2,2*k);
        hold on;
        time = (0:(numel(media)-1))*dt;
        if k == 1
        plts(i) = stairs(time,media+sqrt(covarianza),style,'linewidth',3);
        else
            stairs(time,media+sqrt(covarianza),style,'linewidth',3);
        end
        stairs(time,media-sqrt(covarianza),style,'linewidth',3);
        if k==3
            xlabel('time [s]');
        else
            set(gca,'Xticklabel',[]) 
        end
        ylabel(ascissa_tag);
        ylim(limiti_y);
        set(gca,'FontSize',24)
        box on
    end
end
subplot(3,2,2);
legend(plts,{'1 RFID','2 RFID','3 RFID','4 RFID'},'Orientation','horizontal',...
    'location','northoutside');

%%
l = 5;
RFID = [0, 0,  l, -l
        l, -l, 0,  0]; % Position of RFID
    
for k= 1:4
    load([folder,'/',num2str(k)]);
    switch k
        case 1
            style = 'k';
        case 2
            style = 'g--';
        case 3
            style = 'r:';
        case 4
            style = 'b-.';
    end
    
    subplot(2,2,1);
    hold on
    for i=1:4
        plot(RFID(1,i),RFID(2,i),'sk','linewidth',3);            
    end
    stairs(filters_trace{i}(1,:),filters_trace{i}(2,:),style,'linewidth',3);    
    ylabel('y [m]')
    xlabel('x [m]')
    set(gca,'FontSize',24)
    axis equal
    box on
    subplot(2,2,3);
    hold on
    for i=1:4
        plot(RFID(1,i),RFID(2,i),'sk','linewidth',3);        
    end
    stairs(filters_trace{i}(1,:),filters_trace{i}(2,:),style,'linewidth',3);        
    ylabel('y [m]')
    xlabel('x [m]')
    axis equal
    set(gca,'FontSize',24)
    box on
end