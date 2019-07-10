clc;
clear all;
close all;

lim_x = [0,100];
lim_y = [0.5,9.5];

ds = 0.01; % path discretization


num_path = 500;
paths = {};
for k=1:num_path
    path = [];
    x = lim_x(1);
    y = rand(1)*diff(lim_y) + lim_y(1);
    while x < lim_x(2)
        max_step_lenght = 10;
        max_jump = 2;
        dl = 10; %rand(1)*max_step_lenght;
        x_end = x + dl;
        y_end = lim_y(2)+1;
        while y_end<lim_y(1) || y_end>lim_y(2)
            dh = 2*(rand(1)-0.5)*max_jump;
            y_end = y + dh;
        end

        l_ipot = sqrt(dl*dl+dh*dh);

        di = (ds:ds:l_ipot)/l_ipot;

        dx = dl*di;
        dy = dh*di;

        xs = x + dx;
        ys = y + dy;
        thetas = atan2(dh,dl)*ones(size(xs));
        curvatures = zeros(size(xs));
        path = [path,[xs;ys;thetas;curvatures]];
        x = path(1,end);
        y = path(2,end);
    end
    path = path(:,path(1,:)<lim_x(2));    
    paths{k} = [path];
end

figure();
hold on;
for k=1:numel(paths)
    plot(paths{k}(1,:),paths{k}(2,:));
end

save('percorsi','paths');