clc;
clear all;
close all force;


%% Plot distribuzioni create
u_p = Distribution.unit_pulse(0);
% t_d = Distribution.triangle(0,5);
% g_d = Distribution.gaussian(0,5,5);
% 
% f1 = figure(1);
% hold on;
% u_p.show(f1);
% t_d.show(f1);
% g_d.show(f1);



% %% Esempio 
tag1 = 'fusione';
tag2 = 'fusione';
position = Distribution.triangle(10,1);
controls = [ Distribution.triangle(40, 10), Distribution.triangle(70, 10) ];
measurements = [ Distribution.triangle(60, 10), Distribution.triangle(140, 20) ];

f1 = figure(1);
hold on;
position.show(f1);
correction = position;
for i =1:1+0*length(controls)
   prediction = Distribution.convolve(correction,controls(i));
   prediction.show(f1,tag1);
   measurements.show(f1,tag1);
   correction = Distribution.multilpy(prediction,measurements(i));
   correction.show(f1,tag2); 
   pause(0.5);
   
end