clc;
clear all;
close all force;


%% Plot distribuzioni create
u_p = Distribution_1D.unit_pulse(0);
t_d = Distribution_1D.triangle(0,5);
g_d = Distribution_1D.gaussian(0,5,5);

f1 = figure(1);
hold on;
u_p.show('r--*',f1,"");
t_d.show('b--*',f1,"");
g_d.show('g--*',f1,"");
legend('pulse','triangular','gaussian');


%% Esempio 1D 
tag1 = 'fusione1';
tag2 = 'fusione2';
position = Distribution_1D.triangle(10,1);
controls = [ Distribution_1D.triangle(40, 10), Distribution_1D.triangle(70, 10) ];
measurements = [ Distribution_1D.triangle(60, 10), Distribution_1D.triangle(140, 20) ];

f1 = figure(2);
hold on;
position.show('b--*',f1,tag2);
correction = position;
for i =1:length(controls)
   % Prediction and update
   prediction = Distribution_1D.convolve(correction,controls(i));
   correction = Distribution_1D.multilpy(prediction,measurements(i));

   % Plots
   prediction.show('r--*',f1,tag1);
   measurements(i).show('g--*',f1,tag1);
   correction.show('b--*',f1,tag2);  

   pause();   
end