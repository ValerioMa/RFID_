clc;
clear all;
close all force;


%% Plot distribuzioni create
% u_p = Distribution_3D.unit_pulse([0;0;0]);
% f1 = figure(1);
% hold on;
% u_p.show(f1);


t_d = Distribution_3D.gaussian([-5.5;0;0],diag([5,5,5]), [2;2;6]);
f1 = figure(1);
t_d.show(f1);

% t_g = Distribution_3D.gaussian([0;0;0],diag([5,5,5]), [2;2;6]);
% t_g1 = Distribution_3D.gaussian([4;4;4],diag([5,5,5]), [2;2;6],3);
t_g1 = Distribution_3D.unit_pulse([4;0;0]);

f2 = figure(2);
t_g1.show(f2);


c = Distribution_3D.convolve(t_d, t_g1);
%c = Distribution_3D.multilpy(t_g, t_g1);
f3 = figure(3);
c.show(f3);
% % %% Esempio 1D 
% % tag1 = 'fusione1';
% % tag2 = 'fusione2';
% % position = Distribution_1D.triangle(10,1);
% % controls = [ Distribution_1D.triangle(40, 10), Distribution_1D.triangle(70, 10) ];
% % measurements = [ Distribution_1D.triangle(60, 10), Distribution_1D.triangle(140, 20) ];
% % 
% % f1 = figure(2);
% % hold on;
% % position.show('b--*',f1,tag2);
% % correction = position;
% % for i =1:length(controls)
% %    % Prediction and update
% %    prediction = Distribution_1D.convolve(correction,controls(i));
% %    correction = Distribution_1D.multilpy(prediction,measurements(i));
% % 
% %    % Plots
% %    prediction.show('r--*',f1,tag1);
% %    measurements(i).show('g--*',f1,tag1);
% %    correction.show('b--*',f1,tag2);  
% % 
% %    pause();   
% % end