clc;
clear all;
close all force;


%% Plot distribuzioni create
u_p = Distribution.unit_pulse([0,0]);
g_d = Distribution.gaussian([0,0],eye(2,2),[5,5]);

f1 = figure(1);
hold on;
u_p.show('r--*',f1,"");
t_d.show('b--*',f1,"");
g_d.show('g--*',f1,"");
legend('pulse','triangular','gaussian');

