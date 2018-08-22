%% Simulation for bicycle models
%% 
% This script try to test the different bicycle models
%1) bic_kong.slx: [Kong Model]Kinematic and Dynamic Vehicle Models for
%Autonomous Driving Control Design; input[a, theta]£»
%2) bic_lego.slx: traditional bicycle model. input[v, theta]£»

%%
clear;clc;close all;
%%
Ts = 0.01;
T = 0:Ts:10;
%% initial state
% secnario design: 
% inital speed is 5m/s, and it keep constant. 
% the vehicle start to turn left by 10 deg and then turn right by 10 deg.
x_init = 0;
y_init = 0;
psi_init = 0;

v_0 = 5; % initial speed

%% input
acc_data = zeros(1,length(T));
acc = [T;acc_data]';

v_data = ones(1,length(T))*5;
v = [T;v_data]';

delta_data = [ones(1,length(T)/2+0.5) * 10/180*pi, ones(1,length(T)/2-0.5) * -10/180*pi];      % in rad
delta_f= [T;delta_data]';
%%
sim bic_kong.slx

figure(1)
plot(x_state,y_state)
hold on
% figure(2)
% plot(v_state)

clear x_state y_state

%%
sim bic_lego.slx

figure(1)
plot(x_state,y_state)
hold on

grid on;
%% linearization of bic_lego model
x_h = []; y_h = []; x_0 = x_init; y_0 = y_init; psi_0 = psi_init; 
for i = 1: length(T)

    x_h = [x_h;x_0];
    y_h = [y_h;y_0];
 
v_0 = v(i,2);
alpha_0 = delta_f(i,2);
    
    
 [x_1,y_1,psi_1] = bic_lego_lin(x_0, y_0 ,psi_0, v_0, alpha_0,Ts);
 x_0 =x_1;
 y_0 = y_1;
 psi_0 = psi_1;

end

figure(1)
plot(x_h,y_h)
hold on
%% Lego discretize
x_h = []; y_h = []; x_0 = x_init; y_0 = y_init; psi_0 = psi_init; 
for i = 1: length(T)

    x_h = [x_h;x_0];
    y_h = [y_h;y_0];
 
v_0 = v(i,2);
alpha_0 = delta_f(i,2);
    
    
 [x_1,y_1,psi_1] = bic_lego_dis(x_0, y_0 ,psi_0, v_0, alpha_0,Ts);
 x_0 =x_1;
 y_0 = y_1;
 psi_0 = psi_1;

end

figure(1)
plot(x_h,y_h)
hold on
legend('Kong Model','Lego Model','Lego Model Lin','Lego Model Dis')
