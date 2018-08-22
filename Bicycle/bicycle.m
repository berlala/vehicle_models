%% Simulation for bicycle models
%% 
% This script try to test the different bicycle models
%1) bic_kong.slx: [Kong Model]Kinematic and Dynamic Vehicle Models for
%Autonomous Driving Control Design; input[a, theta]
%2):bic_lego.slx: traditional bicycle model. input[v, theta]

%%
clear;clc;close all;
%%
Ts = 0.01;
T = 0:Ts:10;
%% initial state
x_init = 0;
y_init = 0;
psi_init = 0;

v_0 = 5;

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
legend('Kong Model','Lego Model')
grid on;
