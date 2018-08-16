%% Simulation for bicycle model
%%
clear;clc
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

delta_data = ones(1,length(T)) * 10/180*pi; % in rad
delta_f= [T;delta_data]';
%%
sim bic.slx
%%
plot(x_state,y_state)