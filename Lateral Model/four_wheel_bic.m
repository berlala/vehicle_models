%% 4 Wheel-steering Lateral control bicycle model
%% Ref: Coordinated control of the steering
%system and the distributed motors for
%comprehensive optimization of the
%dynamics performance and the energy
%consumption of an electric vehicle
% YuTong Li, Part D,2017
%% Math  
%x_dot = A*X +B*U
%X = [beta delta];
%Beta is the steering angle on the gravity point;  Delta is the yaw rate
%U = [theta_f, theta_r];

%%
% parameters
%Vx = 2;

Ts = 0.05;
T = 0:Ts:10;

%input
delta_data = [0, ones(1,(length(T)-1)/4)*0,  ones(1,(length(T)-1)/4) * 10/180*pi, ones(1,(length(T)-1)/4) * -10/180*pi, ones(1,(length(T)-1)/4)*0, ];      % in rad
delta_f= [T;delta_data]';
delta_r = [T; zeros(1,length(T))]';


sim bic_yutong.slx;

%% Result and post-process
figure(1)
plot(T, delta_data); hold on;
plot(T, beta_state); 
legend('Front Wheel Steering Cmd[rad]','Gravity-Center Steering Response')




