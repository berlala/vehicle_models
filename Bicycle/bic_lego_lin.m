function [x_1,y_1,theta_1] = bic_lego_lin(x_0, y_0 ,theta_0, v_0, alpha_0,v_1,alpha_1,Ts)
% X_dot = A*X + B*U
% v_0, alpha_0 is the last step input;
% v_1, aplha_1 is the current input


% current state: x_0, y_0, theta_0;
% current input: v_0, alpha_0;

%Test
%x_0 = 0; y_0 = 0; theta_0 = 0; v_0 = 5; alpha_0 = 10/180*pi;

%Ac,Bc have been linearized.
l = 3;
Ac = [0 0 -v_0*sin(theta_0);
          0 0 v_0*cos(theta_0);
           0 0 0];
Bc = [cos(theta_0) 0;
          sin(theta_0) 0; 
          tan(alpha_0)/l, v_0/l/(cos(alpha_0))^2];
csys = ss(Ac,Bc,[],[]);
%X_1= Ac*[x_0;y_0;theta_0] + Bc*[v_0;alpha_0]*Ts;
u      = [v_0   v_1; alpha_0  alpha_1];
X_0 = [x_0; y_0 ; theta_0];
T_in = [0,Ts]'; 

[~,t,X] =  lsim(csys,u,T_in,X_0);

x_1 = X(2,1);
y_1 =X(2,2);
theta_1 =X(2,3);

return
