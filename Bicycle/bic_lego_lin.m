function [x_1,y_1,theta_1] = bic_lego_lin(x_0, y_0 ,theta_0, v_0, alpha_0,Ts)
% X_dot = A*X + B*U

% current state: x_0, y_0, theta_0;
% current input: v_0, alpha_0;

%Test
%x_0 = 0; y_0 = 0; theta_0 = 0; v_0 = 5; alpha_0 = 10/180*pi;
l = 3;

Ac = [0 0 -v_0*sin(theta_0);
          0 0 v_0*cos(theta_0);
           0 0 0];
Bc = [cos(theta_0) 0;
          sin(theta_0) 0; 
          tan(alpha_0)/l, v_0/l/(cos(alpha_0))^2];
csys = ss(Ac,Bc,[],[]);
%X_1= Ac*[x_0;y_0;theta_0] + Bc*[v_0;alpha_0]*Ts;
u = [0  0 v_0; 0 0 alpha_0]';
X_0 = [x_0; y_0 ; theta_0];
T_in = [0,Ts, Ts*2]'; 

   [~,t,X] =  lsim(csys,u,T_in,X_0);

x_1 = X(3,1);
y_1 =X(3,2);
theta_1 =X(3,3);

return
