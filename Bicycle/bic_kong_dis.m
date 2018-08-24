function [x_1,y_1,theta_1,v_1] = bic_kong_dis(x_0, y_0 ,psi_0, v_0, a_0, alpha_0,Ts)
% X(k+1) = A*X(K) + B*U(K)
lf = 1.4;
lr =1.6;

% current state: x_0, y_0, theta_0;
% current input: v_0, alpha_0;

%Test
%x_0 = 0; y_0 = 0; theta_0 = 0; v_0 = 5; alpha_0 = 10/180*pi;

K = lr/(lf+lr);
N = K/(1+K^2*(tan(alpha_0))^2)*1/(cos(alpha_0))^2; %beta_dot
beta_0 = atan(K*tan(alpha_0));

Ack = [0, 0, -v_0*sin(psi_0+beta_0), cos(psi_0+beta_0); 
            0 ,0, v_0*cos(psi_0+beta_0), sin(psi_0+beta_0) ;
            0 ,0 ,0 , 1/lr*sin(beta_0);
            0,0, 0, 0];
 Bck = [0, -v_0*sin(psi_0+beta_0)*N;
            0,  v_0*cos(psi_0+beta_0)*N;
            0 , v_0/lr*cos(beta_0)*N;
            1,0];

csys = ss(Ack,Bck,[],[]);
dsys = c2d(csys,Ts); %discrete model
Ad = dsys.a;
Bd = dsys.b;

X_1= Ad*[x_0;y_0;psi_0;v_0] + Bd*[a_0;alpha_0];

x_1 = X_1(1);
y_1 = X_1(2);
theta_1 = X_1(3);
v_1 = X_1(4);
return
