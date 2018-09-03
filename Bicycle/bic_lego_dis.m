function [x_1,y_1,theta_1] = bic_lego_dis(x_0, y_0 ,theta_0, v_0, alpha_0,Ts)
% X(k+1) = A*X(K) + B*U(K)


% current state: x_0, y_0, theta_0;
% current input: v_0, alpha_0;

%Test
%x_0 = 0; y_0 = 0; theta_0 = 0; v_0 = 5; alpha_0 = 10/180*pi;

l = 3;

Ac = [0, 0, -v_0*sin(theta_0);
          0, 0, v_0*cos(theta_0);
          0, 0, 0];
Bc = [cos(theta_0), 0;
          sin(theta_0), 0; 
          tan(alpha_0)/l, v_0/l/(cos(alpha_0))^2];
csys = ss(Ac,Bc,[],[]); % continue state-space lin model
dsys = c2d(csys,Ts); %discrete model
Ad = dsys.a;
Bd = dsys.b;

X_1= Ad*[x_0;y_0;theta_0] + Bd*[v_0;alpha_0];

x_1 = X_1(1);
y_1 = X_1(2);
theta_1 = X_1(3);
return
