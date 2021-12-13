function [y,path] = generateSampleTransition(x,u)

alpha = ones(1,6)*0.001;
% alpha = ones(1,6)*0.00001;
% alpha = zeros(1,6);
alpha = sqrt(alpha);

dt = 0.025:0.025:0.25;
% dt = 0.1:0.1:1;
if u == 1
    x(3) = pi/2; u_t = [pi/2,pi]; 
elseif u==2 
    x(3) = pi/2; u_t = [pi/2,-pi];
elseif u == 3
    x(3) = -pi/2; u_t = [pi/2,pi];
else
    x(3) = -pi/2; u_t = [pi/2, -pi];
end

% if u==1
%     u_t = [pi/2,pi];
% else
%     u_t = [pi/2,-pi];
% end
x_t1 = x;

path = zeros(3,length(dt));

n1 = randn*(alpha(1)*u_t(1)^2 + alpha(2)*u_t(2)^2);
n2 = randn*(alpha(3)*u_t(1)^2 + alpha(4)*u_t(2)^2);
n3 = randn*(alpha(5)*u_t(1)^2 + alpha(6)*u_t(2)^2);

for i=1:10
    v_hat = u_t(1) + n1;
    w_hat = u_t(2) + n2;
    gamma = n3;
    r_hat = v_hat/w_hat;

    x_t = zeros(1,3);
    x_t(1) = x_t1(1) - r_hat*sin(x_t1(3)) + r_hat*sin(x_t1(3) + w_hat*dt(i));
    x_t(2) = x_t1(2) + r_hat*cos(x_t1(3)) - r_hat*cos(x_t1(3) + w_hat*dt(i));
    x_t(3) = x_t1(3) + w_hat*dt(i) + gamma*dt(i);
    
    path(:,i) = x_t;
end

y = x_t(1:2);
% y = x_t;
end