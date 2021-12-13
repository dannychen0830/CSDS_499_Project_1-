function x = sample_motion_model_odometry(u, x_0, alpha)
alpha = sqrt(alpha);
drot1 = atan2(u(5) - u(2), u(4) - u(1)) - u(3);
dtrans = sqrt((u(4)-u(1))^2 + (u(5)-u(2))^2);
drot2 = u(6) - u(3) - drot1;

drot1hat = drot1 + randn*(alpha(1)*drot1^2 + alpha(2)*dtrans^2);
dtranshat = dtrans + randn*(alpha(4)*drot1^2 + alpha(4)*drot2^2 + alpha(3)*dtrans^2);
drot2hat = drot2 + randn*(alpha(1)*drot1^2 + alpha(2)*dtrans^2);

x = zeros(1,3);
x(1) = x_0(1) + dtranshat*cos(x_0(3) + drot1hat);
x(2) = x_0(2) + dtranshat*sin(x_0(3) + drot1hat);
x(3) = x_0(3) + drot1hat + drot2hat;
end
