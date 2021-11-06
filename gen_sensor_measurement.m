% given the true distance between robot and obstable, generate a
% measurement
% input: 
%   x - true distance from robot to obstable in the direction of bearing
% output: 
%   y - a measurement 
function y = gen_sensor_measurement(x)

max = 5; % maximum sensor reading

% generate model for the case where distance exceeds max
a = 0.05; b = 0.2; 
xi = rand; 
if xi < a, z = 0; 
elseif a < xi && xi < b, z = (xi - a)*max/(b-a); 
else, z = max; 
end

s = 0.1; % variance of gaussian 
w = 0.1; % prob. of wrong reading vs. gaussian reading

% if true distance exceed max reading distance, draw from error
% distribution
if x >= max, y = z;
% if not, draw gaussian / error mixture
else
    if rand < w, y = z;
    else, y = normrnd(x,s);
        if y < 0; y = 0; end
        if y > max; y = max; end
    end
end