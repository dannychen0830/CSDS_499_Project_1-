% given the true distance between robot and obstable, generate a
% measurement
% input: 
%   x - true distance from robot to obstable in the direction of bearing
%   param - parameters for error [a, b, max, s, w]
%            a, b - linear function, max - max reading, s - std, w - error
%            to gaussian weight
% output: 
%   y - a measurement 

% param = [0.05, 0.2, 5, 0.1, 0.15]
function y = gen_sensor_measurement(x, param)
a = param(1); b = param(2); max = param(3); s = param(4); w = param(5); 

% generate model for the case where distance exceeds max
xi = rand; 
if xi < a, z = 0; 
elseif a < xi && xi < b, z = (xi - a)*max/(b-a); 
else, z = max; 
end

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