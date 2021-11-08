% get the probability of the sensor measurement given position and map
% input: 
%   z - the measurement 
%   x - true distance from robot to obstable in the direction of bearing
%   param - parameters for error [a, b, max, s, w]
%            a, b - linear function, max - max reading, s - std, w - error
%            to gaussian weight
%           suggested: [0.0200, 0.2000, 5.0000, 0.1000, 0.1000]
% output: 
%   p - the probability of getting the sensor measurement given robot
%   position and map
function p = prob_sensor_measurement(z, x, param)
a = param(1); b = param(2); max = param(3); s = param(4)*1.5; w = param(5); 
if z == 0,
    p = a*w + normcdf(0,x,s)*(1-w);
elseif z == max,
    if x < max
        p = (1-b)*w + normcdf(max-x,x,s)*(1-w);
    else
        p = (1-b-a);
    end
else 
    p = (a + (b-a)/max*z)*w + (normcdf(z+s*0.05,x,s)-normcdf(z,x,s))*(1-w);
end
end