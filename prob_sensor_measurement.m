% belief assignment given true distance

function p = prob_sensor_measurement(z, x, param)
a = param(1); b = param(2); max = param(3); s = param(4)*1.5; w = param(5); 
if z == 0,
    p = a*w + normcdf(0,x,s)*(1-w);
elseif z == max,
    p = (1-b)*w + normcdf(max-x,x,s)*(1-w);
else 
    p = (a + (b-a)/max*z)*w + (normcdf(z+s*0.1,x,s)-normcdf(z,x,s))*(1-w);
end
end