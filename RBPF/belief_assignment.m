function p = belief_assignment(z,x,d,param)
y = min(x,param(3));
if x == param(3) 
    d(1) = 0; d(2) = 0;
end
if 0 <= z && z < y - d(1) - d(2)
    p = 0.3;
elseif y - d(1) - d(2) <= z && z < y - d(2)
    p = 0.4/d(1)*(z - (y - d(1) - d(2))) + 0.3;
elseif y - d(2) <= z && z < min(y + d(2), param(3))
    p = 0.7;
elseif y + d(2) <= z && z < min(y + d(1) + d(2), param(3))
    p = -0.4/d(1)*(z - (y + d(2))) + 0.7;
else
    p = 0.5;
end
